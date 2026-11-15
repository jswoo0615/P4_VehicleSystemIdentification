#include "p4/io/IONIQ5CsvReader.hpp"
#include "p4/identification/GolfDataset.hpp"

#include <cmath>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace {
using Segment=p4::io::ModelACsvSamples;
using File=std::vector<Segment>;
struct Fit { double tau=0, gain=0, bias=0, kv=0, wheelbase=3.0, sse=INFINITY; std::size_t n=0; };
double input(double delta,double speed,double gain,double kv,bool speed_model,double wheelbase) {
    return speed_model ? delta*speed/(wheelbase+kv*speed*speed) : gain*delta;
}
Fit evaluate(const std::vector<File>& files,const Fit& fit,bool speed_model,double trim_s=0) {
    Fit out=fit; out.sse=0; out.n=0;
    for(const auto& file:files) for(const auto& s:file) {
        double p=s.yaw_rad_s.front();
        for(std::size_t i=1;i<s.time_s.size();++i) {
            const double a=std::exp(-(s.time_s[i]-s.time_s[i-1])/fit.tau);
            p=a*p+(1-a)*(input(s.steering_rad[i-1],s.speed_m_s[i-1],fit.gain,fit.kv,speed_model,fit.wheelbase)+fit.bias);
            if(s.time_s[i]-s.time_s.front()>=trim_s) {
                const double e=s.yaw_rad_s[i]-p;
                out.sse+=e*e; ++out.n;
            }
        }
    }
    return out;
}
Fit fit(const std::vector<File>& files,bool speed_model,double wheelbase=3.0) {
    Fit best;
    // A searches gain and bias; A-Speed uses the caller's wheelbase and
    // searches nonnegative kv. No held-out file enters this search.
    for(int ti=0;ti<=80;++ti) {
        const double tau=.03*std::pow(100.0,ti/80.0);
        for(int ki=0;ki<=(speed_model?60:0);++ki) {
            const double kv=ki*.0025;
            double qq=0,qh=0,hh=0,qy=0,hy=0;
            for(const auto& file:files) for(const auto& s:file) {
                double p=s.yaw_rad_s.front(),q=0,h=0;
                for(std::size_t i=1;i<s.time_s.size();++i) {
                    const double a=std::exp(-(s.time_s[i]-s.time_s[i-1])/tau);
                    p*=a;
                    const double u=input(s.steering_rad[i-1],s.speed_m_s[i-1],1,kv,speed_model,wheelbase);
                    q=a*q+(1-a)*u; h=a*h+(1-a);
                    const double y=s.yaw_rad_s[i]-p-(speed_model?q:0);
                    if(!speed_model) {qq+=q*q; qh+=q*h; qy+=q*y;}
                    hh+=h*h; hy+=h*y;
                }
            }
            Fit candidate; candidate.tau=tau; candidate.kv=kv; candidate.wheelbase=wheelbase;
            if(speed_model) candidate.bias=hy/hh;
            else {
                const double det=qq*hh-qh*qh;
                if(det<=1e-12*qq*hh) continue;
                candidate.gain=(qy*hh-hy*qh)/det;
                candidate.bias=(hy*qq-qy*qh)/det;
            }
            candidate=evaluate(files,candidate,speed_model);
            if(candidate.sse<best.sse) best=candidate;
        }
    }
    if(!std::isfinite(best.sse)) throw std::runtime_error("No valid Model A fit");
    return best;
}
double rmse(const Fit& x) {return std::sqrt(x.sse/x.n);}
std::vector<File> excluding(const std::vector<File>& files,std::size_t held) {
    std::vector<File> out;
    for(std::size_t i=0;i<files.size();++i) if(i!=held) out.push_back(files[i]);
    return out;
}
}
int main(int argc,char** argv) {
    try {
        if(argc>=2 && std::string(argv[1])=="--golf-kfold") {
            if(argc!=4) throw std::runtime_error("Usage: model_a_speed_cv --golf-kfold dataset_dir result.json");
            const auto dataset=p4::identification::GolfDataset::load(argv[2]);
            const auto& unique=dataset.paths();
            const auto& duplicates=dataset.duplicates();
            const auto& files=dataset.files();
            constexpr std::size_t folds=p4::identification::GolfDataset::fold_count;
            std::ofstream out(argv[3]);
            if(!out) throw std::runtime_error("Cannot write JSON result");
            out<<std::setprecision(17)<<"{\n  \"experiment\": \"GOLF-A-KFOLD-001\",\n"
               <<"  \"folds\": 5, \"grouping\": \"whole CSV file\",\n"
               <<"  \"wheelbase_nominal_m\": 2.63, \"minimum_speed_m_s\": 5, \"sample_period_s\": 0.02,\n"
               <<"  \"unique_files\": "<<unique.size()<<", \"exact_duplicate_files\": [";
            for(std::size_t i=0;i<duplicates.size();++i) {if(i) out<<", ";out<<'"'<<duplicates[i]<<'"';}
            out<<"],\n  \"results\": {\n";
            for(bool speed_model:{false,true}) {
                const char* name=speed_model?"model_a_speed":"model_a";
                if(speed_model) out<<",\n";
                out<<"    \""<<name<<"\": {\n      \"fold_results\": [\n";
                double total_sse=0; std::size_t total_n=0;
                for(std::size_t fold=0;fold<folds;++fold) {
                    const auto partition=dataset.fold(fold);
                    const auto& train=partition.training;
                    const auto& valid=partition.validation;
                    const auto& held_indices=partition.held_indices;
                    const auto& held=partition.held_names;
                    const auto f=fit(train,speed_model,2.63);
                    const auto v=evaluate(valid,f,speed_model);
                    total_sse+=v.sse;total_n+=v.n;
                    if(fold) out<<",\n";
                    out<<"        {\"fold\": "<<fold+1<<", \"test_files\": [";
                    for(std::size_t i=0;i<held.size();++i) {if(i) out<<", ";out<<'"'<<held[i]<<'"';}
                    out<<"], \"file_results\": [";
                    for(std::size_t i=0;i<held_indices.size();++i) {
                        if(i) out<<", ";
                        const auto one=evaluate({files[held_indices[i]]},f,speed_model);
                        out<<"{\"file\": \""<<held[i]<<"\", \"rmse_radps\": "<<rmse(one)
                           <<", \"samples\": "<<one.n<<"}";
                    }
                    out<<"], \"rmse_radps\": "<<rmse(v)<<", \"samples\": "<<v.n
                       <<", \"tau_s\": "<<f.tau<<", \"b_radps\": "<<f.bias;
                    if(speed_model) out<<", \"k_v_s2_per_m\": "<<f.kv;
                    else out<<", \"K_r_per_s\": "<<f.gain;
                    out<<"}";
                    std::cout<<name<<" fold "<<fold+1<<" RMSE="<<rmse(v)<<" n="<<v.n<<'\n';
                }
                const auto full=fit(files,speed_model,2.63);
                out<<"\n      ], \"pooled_cv_rmse_radps\": "<<std::sqrt(total_sse/total_n)
                   <<", \"cv_samples\": "<<total_n
                   <<", \"full_fit\": {\"tau_s\": "<<full.tau<<", \"b_radps\": "<<full.bias;
                if(speed_model) out<<", \"k_v_s2_per_m\": "<<full.kv;
                else out<<", \"K_r_per_s\": "<<full.gain;
                out<<"}}";
                std::cout<<name<<" pooled CV RMSE="<<std::sqrt(total_sse/total_n)<<'\n';
            }
            out<<"\n  }\n}\n";
            return 0;
        }
        if(argc>=2 && std::string(argv[1])=="--golf-single") {
            if(argc!=4) throw std::runtime_error("Usage: model_a_speed_cv --golf-single 2023-11_1.csv result.json");
            const auto path=std::filesystem::canonical(argv[2]).string();
            p4::io::IONIQ5CsvReader reader(p4::io::CsvChannelMap::StanfordGolf);
            p4::io::ModelACsvOptions options;
            options.minimum_speed_m_s=5.0;
            options.sample_period_s=0.005;
            const auto train=reader.readModelASegments(path,options,20.0,70.0);
            const auto valid=reader.readModelASegments(path,options,75.0,100.0);
            if(train.empty() || valid.empty()) throw std::runtime_error("No usable training or validation segments");
            const std::vector<File> training{train}, validation{valid};
            const auto a=fit(training,false,2.63), as=fit(training,true,2.63);
            const auto av=evaluate(validation,a,false), asv=evaluate(validation,as,true);
            double speed_min=INFINITY, speed_max=-INFINITY;
            double steer_min=INFINITY, steer_max=-INFINITY;
            for(const auto& segment:train) {
                for(double v:segment.speed_m_s) { speed_min=std::min(speed_min,v); speed_max=std::max(speed_max,v); }
                for(double d:segment.steering_rad) { steer_min=std::min(steer_min,d); steer_max=std::max(steer_max,d); }
            }
            std::ofstream out(argv[3]);
            if(!out) throw std::runtime_error("Cannot write JSON result");
            out<<std::setprecision(17)
               <<"{\n  \"experiment\": \"GOLF-A-001\",\n  \"dataset\": \"2023-11_1.csv\",\n"
               <<"  \"train_interval_s\": [20, 70],\n  \"validation_interval_s\": [75, 100],\n"
               <<"  \"minimum_speed_m_s\": 5,\n  \"sample_period_s\": 0.005,\n"
               <<"  \"training_speed_range_mps\": ["<<speed_min<<", "<<speed_max<<"],\n"
               <<"  \"training_road_wheel_angle_range_rad\": ["<<steer_min<<", "<<steer_max<<"],\n"
               <<"  \"wheelbase_nominal_m\": 2.63,\n"
               <<"  \"model_a\": {\"tau_s\": "<<a.tau<<", \"K_r_per_s\": "<<a.gain
               <<", \"b_radps\": "<<a.bias<<", \"train_rmse_radps\": "<<rmse(a)
               <<", \"validation_rmse_radps\": "<<rmse(av)<<", \"train_samples\": "<<a.n
               <<", \"validation_samples\": "<<av.n<<"},\n"
               <<"  \"model_a_speed\": {\"tau_s\": "<<as.tau<<", \"k_v_s2_per_m\": "<<as.kv
               <<", \"b_radps\": "<<as.bias<<", \"train_rmse_radps\": "<<rmse(as)
               <<", \"validation_rmse_radps\": "<<rmse(asv)<<", \"train_samples\": "<<as.n
               <<", \"validation_samples\": "<<asv.n<<"}\n}\n";
            std::cout<<"GOLF-A-001 train segments="<<train.size()<<" validation segments="<<valid.size()<<'\n'
                     <<"Model A: tau="<<a.tau<<" K="<<a.gain<<" b="<<a.bias
                     <<" train RMSE="<<rmse(a)<<" validation RMSE="<<rmse(av)<<'\n'
                     <<"Model A-Speed: tau="<<as.tau<<" kv="<<as.kv<<" b="<<as.bias
                     <<" train RMSE="<<rmse(as)<<" validation RMSE="<<rmse(asv)<<'\n';
            return 0;
        }
        if(argc>1 && (std::string(argv[1])=="--all-file-cv" || std::string(argv[1])=="--fit-all")) {
            const bool fit_all=std::string(argv[1])=="--fit-all";
            if(argc<5) throw std::runtime_error("Usage: model_a_speed_cv (--all-file-cv|--fit-all) file1.csv file2.csv ... (at least 3 files)");
            p4::io::IONIQ5CsvReader reader;
            std::vector<std::string> names,test_names;
            std::vector<File> files,tests;
            std::unordered_set<std::string> seen;
            bool testing=false;
            for(int i=2;i<argc;++i) {
                if(std::string(argv[i])=="--test") {
                    if(!fit_all) throw std::runtime_error("--test is supported only with --fit-all");
                    testing=true; continue;
                }
                const auto path=std::filesystem::canonical(argv[i]).string();
                if(!seen.insert(path).second) throw std::runtime_error("Duplicate or train/test overlap: "+path);
                auto segments=reader.readModelASegments(path);
                if(segments.empty()) throw std::runtime_error("No usable driving segments: "+path);
                (testing?test_names:names).push_back(path);
                (testing?tests:files).push_back(std::move(segments));
            }
            if(files.size()<3) throw std::runtime_error("Need at least three training files");
            std::cout<<std::fixed<<std::setprecision(6);
            if(fit_all) {
                std::cout<<"Full-file fit: "<<files.size()<<" files (training RMSE only)\n";
                for(bool speed_model:{false,true}) {
                    const auto f=fit(files,speed_model);
                    const char* name=speed_model?"A1-speed":"A0-constant";
                    std::cout<<std::setprecision(17)<<name<<" all-file tau="<<f.tau
                             <<" b="<<f.bias;
                    if(speed_model) std::cout<<" kv="<<f.kv<<" (L=3 m)";
                    else std::cout<<" K="<<f.gain;
                    std::cout<<" train RMSE="<<rmse(f)<<" n="<<f.n<<'\n';
                    for(std::size_t i=0;i<tests.size();++i) {
                        const auto score=evaluate({tests[i]},f,speed_model);
                        std::cout<<name<<" new test "<<std::filesystem::path(test_names[i]).filename().string()
                                 <<" RMSE="<<rmse(score)<<" n="<<score.n<<'\n';
                    }
                }
                return 0;
            }
            std::cout<<"File-group leave-one-file-out CV: "<<files.size()<<" folds\n";
            for(bool speed_model:{false,true}) {
                const char* name=speed_model?"A1-speed":"A0-constant";
                double cv_sse=0; std::size_t cv_n=0;
                for(std::size_t i=0;i<files.size();++i) {
                    const auto f=fit(excluding(files,i),speed_model);
                    const auto v=evaluate({files[i]},f,speed_model);
                    cv_sse+=v.sse; cv_n+=v.n;
                    std::cout<<name<<" fold "<<std::filesystem::path(names[i]).filename().string()
                             <<" RMSE="<<rmse(v)<<" n="<<v.n
                             <<" tau="<<f.tau<<" b="<<f.bias;
                    if(speed_model) std::cout<<" kv="<<f.kv;
                    else std::cout<<" K="<<f.gain;
                    std::cout<<'\n';
                }
                std::cout<<name<<" pooled file CV RMSE="<<std::sqrt(cv_sse/cv_n)
                         <<" n="<<cv_n<<'\n';
            }
            return 0;
        }
        std::vector<std::string> train_names,test_names;
        bool testing=false;
        for(int i=1;i<argc;++i) {
            if(std::string(argv[i])=="--test") {testing=true;continue;}
            (testing?test_names:train_names).push_back(std::filesystem::canonical(argv[i]).string());
        }
        if(train_names.size()<3||test_names.empty()) throw std::runtime_error("Usage: model_a_speed_cv train1.csv train2.csv train3.csv --test test.csv ...");
        for(const auto& a:train_names) for(const auto& b:test_names) if(a==b) throw std::runtime_error("Train/test overlap");
        p4::io::IONIQ5CsvReader reader;
        std::vector<File> train,test;
        for(const auto& p:train_names) train.push_back(reader.readModelASegments(p));
        for(const auto& p:test_names) test.push_back(reader.readModelASegments(p));
        std::cout<<std::fixed<<std::setprecision(6);
        for(bool speed_model:{false,true}) {
            const char* name=speed_model?"A1-speed":"A0-constant";
            double cv_sse=0; std::size_t cv_n=0;
            for(std::size_t i=0;i<train.size();++i) {
                auto f=fit(excluding(train,i),speed_model);
                auto v=evaluate({train[i]},f,speed_model);
                cv_sse+=v.sse; cv_n+=v.n;
                std::cout<<name<<" CV "<<std::filesystem::path(train_names[i]).filename().string()<<" RMSE="<<rmse(v)<<" n="<<v.n<<'\n';
            }
            std::cout<<name<<" pooled CV RMSE="<<std::sqrt(cv_sse/cv_n)<<'\n';
            auto full=fit(train,speed_model);
            std::cout<<std::setprecision(17)<<name<<" full-train tau="<<full.tau<<" b="<<full.bias;
            if(speed_model) std::cout<<" kv="<<full.kv<<" (L=3 m)";
            else std::cout<<" K="<<full.gain;
            std::cout<<" train RMSE="<<rmse(full)<<'\n'<<std::setprecision(6);
            double test_sse=0; std::size_t test_n=0;
            for(std::size_t i=0;i<test.size();++i) {
                auto v=evaluate({test[i]},full,speed_model);
                test_sse+=v.sse; test_n+=v.n;
                std::cout<<name<<" test "<<std::filesystem::path(test_names[i]).filename().string()<<" RMSE="<<rmse(v)<<" n="<<v.n<<'\n';
                for(std::size_t j=0;j<test[i].size();++j) {
                    auto segment=evaluate({File{test[i][j]}},full,speed_model);
                    std::cout<<name<<" test segment "<<j+1<<" RMSE="<<rmse(segment)<<" n="<<segment.n<<'\n';
                    auto after_start=evaluate({File{test[i][j]}},full,speed_model,.2);
                    std::cout<<name<<" test segment "<<j+1<<" after_0.2s_RMSE="<<rmse(after_start)<<" n="<<after_start.n<<'\n';
                }
            }
            std::cout<<name<<" pooled test RMSE="<<std::sqrt(test_sse/test_n)<<'\n';
        }
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
