#include "p4/io/IONIQ5CsvReader.hpp"
#include "p4/identification/GolfDataset.hpp"
#include "p4/models/LinearBicycleModel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Segment=p4::io::ModelACsvSamples;
using File=std::vector<Segment>;
using Params=std::array<double,4>; // Cf/m, Cr/m, Iz/m, lf; lr=2.63-lf
struct Score {double yaw2=0,vy2=0;std::size_t n=0;};
struct Scale {double yaw=0,vy=0;};
struct Fit {Params p{};double loss=INFINITY;};
struct ASpeedFit {double tau=0,kv=0,bias=0,sse=INFINITY;};
constexpr double L=2.63, G=9.80665;

bool valid(const Params& p) {
    return p[0]>=10&&p[0]<=250&&p[1]>=10&&p[1]<=250&&
           p[2]>=.7&&p[2]<=3.2&&p[3]>=.7&&p[3]<=1.6;
}
Score evaluate(const std::vector<File>& files,const Params& p) {
    const p4::models::BicycleParameters physical{1.0,p[2],p[3],L-p[3],p[0],p[1]};
    Score score;
    for(const auto& file:files) for(const auto& s:file) {
        p4::models::LinearBicycleModel model(physical);
        model.reset({s.lateral_velocity_m_s.front(),s.yaw_rad_s.front()});
        for(std::size_t i=1;i<s.time_s.size();++i) {
            const auto x=model.stepImplicitMidpoint(s.steering_rad[i-1],s.speed_m_s[i-1],
                s.time_s[i]-s.time_s[i-1],.02,G*std::sin(s.bank_rad[i-1]));
            const double ey=s.yaw_rad_s[i]-x.yaw_rate_rad_s;
            const double ev=s.lateral_velocity_m_s[i]-x.lateral_velocity_m_s;
            score.yaw2+=ey*ey;score.vy2+=ev*ev;++score.n;
        }
    }
    return score;
}
Scale scaleOf(const std::vector<File>& files) {
    double sy=0,sv=0,yy=0,vv=0;std::size_t n=0;
    for(const auto& file:files) for(const auto& s:file) for(std::size_t i=1;i<s.time_s.size();++i) {
        const double y=s.yaw_rad_s[i],v=s.lateral_velocity_m_s[i];
        sy+=y;sv+=v;yy+=y*y;vv+=v*v;++n;
    }
    if(n==0) throw std::runtime_error("No training samples");
    return {std::sqrt(yy/n-(sy/n)*(sy/n)),std::sqrt(vv/n-(sv/n)*(sv/n))};
}
double impliedKv(const Params& p) {
    return ((L-p[3])/p[0]-p[3]/p[1])/L;
}
// One grid search on this fold's training files only, matching Model A-Speed.
ASpeedFit fitASpeed(const std::vector<File>& files) {
    ASpeedFit best;
    auto consider=[&](double tau,double kv) {
            double hh=0,hy=0,yy=0;
            for(const auto& file:files) for(const auto& s:file) {
                double p=s.yaw_rad_s.front(),q=0,h=0;
                for(std::size_t i=1;i<s.time_s.size();++i) {
                    const double a=std::exp(-(s.time_s[i]-s.time_s[i-1])/tau);
                    p*=a;
                    const double u=s.steering_rad[i-1]*s.speed_m_s[i-1]/
                        (L+kv*s.speed_m_s[i-1]*s.speed_m_s[i-1]);
                    q=a*q+(1-a)*u;h=a*h+(1-a);
                    const double y=s.yaw_rad_s[i]-p-q;
                    hh+=h*h;hy+=h*y;yy+=y*y;
                }
            }
            const double sse=yy-hy*hy/hh;
            if(sse<best.sse) best={tau,kv,hy/hh,sse};
    };
    for(int ti=0;ti<=80;++ti) {
        const double tau=.03*std::pow(100.0,ti/80.0);
        for(int ki=0;ki<=60;++ki) consider(tau,ki*.0025);
    }
    const double center=best.kv;
    for(int ti=0;ti<=80;++ti) {
        const double tau=.03*std::pow(100.0,ti/80.0);
        for(int ki=-10;ki<=10;++ki) {
            const double kv=center+ki*.00025;
            if(kv>=0) consider(tau,kv);
        }
    }
    return best;
}
Score evaluateASpeed(const std::vector<File>& files,const ASpeedFit& f) {
    Score score;
    for(const auto& file:files) for(const auto& s:file) {
        double r=s.yaw_rad_s.front();
        for(std::size_t i=1;i<s.time_s.size();++i) {
            const double a=std::exp(-(s.time_s[i]-s.time_s[i-1])/f.tau);
            const double v=s.speed_m_s[i-1];
            r=a*r+(1-a)*(v*s.steering_rad[i-1]/(L+f.kv*v*v)+f.bias);
            const double error=s.yaw_rad_s[i]-r;
            score.yaw2+=error*error;++score.n;
        }
    }
    return score;
}
double loss(const std::vector<File>& files,const Params& p,const Scale& scale,
            double target_kv=std::numeric_limits<double>::quiet_NaN()) {
    if(!valid(p)) return INFINITY;
    const auto s=evaluate(files,p);
    const double data=(s.yaw2/(scale.yaw*scale.yaw)+s.vy2/(scale.vy*scale.vy))/s.n;
    if(!std::isfinite(target_kv)) return data;
    const double residual=(impliedKv(p)-target_kv)/.0025;
    return data+.05*residual*residual;
}
Fit fit(const std::vector<File>& files,const Scale& scale,double fixed_lf=0,
        double target_kv=std::numeric_limits<double>::quiet_NaN()) {
    Fit best;
    for(const Params& seed:std::array<Params,11>{{
        {40,60,1.4,.9},{60,60,1.6,1.04},{90,90,1.8,1.2},{35,100,2.2,1.4},
        {100,40,1.0,1.0},{50,120,2.6,1.3},{120,120,1.2,.8},{25,25,3.0,1.5},
        {50,50,2.5,1.0},{50,45,2.35,.95},{50,55,2.65,1.05}}}) {
        Params p=seed;if(fixed_lf>0) p[3]=fixed_lf;
        const double value=loss(files,p,scale,target_kv);
        if(value<best.loss) best={p,value};
    }
    auto refine=[&]() { for(double step:{.35,.18,.09,.045,.02,.008}) {
        for(int iteration=0;iteration<20;++iteration) {
            Fit candidate=best;
            for(std::size_t j=0;j<(fixed_lf>0?3u:4u);++j) for(int direction:{-1,1}) {
                Params p=best.p;p[j]*=std::exp(direction*step);
                const double value=loss(files,p,scale,target_kv);
                if(value<candidate.loss) candidate={p,value};
            }
            if(fixed_lf==0) for(int d1:{-1,1}) for(int d2:{-1,1}) {
                Params p=best.p;p[2]*=std::exp(d1*step);p[3]*=std::exp(d2*step);
                const double value=loss(files,p,scale,target_kv);
                if(value<candidate.loss) candidate={p,value};
            }
            if(candidate.loss>=best.loss-1e-9) break;
            best=candidate;
        }
    }};
    refine();
    if(fixed_lf==0) {
        for(double lf:{0.8,1.0,1.2,1.4}) {
            const auto candidate=fit(files,scale,lf,target_kv);
            if(candidate.loss<best.loss) best=candidate;
        }
        refine();
    }
    if(!std::isfinite(best.loss)) throw std::runtime_error("Model B fit failed");
    return best;
}
double rmse(double sse,std::size_t n) {return std::sqrt(sse/n);}
void writeParams(std::ostream& out,const Params& p) {
    out<<"\"cf_over_m\": "<<p[0]<<", \"cr_over_m\": "<<p[1]
       <<", \"iz_over_m_m2\": "<<p[2]<<", \"lf_m\": "<<p[3]
       <<", \"lr_m\": "<<L-p[3];
}
}

int main(int argc,char** argv) {
    try {
        const bool compare=argc==4&&std::string(argv[1])=="--compare-a-speed";
        if(!compare&&argc!=3) throw std::runtime_error("Usage: model_b_golf_cv [--compare-a-speed] dataset_dir result.json");
        const char* dataset_dir=argv[compare?2:1];
        const char* result_path=argv[compare?3:2];
        const auto dataset=p4::identification::GolfDataset::load(dataset_dir);
        const auto& unique=dataset.paths();
        const auto& files=dataset.files();
        constexpr auto folds=p4::identification::GolfDataset::fold_count;
        std::ofstream out(result_path);if(!out) throw std::runtime_error("Cannot write result JSON");
        if(compare) {
            out<<std::setprecision(17)<<"{\n  \"experiment\": \"GOLF-B0-B1-KFOLD-001\",\n"
               <<"  \"folds\": "<<folds<<", \"unique_files\": "<<unique.size()
               <<", \"wheelbase_m\": 2.63, \"sample_period_s\": 0.02,\n"
               <<"  \"b1_penalty\": {\"weight\": 0.05, \"kv_scale_s2_per_m\": 0.0025},\n"
               <<"  \"fold_results\": [\n";
            Score pooledA,pooled0,pooled1;
            for(std::size_t fold=0;fold<folds;++fold) {
                const auto partition=dataset.fold(fold);
                const auto& train=partition.training;
                const auto& test=partition.validation;
                const auto& held=partition.held_indices;
                const auto a=fitASpeed(train);
                const auto scale=scaleOf(train);
                const auto b0=fit(train,scale);
                const auto b1=fit(train,scale,0,a.kv);
                const auto sa=evaluateASpeed(test,a);
                const auto s0=evaluate(test,b0.p),s1=evaluate(test,b1.p);
                pooledA.yaw2+=sa.yaw2;pooledA.n+=sa.n;
                pooled0.yaw2+=s0.yaw2;pooled0.vy2+=s0.vy2;pooled0.n+=s0.n;
                pooled1.yaw2+=s1.yaw2;pooled1.vy2+=s1.vy2;pooled1.n+=s1.n;
                if(fold) out<<",\n";
                out<<"    {\"fold\": "<<fold+1<<", \"test_files\": [";
                for(std::size_t j=0;j<held.size();++j) {
                    if(j) out<<", ";out<<'"'<<unique[held[j]].filename().string()<<'"';
                }
                out<<"], \"training_a_speed\": {\"tau_s\": "<<a.tau
                   <<", \"kv_s2_per_m\": "<<a.kv<<", \"b_radps\": "<<a.bias
                   <<", \"test_yaw_rmse_radps\": "<<rmse(sa.yaw2,sa.n)<<"},\n"
                   <<"      \"b0\": {\"parameters\": {";writeParams(out,b0.p);
                out<<"}, \"implied_kv_s2_per_m\": "<<impliedKv(b0.p)
                   <<", \"yaw_rmse_radps\": "<<rmse(s0.yaw2,s0.n)
                   <<", \"vy_rmse_mps\": "<<rmse(s0.vy2,s0.n)<<"},\n"
                   <<"      \"b1\": {\"parameters\": {";writeParams(out,b1.p);
                out<<"}, \"implied_kv_s2_per_m\": "<<impliedKv(b1.p)
                   <<", \"training_data_objective\": "<<loss(train,b1.p,scale)
                   <<", \"training_penalized_objective\": "<<b1.loss
                   <<", \"yaw_rmse_radps\": "<<rmse(s1.yaw2,s1.n)
                   <<", \"vy_rmse_mps\": "<<rmse(s1.vy2,s1.n)<<"}, \"file_results\": [";
                for(std::size_t j=0;j<held.size();++j) {
                    if(j) out<<", ";
                    const auto fa=evaluateASpeed({files[held[j]]},a);
                    const auto f0=evaluate({files[held[j]]},b0.p);
                    const auto f1=evaluate({files[held[j]]},b1.p);
                    out<<"{\"file\": \""<<unique[held[j]].filename().string()
                       <<"\", \"a_speed_yaw_rmse_radps\": "<<rmse(fa.yaw2,fa.n)
                       <<", \"b0_yaw_rmse_radps\": "<<rmse(f0.yaw2,f0.n)
                       <<", \"b1_yaw_rmse_radps\": "<<rmse(f1.yaw2,f1.n)
                       <<", \"b0_vy_rmse_mps\": "<<rmse(f0.vy2,f0.n)
                       <<", \"b1_vy_rmse_mps\": "<<rmse(f1.vy2,f1.n)<<"}";
                }
                out<<"]}";
                std::cout<<"fold "<<fold+1<<" A-Speed kv="<<a.kv
                         <<" B0 yaw="<<rmse(s0.yaw2,s0.n)<<" B1 yaw="<<rmse(s1.yaw2,s1.n)
                         <<" B0 vy="<<rmse(s0.vy2,s0.n)<<" B1 vy="<<rmse(s1.vy2,s1.n)<<'\n';
            }
            const auto all_a=fitASpeed(files);
            const auto full_scale=scaleOf(files);
            const auto full0=fit(files,full_scale),full1=fit(files,full_scale,0,all_a.kv);
            out<<"\n  ], \"pooled\": {\"a_speed_refined_yaw_rmse_radps\": "<<rmse(pooledA.yaw2,pooledA.n)
               <<", \"b0\": {\"yaw_rmse_radps\": "<<rmse(pooled0.yaw2,pooled0.n)
               <<", \"vy_rmse_mps\": "<<rmse(pooled0.vy2,pooled0.n)<<"}, \"b1\": {\"yaw_rmse_radps\": "
               <<rmse(pooled1.yaw2,pooled1.n)<<", \"vy_rmse_mps\": "<<rmse(pooled1.vy2,pooled1.n)
               <<"}}, \"cv_samples\": "<<pooled0.n<<", \"full_fit_reference\": {\"training_a_speed_kv\": "
               <<all_a.kv<<", \"b0\": {";writeParams(out,full0.p);
            out<<"}, \"b1\": {";writeParams(out,full1.p);out<<"}}\n}\n";
            return 0;
        }
        out<<std::setprecision(17)<<"{\n  \"experiment\": \"GOLF-B-KFOLD-001\",\n"
           <<"  \"model\": \"normalized linear 2DOF bicycle with road bank\",\n"
           <<"  \"folds\": "<<folds<<", \"unique_files\": "<<unique.size()
           <<", \"wheelbase_m\": 2.63, \"minimum_speed_m_s\": 5, \"sample_period_s\": 0.02,\n"
           <<"  \"parameter_bounds\": {\"cf_over_m\": [10,250], \"cr_over_m\": [10,250], "
           <<"\"iz_over_m_m2\": [0.7,3.2], \"lf_m\": [0.7,1.6]},\n"
           <<"  \"fold_results\": [\n";
        Score pooled;
        for(std::size_t fold=0;fold<folds;++fold) {
            const auto partition=dataset.fold(fold);
            const auto& train=partition.training;
            const auto& test=partition.validation;
            const auto& held=partition.held_indices;
            const auto fitted=fit(train,scaleOf(train));
            const auto score=evaluate(test,fitted.p);
            pooled.yaw2+=score.yaw2;pooled.vy2+=score.vy2;pooled.n+=score.n;
            if(fold) out<<",\n";
            out<<"    {\"fold\": "<<fold+1<<", \"parameters\": {";writeParams(out,fitted.p);
            out<<"}, \"training_objective\": "<<fitted.loss
               <<", \"yaw_rmse_radps\": "<<rmse(score.yaw2,score.n)
               <<", \"vy_rmse_mps\": "<<rmse(score.vy2,score.n)
               <<", \"samples\": "<<score.n<<", \"file_results\": [";
            for(std::size_t j=0;j<held.size();++j) {
                if(j) out<<", ";const auto one=evaluate({files[held[j]]},fitted.p);
                out<<"{\"file\": \""<<unique[held[j]].filename().string()
                   <<"\", \"yaw_rmse_radps\": "<<rmse(one.yaw2,one.n)
                   <<", \"vy_rmse_mps\": "<<rmse(one.vy2,one.n)<<"}";
            }
            out<<"]}";
            std::cout<<"fold "<<fold+1<<" yaw RMSE="<<rmse(score.yaw2,score.n)
                     <<" vy RMSE="<<rmse(score.vy2,score.n)<<" n="<<score.n<<'\n';
        }
        const auto full_scale=scaleOf(files);
        const auto full=fit(files,full_scale);
        out<<"\n  ], \"pooled_yaw_rmse_radps\": "<<rmse(pooled.yaw2,pooled.n)
           <<", \"pooled_vy_rmse_mps\": "<<rmse(pooled.vy2,pooled.n)
           <<", \"cv_samples\": "<<pooled.n<<", \"full_fit\": {";
        writeParams(out,full.p);
        out<<", \"training_objective\": "<<full.loss<<"}, \"lf_profile\": [";
        for(double lf:{0.8,1.0,1.04,1.2,1.4,1.55}) {
            const auto profile=fit(files,full_scale,lf);
            if(lf!=0.8) out<<", ";
            out<<"{\"lf_m\": "<<lf<<", \"best_loss\": "<<profile.loss
               <<", \"parameters\": {";writeParams(out,profile.p);out<<"}}";
        }
        out<<"]\n}\n";
        std::cout<<"pooled yaw RMSE="<<rmse(pooled.yaw2,pooled.n)
                 <<" vy RMSE="<<rmse(pooled.vy2,pooled.n)<<'\n';
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
