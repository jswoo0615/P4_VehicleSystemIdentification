#include "p4/io/IONIQ5LoadSamples.hpp"

#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Point=p4::io::LoadPoint;
using File=std::vector<Point>;
struct Fit {std::array<double,4> c{}; double vs=0;};
double predict(const Point& p,const Fit& f,bool nonlinear) {
    return f.c[0]+f.c[1]*p.delta+f.c[2]*p.rate+
           (nonlinear?f.c[3]*std::tanh(p.rate/f.vs):0);
}
Fit solve(const std::vector<File>& files,bool left,bool nonlinear,double vs) {
    const int n=nonlinear?4:3; double a[4][5]={};
    for(const auto& file:files) for(const auto& p:file) {
        const double x[4]={1,p.delta,p.rate,nonlinear?std::tanh(p.rate/vs):0};
        const double y=left?p.left:p.right;
        for(int i=0;i<n;++i) {for(int j=0;j<n;++j) a[i][j]+=x[i]*x[j]; a[i][n]+=x[i]*y;}
    }
    for(int i=0;i<n;++i) {
        int pivot=i; for(int j=i+1;j<n;++j) if(std::abs(a[j][i])>std::abs(a[pivot][i])) pivot=j;
        if(std::abs(a[pivot][i])<1e-10) throw std::runtime_error("Rank deficient S-Load design");
        for(int j=0;j<=n;++j) std::swap(a[i][j],a[pivot][j]);
        const double scale=a[i][i]; for(int j=i;j<=n;++j) a[i][j]/=scale;
        for(int k=0;k<n;++k) if(k!=i) {
            const double m=a[k][i]; for(int j=i;j<=n;++j) a[k][j]-=m*a[i][j];
        }
    }
    Fit f; f.vs=vs; for(int i=0;i<n;++i) f.c[i]=a[i][n]; return f;
}
struct Score {double sse=0; std::size_t n=0; double reverse_sse=0; std::size_t reverse_n=0;};
Score score(const File& file,const Fit& f,bool left,bool nonlinear) {
    Score out; double last_sign=0,reverse_until=-1;
    for(const auto& p:file) {
        const double e=(left?p.left:p.right)-predict(p,f,nonlinear);
        out.sse+=e*e; ++out.n;
        // A reversal is a change of rate sign after a meaningful steering
        // update. Report the following 200 ms as a separate diagnostic.
        const double sign=p.rate>.02?1:(p.rate<-.02?-1:0);
        if(sign!=0) {
            if(last_sign!=0&&sign!=last_sign) reverse_until=p.t+.2;
            last_sign=sign;
        }
        if(p.t<=reverse_until) {out.reverse_sse+=e*e; ++out.reverse_n;}
    }
    return out;
}
double rmse(double sse,std::size_t n) {return n?std::sqrt(sse/n):NAN;}
std::vector<File> excluding(const std::vector<File>& files,std::size_t held) {
    std::vector<File> out; for(std::size_t i=0;i<files.size();++i) if(i!=held) out.push_back(files[i]); return out;
}
}
int main(int argc,char** argv) {
    try {
        std::vector<std::string> train_names,test_names; bool testing=false,causal_rate=false;
        for(int i=1;i<argc;++i) {
            if(std::string(argv[i])=="--test") {testing=true;continue;}
            if(std::string(argv[i])=="--causal-rate") {causal_rate=true;continue;}
            (testing?test_names:train_names).push_back(std::filesystem::canonical(argv[i]).string());
        }
        if(train_names.size()<3||test_names.empty()) throw std::runtime_error("Usage: model_s_load_friction_cv train1.csv train2.csv train3.csv --test test.csv ...");
        for(const auto& a:train_names) for(const auto& b:test_names) if(a==b) throw std::runtime_error("Train/test overlap");
        std::vector<File> train,test;
        const auto mode=causal_rate ? p4::io::SteeringRateMode::PreviousUpdate
                                    : p4::io::SteeringRateMode::Centered;
        for(const auto& p:train_names) train.push_back(p4::io::readGuardedLoadSamples(p,mode));
        for(const auto& p:test_names) test.push_back(p4::io::readGuardedLoadSamples(p,mode));
        const std::vector<double> candidates={.01,.02,.04,.08,.16,.32,.64,1.28};
        std::cout<<std::fixed<<std::setprecision(4);
        std::cout<<"rate_mode="<<(causal_rate?"causal":"centered")<<'\n';
        for(bool left:{true,false}) {
            const char* side=left?"left":"right";
            for(bool nonlinear:{false,true}) {
                const char* name=nonlinear?"S1-friction":"S0-linear";
                double best_cv=INFINITY,best_vs=0;
                const auto& grid=nonlinear?candidates:std::vector<double>{1};
                for(const double vs:grid) {
                    double total=0;
                    for(std::size_t i=0;i<train.size();++i) {
                        const auto f=solve(excluding(train,i),left,nonlinear,vs);
                        total+=score(train[i],f,left,nonlinear).sse;
                    }
                    if(total<best_cv) {best_cv=total;best_vs=vs;}
                }
                auto f=solve(train,left,nonlinear,best_vs);
                std::cout<<side<<' '<<name<<" selected_vs="<<best_vs
                         <<" pooled_CV_RMSE="<<rmse(best_cv,train[0].size()+train[1].size()+train[2].size())
                         <<" coefficients="<<f.c[0]<<','<<f.c[1]<<','<<f.c[2]<<','<<f.c[3]<<'\n';
                for(std::size_t i=0;i<train.size();++i) {
                    const auto fold=solve(excluding(train,i),left,nonlinear,best_vs);
                    const auto v=score(train[i],fold,left,nonlinear);
                    std::cout<<side<<' '<<name<<" CV "<<std::filesystem::path(train_names[i]).filename().string()
                             <<" RMSE="<<rmse(v.sse,v.n)<<" reversal_RMSE="<<rmse(v.reverse_sse,v.reverse_n)<<" reversal_n="<<v.reverse_n<<'\n';
                }
                double sum=0; std::size_t count=0;
                for(std::size_t i=0;i<test.size();++i) {
                    const auto v=score(test[i],f,left,nonlinear); sum+=v.sse;count+=v.n;
                    std::cout<<side<<' '<<name<<" test "<<std::filesystem::path(test_names[i]).filename().string()
                             <<" RMSE="<<rmse(v.sse,v.n)<<" reversal_RMSE="<<rmse(v.reverse_sse,v.reverse_n)<<" reversal_n="<<v.reverse_n<<'\n';
                }
                std::cout<<side<<' '<<name<<" pooled_test_RMSE="<<rmse(sum,count)<<'\n';
            }
        }
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
