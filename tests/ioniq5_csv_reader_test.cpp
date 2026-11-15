#include "p4/io/IONIQ5CsvReader.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
    const auto file=std::filesystem::temp_directory_path()/"p4_ioniq5_reader_test.csv";
    {
        std::ofstream out(file);
        out<<"time_s,delta_stm_rad,w_z_cor_radps,v_x_cor_mps\n";
        for(int i=0;i<8000;++i) {
            const double t=i*0.001;
            const double yaw=(i>=3000&&i<3020)?0.5:0.02;
            out<<t<<','<<0.1<<','<<yaw<<','<<3.0<<'\n';
        }
    }
    try {
        p4::io::IONIQ5CsvReader reader;
        const auto segments=reader.readModelASegments(file.string());
        std::filesystem::remove(file);
        if(segments.size()!=2) return 1;
        for(const auto& s:segments) {
            if(s.time_s.size()<30) return 2;
            for(std::size_t i=0;i<s.time_s.size();++i) {
                if(std::abs(s.steering_rad[i]-0.1)>1e-12 ||
                   (s.time_s[i]>2.899 && s.time_s[i]<3.12)) return 3;
                if(i && (s.time_s[i]-s.time_s[i-1]<0.019 ||
                         s.time_s[i]-s.time_s[i-1]>0.021)) return 4;
            }
        }
        return 0;
    } catch(const std::exception& e) {
        std::filesystem::remove(file);
        std::cerr<<e.what()<<'\n'; return 5;
    }
}
