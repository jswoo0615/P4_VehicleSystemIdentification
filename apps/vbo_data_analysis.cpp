
#include "p4/io/VBOReader.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

struct Statistics {
    double min = std::numeric_limits<double>::infinity();
    double max = -std::numeric_limits<double>::infinity();
    double sum = 0.0;
    double sum_squared = 0.0;
    std::size_t count = 0;

    void add(double value) {
        min = std::min(min, value);
        max = std::max(max, value);
        sum += value;
        sum_squared += value * value;
        ++count;
    }

    double mean() const {
        return count ? sum / static_cast<double>(count) : 0.0;
    }

    double rms() const {
        return count
            ? std::sqrt(sum_squared / static_cast<double>(count))
            : 0.0;
    }
};

void printStats(
    const std::string& name,
    const Statistics& stats
) {
    std::cout
        << std::setw(16) << std::left << name
        << std::setw(14) << stats.min
        << std::setw(14) << stats.max
        << std::setw(14) << stats.mean()
        << stats.rms() << '\n';
}

} // anonymous namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            std::cerr
                << "Usage: vbo_data_analysis <file.vbo>\n";
            return 1;
        }

        p4::io::VBOReader reader;
        const auto data = reader.read(argv[1]);

        if (data.sampleCount() < 2) {
            throw std::runtime_error("Insufficient samples");
        }

        // 한 번만 채널 인덱스를 검색
        const auto idxSteer = data.columnIndex("steer_ang2");
        const auto idxYaw = data.columnIndex("YawRate");
        const auto idxYaw2 = data.columnIndex("Yaw_Rate");
        const auto idxSpeed = data.columnIndex("velocity");
        const auto idxLatAcc = data.columnIndex("Latacc");

        Statistics steerStats;
        Statistics yawStats;
        Statistics yaw2Stats;
        Statistics speedStats;
        Statistics latAccStats;
        Statistics yawDifferenceStats;
        Statistics dtStats;

        std::size_t largeGaps = 0;
        const double gapThreshold = 0.02; // 20 ms

        std::filesystem::create_directories("output");
        std::ofstream csv("output/vbo_analysis.csv");

        if (!csv) {
            throw std::runtime_error("Cannot create CSV");
        }

        csv << std::setprecision(12);
        csv << "time,dt,steering,yawrate,yaw_rate,"
               "speed_kmh,latacc_g,yaw_difference\n";

        double firstTime = data.samples.front().time;

        for (std::size_t k = 0; k < data.sampleCount(); ++k) {
            const auto& sample = data.samples[k];

            const double steering = sample.values[idxSteer];
            const double yaw = sample.values[idxYaw];
            const double yaw2 = sample.values[idxYaw2];
            const double speed = sample.values[idxSpeed];
            const double latAcc = sample.values[idxLatAcc];

            const double yawDifference = yaw - yaw2;

            steerStats.add(steering);
            yawStats.add(yaw);
            yaw2Stats.add(yaw2);
            speedStats.add(speed);
            latAccStats.add(latAcc);
            yawDifferenceStats.add(yawDifference);

            double dt = 0.0;

            if (k > 0) {
                dt = sample.time - data.samples[k - 1].time;

                if (dt <= 0.0) {
                    throw std::runtime_error(
                        "Non-increasing timestamp"
                    );
                }

                dtStats.add(dt);

                if (dt > gapThreshold) {
                    ++largeGaps;
                }
            }

            csv << sample.time - firstTime << ','
                << dt << ','
                << steering << ','
                << yaw << ','
                << yaw2 << ','
                << speed << ','
                << latAcc << ','
                << yawDifference << '\n';
        }

        std::cout << std::fixed
                  << std::setprecision(6);

        std::cout << "\n=== VBOX DATA QUALITY ===\n";
        std::cout << "File        : " << argv[1] << '\n';
        std::cout << "Samples     : " << data.sampleCount() << '\n';
        std::cout << "Duration    : "
                  << data.samples.back().time - firstTime
                  << " s\n";

        std::cout << "\n=== SAMPLING ===\n";
        std::cout << "Min dt      : " << dtStats.min << " s\n";
        std::cout << "Max dt      : " << dtStats.max << " s\n";
        std::cout << "Mean dt     : " << dtStats.mean() << " s\n";
        std::cout << "Gaps > 20ms : " << largeGaps << '\n';

        std::cout << "\n=== SIGNAL STATISTICS ===\n";
        std::cout
            << std::setw(16) << std::left << "Channel"
            << std::setw(14) << "Min"
            << std::setw(14) << "Max"
            << std::setw(14) << "Mean"
            << "RMS\n";

        printStats("Steering", steerStats);
        printStats("YawRate", yawStats);
        printStats("Yaw_Rate", yaw2Stats);
        printStats("Speed km/h", speedStats);
        printStats("Latacc g", latAccStats);

        std::cout << "\n=== YAW CHANNEL COMPARISON ===\n";
        std::cout << "Mean difference : "
                  << yawDifferenceStats.mean() << '\n';
        std::cout << "RMS difference  : "
                  << yawDifferenceStats.rms() << '\n';

        std::cout << "\nSaved: output/vbo_analysis.csv\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << '\n';
        return 1;
    }
}
