
#include "p4/io/VBOReader.hpp"
#include "p4/maneuvers/ManeuverDetector.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    try {
        if (argc != 3) {
            std::cerr
                << "Usage: maneuver_export "
                << "<input.vbo> <output_prefix>\n";
            return 1;
        }

        const std::string input = argv[1];
        const std::string prefix = argv[2];

        p4::io::VBOReader reader;
        const auto data = reader.read(input);

        p4::maneuvers::ManeuverDetector detector;
        const auto maneuvers =
            detector.detectStepSteer(
                data, "steer_ang2"
            );

        const std::vector<std::string> channels = {
            "steer_ang2",
            "YawRate",
            "Yaw_Rate",
            "velocity"
        };

        if (!data.hasColumn("steer_ang2")) {
            throw std::runtime_error(
                "Missing steering channel"
            );
        }

        const fs::path output_path(prefix);
        if (output_path.has_parent_path()) {
            fs::create_directories(
                output_path.parent_path()
            );
        }

        const double t0 = data.timeAt(0);

        // ----------------------------------------
        // Signal CSV
        // ----------------------------------------
        std::ofstream signal(prefix + "_signals.csv");
        if (!signal) {
            throw std::runtime_error(
                "Cannot create signal CSV"
            );
        }

        signal << std::setprecision(12);
        signal << "time,steer_ang2,YawRate,"
               << "Yaw_Rate,velocity\n";

        for (std::size_t i = 0;
             i < data.sampleCount(); ++i) {

            signal << data.timeAt(i) - t0;

            for (const auto& channel : channels) {
                signal << ",";

                if (data.hasColumn(channel)) {
                    signal << data.value(i, channel);
                }
            }

            signal << '\n';
        }

        // ----------------------------------------
        // Maneuver CSV
        // ----------------------------------------
        std::ofstream events(prefix + "_maneuvers.csv");
        if (!events) {
            throw std::runtime_error(
                "Cannot create maneuver CSV"
            );
        }

        events << std::setprecision(12);
        events
            << "id,start,end,start_index,end_index,"
            << "delta_deg,rise_s,hold_s\n";

        for (std::size_t i = 0;
             i < maneuvers.size(); ++i) {

            const auto& m = maneuvers[i];

            events
                << i + 1 << ","
                << m.start_time - t0 << ","
                << m.end_time - t0 << ","
                << m.start_index << ","
                << m.end_index << ","
                << m.steering_delta_deg << ","
                << m.rise_time << ","
                << m.hold_time << '\n';
        }

        if (!signal || !events) {
            throw std::runtime_error(
                "CSV write failure"
            );
        }

        std::cout
            << "Input      : " << input << '\n'
            << "Samples    : " << data.sampleCount() << '\n'
            << "Candidates : " << maneuvers.size() << '\n'
            << "Signal CSV : " << prefix << "_signals.csv\n"
            << "Event CSV  : " << prefix << "_maneuvers.csv\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << '\n';
        return 1;
    }
}
