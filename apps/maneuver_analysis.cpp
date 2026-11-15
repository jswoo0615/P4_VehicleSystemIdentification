
#include "p4/io/VBOReader.hpp"
#include "p4/maneuvers/ManeuverSegmenter.hpp"
#include "p4/maneuvers/ManeuverGrouper.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr
            << "Usage: maneuver_segment_analysis "
            << "<file1.vbo> [file2.vbo ...]\n";
        return 1;
    }

    try {
        p4::io::VBOReader reader;
        p4::maneuvers::ManeuverSegmenter segmenter;
        p4::maneuvers::ManeuverGrouper grouper;

        std::cout << std::fixed
                  << std::setprecision(3);

        for (int f = 1; f < argc; ++f) {
            const std::string filename = argv[f];

            const auto data = reader.read(filename);
            const auto events = segmenter.analyze(data);
            const auto groups = grouper.group(events);

            std::cout
                << "\n=== " << filename << " ===\n"
                << "Samples: " << data.sampleCount() << '\n'
                << "Events : " << events.size() << '\n'
                << "Groups : " << groups.size() << '\n';

            // ------------------------------------
            // Event details
            // ------------------------------------
            for (const auto& e : events) {
                std::cout
                    << "  Event #" << e.event_id
                    << " [" << e.start_s
                    << ", " << e.end_s << "] s"
                    << " | " << p4::maneuvers::toString(
                        e.candidate
                    )
                    << " | Steer=" << e.max_abs_steer_deg
                    << " deg"
                    << " | Yaw=" << e.max_abs_yaw_deg_s
                    << " deg/s"
                    << " | Speed=" << e.mean_speed_kmh
                    << " km/h\n";
            }

            // ------------------------------------
            // Group details
            // ------------------------------------
            for (const auto& g : groups) {
                std::cout
                    << "  Group #" << g.group_id
                    << " [" << g.start_s
                    << ", " << g.end_s << "] s"
                    << " | Duration=" << g.duration_s
                    << " s"
                    << " | Events=" << g.events.size()
                    << " | IDs=";

                for (std::size_t i = 0;
                     i < g.events.size(); ++i) {
                    if (i != 0) {
                        std::cout << ",";
                    }

                    std::cout << g.events[i].event_id;
                }

                std::cout << '\n';
            }
        }

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] "
                  << e.what() << '\n';
        return 1;
    }
}
