
#include "p4/io/VBOReader.hpp"
#include "p4/maneuvers/ManeuverSegmenter.hpp"
#include "p4/maneuvers/MajorExcursionDetector.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr
            << "Usage: major_excursion_analysis "
            << "<file1.vbo> [file2.vbo ...]\n";
        return 1;
    }

    p4::io::VBOReader reader;
    p4::maneuvers::ManeuverSegmenter segmenter;
    p4::maneuvers::MajorExcursionDetector detector;

    std::cout << std::fixed
              << std::setprecision(3);

    int failures = 0;

    for (int f = 1; f < argc; ++f) {
        try {
            const std::string filename = argv[f];

            const auto data = reader.read(filename);
            const auto events = segmenter.analyze(data);
            const auto excursions =
                detector.analyze(data, events);

            std::cout
                << "\n=== " << filename << " ===\n"
                << "Events     : " << events.size() << '\n'
                << "Excursions : " << excursions.size()
                << '\n';

            for (const auto& e : excursions) {
                std::cout
                    << "  Event " << e.event_id
                    << " / Excursion " << e.excursion_id
                    << " [" << e.start_s
                    << ", " << e.end_s << "] s"
                    << " | "
                    << p4::maneuvers::toString(e.type)
                    << " | Delta="
                    << e.steering_delta_deg
                    << " deg\n";
            }
        }
        catch (const std::exception& e) {
            ++failures;

            std::cerr
                << "[ERROR] " << argv[f]
                << ": " << e.what() << '\n';
        }
    }

    return failures == 0 ? 0 : 1;
}
