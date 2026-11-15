
#include "p4/io/VBOReader.hpp"
#include "p4/maneuvers/ManeuverSegmenter.hpp"
#include "p4/maneuvers/MajorExcursionDetector.hpp"
#include "p4/maneuvers/ManeuverPhaseAnalyzer.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr
            << "Usage: maneuver_phase_analysis "
            << "<file1.vbo> [file2.vbo ...]\n";
        return 1;
    }

    p4::io::VBOReader reader;
    p4::maneuvers::ManeuverSegmenter segmenter;
    p4::maneuvers::MajorExcursionDetector excursionDetector;
    p4::maneuvers::ManeuverPhaseAnalyzer phaseAnalyzer;

    std::cout << std::fixed << std::setprecision(3);

    int failures = 0;

    for (int f = 1; f < argc; ++f) {
        try {
            const std::string filename = argv[f];

            const auto data = reader.read(filename);
            const auto events = segmenter.analyze(data);
            const auto excursions =
                excursionDetector.analyze(data, events);
            const auto phases =
                phaseAnalyzer.analyze(
                    data, events, excursions
                );

            std::cout
                << "\n=== " << filename << " ===\n"
                << "Events     : " << events.size() << '\n'
                << "Excursions : " << excursions.size() << '\n'
                << "Phases     : " << phases.size() << '\n';

            for (const auto& p : phases) {
                std::cout
                    << "  Event " << p.event_id
                    << " / Phase " << p.phase_id
                    << " [" << p.start_s
                    << ", " << p.end_s << "] s"
                    << " | "
                    << p4::maneuvers::toString(p.type)
                    << " | Steer="
                    << p.steer_start_deg << " -> "
                    << p.steer_end_deg << " deg"
                    << '\n';
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
