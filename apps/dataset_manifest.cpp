
#include "p4/io/VBOReader.hpp"
#include "p4/maneuvers/ManeuverSegmenter.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

namespace {

// CSV fields must be escaped when paths contain commas or quotes.
std::string csvField(const std::string& value) {
    std::string result = "\"";

    for (const char ch : value) {
        if (ch == '"') {
            result += "\"\"";
        } else {
            result += ch;
        }
    }

    result += '"';
    return result;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr
            << "Usage: dataset_manifest "
            << "<output.csv> <file1.vbo> [file2.vbo ...]\n";
        return 1;
    }

    try {
        const fs::path output_path = argv[1];

        if (!output_path.parent_path().empty()) {
            fs::create_directories(
                output_path.parent_path()
            );
        }

        std::ofstream csv(output_path);

        if (!csv) {
            throw std::runtime_error(
                "Cannot create manifest CSV"
            );
        }

        csv
            << "file,group_id,event_id,"
            << "start_s,end_s,maneuver,"
            << "mean_speed_kmh,max_abs_steer_deg\n";

        p4::io::VBOReader reader;
        p4::maneuvers::ManeuverSegmenter segmenter;

        std::size_t processed = 0;
        std::size_t total_events = 0;
        std::size_t failures = 0;

        csv << std::fixed << std::setprecision(6);

        for (int i = 2; i < argc; ++i) {
            const fs::path path = argv[i];
            const std::string filename =
                path.filename().string();

            // Previously audited exact duplicate:
            // vblog_042.vbo == vblog_035.vbo
            // Keep only one copy in the dataset.
            if (filename == "vblog_042.vbo") {
                std::cout
                    << "[DUPLICATE SKIP] "
                    << filename << '\n';
                continue;
            }

            try {
                const auto data =
                    reader.read(path.string());

                const auto events =
                    segmenter.analyze(data);

                // One group per recording file.
                // Every event in a file stays together.
                const std::string group_id =
                    path.stem().string();

                for (const auto& event : events) {
                    csv
                        << csvField(path.string()) << ','
                        << csvField(group_id) << ','
                        << event.event_id << ','
                        << event.start_s << ','
                        << event.end_s << ','
                        << csvField(
                            p4::maneuvers::toString(
                                event.candidate
                            )
                        ) << ','
                        << event.mean_speed_kmh << ','
                        << event.max_abs_steer_deg
                        << '\n';

                    ++total_events;
                }

                ++processed;

                std::cout
                    << "[OK] " << filename
                    << " | events=" << events.size()
                    << '\n';
            }
            catch (const std::exception& e) {
                ++failures;

                std::cerr
                    << "[ERROR] " << filename
                    << ": " << e.what() << '\n';
            }
        }

        csv.close();

        std::cout
            << "\n=== DATASET MANIFEST SUMMARY ===\n"
            << "Processed files : " << processed << '\n'
            << "Total events    : " << total_events << '\n'
            << "Failed files    : " << failures << '\n'
            << "Output          : "
            << output_path.string() << '\n';

        return failures == 0 ? 0 : 1;
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << '\n';
        return 1;
    }
}
