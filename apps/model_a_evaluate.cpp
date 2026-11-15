
#include "p4/io/VBOReader.hpp"
#include "p4/identification/ManifestEvent.hpp"
#include "p4/models/ModelARollout.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct Model {
    double tau = 0.0;
    double gain = 0.0;
};

using Event = p4::identification::ManifestEvent;

struct Metrics {
    std::size_t samples = 0;
    double sse = 0.0;
    double sae = 0.0;
    double mean_yaw = 0.0;
    double sst = 0.0;

    double rmse() const {
        return std::sqrt(sse / samples);
    }

    double mae() const {
        return sae / samples;
    }

    double r2() const {
        return sst > 1e-12
            ? 1.0 - sse / sst
            : std::numeric_limits<double>::quiet_NaN();
    }
};

std::string readText(const fs::path& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error(
            "Cannot open: " + path.string()
        );
    }

    return {
        std::istreambuf_iterator<char>(file),
        std::istreambuf_iterator<char>()
    };
}

// Reads the exact flat parameter keys written by
// our current model_a_train.cpp.
// This is not a general-purpose JSON parser.
double jsonNumber(
    const std::string& text,
    const std::string& key
) {
    const std::regex pattern(
        "\"" + key + "\"\\s*:\\s*"
        "(-?[0-9]+(?:\\.[0-9]+)?"
        "(?:[eE][+-]?[0-9]+)?)"
    );

    std::smatch match;

    if (!std::regex_search(text, match, pattern)) {
        throw std::runtime_error(
            "Missing model parameter: " + key
        );
    }

    return std::stod(match[1].str());
}

Model loadModel(const fs::path& path) {
    const std::string text = readText(path);

    if (text.find("\"FirstOrderYawModel\"") ==
            std::string::npos ||
        jsonNumber(text, "schema_version") != 1.0 ||
        text.find("\"steer_ang2\"") ==
            std::string::npos ||
        text.find("\"YawRate\"") ==
            std::string::npos) {
        throw std::runtime_error(
            "Unsupported model schema or channels"
        );
    }

    Model model;
    model.gain = jsonNumber(text, "Kr");
    model.tau = jsonNumber(text, "tau_r");

    if (!std::isfinite(model.gain) ||
        !std::isfinite(model.tau) ||
        model.tau <= 0.0) {
        throw std::runtime_error(
            "Invalid saved model parameters"
        );
    }

    return model;
}

std::vector<std::string> parseCsv(
    const std::string& line
) {
    std::vector<std::string> result;
    std::string field;
    bool quoted = false;

    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];

        if (c == '"') {
            if (quoted && i + 1 < line.size() &&
                line[i + 1] == '"') {
                field += '"';
                ++i;
            } else {
                quoted = !quoted;
            }
        } else if (c == ',' && !quoted) {
            result.push_back(field);
            field.clear();
        } else {
            field += c;
        }
    }

    if (quoted) {
        throw std::runtime_error(
            "Invalid CSV quoting"
        );
    }

    if (!field.empty() && field.back() == '\r') {
        field.pop_back();
    }

    result.push_back(field);
    return result;
}

std::vector<Event> loadManifest(
    const fs::path& path
) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error(
            "Cannot open test manifest"
        );
    }

    std::string line;
    if (!std::getline(file, line)) {
        throw std::runtime_error("Empty CSV");
    }

    const auto header = parseCsv(line);
    std::map<std::string, std::size_t> columns;

    for (std::size_t i = 0; i < header.size(); ++i) {
        columns[header[i]] = i;
    }

    for (const char* key : {
        "file", "group_id", "event_id",
        "start_s", "end_s", "maneuver"
    }) {
        if (!columns.count(key)) {
            throw std::runtime_error(
                std::string("Missing CSV column: ") + key
            );
        }
    }

    std::vector<Event> events;

    while (std::getline(file, line)) {
        if (line.empty() || line == "\r") {
            continue;
        }

        const auto fields = parseCsv(line);
        if (fields.size() != header.size()) {
            throw std::runtime_error(
                "Invalid CSV row"
            );
        }

        auto get = [&](const std::string& key)
            -> const std::string& {
            return fields.at(columns.at(key));
        };

        Event e;
        e.file = get("file");
        e.group_id = get("group_id");
        e.maneuver = get("maneuver");
        e.event_id = std::stoull(get("event_id"));
        e.start_s = std::stod(get("start_s"));
        e.end_s = std::stod(get("end_s"));

        if (!std::isfinite(e.start_s) ||
            !std::isfinite(e.end_s) ||
            e.start_s < 0.0 || e.end_s <= e.start_s ||
            e.file.empty() || e.group_id.empty()) {
            throw std::runtime_error(
                "Invalid test event"
            );
        }

        events.push_back(e);
    }

    if (events.empty()) {
        throw std::runtime_error(
            "No test events"
        );
    }

    return events;
}

Metrics evaluate(
    const p4::io::VBOData& data,
    const Event& e,
    const Model& model,
    std::ostream* prediction_csv = nullptr
) {
    for (const char* key : {
        "steer_ang2", "YawRate", "velocity"
    }) {
        if (!data.hasColumn(key)) {
            throw std::runtime_error(
                std::string("Missing channel: ") + key
            );
        }
    }

    std::vector<double> t, u, y;
    const double t0 = data.timeAt(0);

    for (std::size_t i = 0;
         i < data.sampleCount(); ++i) {

        const double ti = data.timeAt(i) - t0;

        if (ti < e.start_s - 1e-9 ||
            ti > e.end_s + 1e-9) {
            continue;
        }

        const double ui = data.value(i, "steer_ang2");
        const double yi = data.value(i, "YawRate");
        const double vi = data.value(i, "velocity");

        if (!std::isfinite(ti) ||
            !std::isfinite(ui) ||
            !std::isfinite(yi) ||
            !std::isfinite(vi)) {
            throw std::runtime_error(
                "Non-finite measurement"
            );
        }

        if (!t.empty() && ti <= t.back()) {
            throw std::runtime_error(
                "Non-increasing timestamps"
            );
        }

        t.push_back(ti);
        u.push_back(ui);
        y.push_back(yi);
    }

    if (t.size() < 10) {
        throw std::runtime_error(
            "Too few test samples"
        );
    }

    // Frozen model: no optimization, no refitting.
    const auto prediction = p4::models::simulateModelA(
        t, u, y.front(), model.tau, model.gain
    );

    // Export measured and predicted yaw rates.
    // No parameter fitting is performed here.
    if (prediction_csv != nullptr) {
        *prediction_csv
            << "time_s,steer,yaw_measured,"
            << "yaw_predicted,error\n";

        *prediction_csv
            << std::fixed << std::setprecision(9);

        for (std::size_t k = 0; k < t.size(); ++k) {
            const double error =
                y[k] - prediction[k];

            *prediction_csv
                << t[k] - t.front() << ','
                << u[k] << ','
                << y[k] << ','
                << prediction[k] << ','
                << error << '\n';
        }
    }

    Metrics result;
    result.samples = y.size();

    for (double value : y) {
        result.mean_yaw += value;
    }
    result.mean_yaw /= y.size();

    for (std::size_t k = 0; k < y.size(); ++k) {
        const double error = y[k] - prediction[k];
        const double dy = y[k] - result.mean_yaw;

        result.sse += error * error;
        result.sae += std::abs(error);
        result.sst += dy * dy;
    }

    return result;
}

struct Aggregate {
    std::size_t events = 0;
    double sum_event_mse = 0.0;
    double sum_event_mae = 0.0;

    void add(const Metrics& m) {
        ++events;
        sum_event_mse += m.rmse() * m.rmse();
        sum_event_mae += m.mae();
    }

    double rmse() const {
        return std::sqrt(sum_event_mse / events);
    }

    double mae() const {
        return sum_event_mae / events;
    }
};

} // namespace

int main(int argc, char** argv) {
    if (argc != 3 && argc != 4) {
        std::cerr
            << "Usage: model_a_evaluate "
            << "<model.json> <test_manifest.csv> "
            << "[prediction_output_dir]\n";
        return 1;
    }

    try {
        const Model model = loadModel(argv[1]);
        const auto events = loadManifest(argv[2]);

        const fs::path prediction_dir =
            argc == 4 ? fs::path(argv[3])
                    : fs::path("output/predictions");

        fs::create_directories(prediction_dir);

        p4::io::VBOReader reader;

        std::map<std::string, Aggregate> by_maneuver;
        Aggregate overall;

        std::string cached_file;
        // VBOData is loaded per file; events remain in
        // their manifest order and share no parameters.

        std::cout << std::fixed << std::setprecision(6)
            << "\n=== FROZEN MODEL TEST ===\n"
            << "Model Tau  : " << model.tau << " s\n"
            << "Model Gain : " << model.gain << " 1/s\n"
            << "Test events: " << events.size() << "\n\n";

        for (const auto& e : events) {
            const auto data = reader.read(e.file);

            const fs::path csv_path =
                prediction_dir /
                (e.group_id + "_event_" +
                std::to_string(e.event_id) + ".csv");

            std::ofstream prediction_file(csv_path);

            if (!prediction_file) {
                throw std::runtime_error(
                    "Cannot create prediction CSV: " +
                    csv_path.string()
                );
            }

            const Metrics m = evaluate(
                data,
                e,
                model,
                &prediction_file
            );

            prediction_file.close();

            if (!prediction_file) {
                throw std::runtime_error(
                    "Prediction CSV write failed: " +
                    csv_path.string()
                );
            }

            overall.add(m);
            by_maneuver[e.maneuver].add(m);

            std::cout
                << e.group_id << " / Event " << e.event_id
                << " | " << e.maneuver
                << " | RMSE=" << m.rmse()
                << " | MAE=" << m.mae()
                << " | R2=" << m.r2()
                << '\n';
        }

        std::cout
            << "\n=== TEST SUMMARY ===\n"
            << "Events : " << overall.events << '\n'
            << "RMSE   : " << overall.rmse()
            << " deg/s (equal event weight)\n"
            << "MAE    : " << overall.mae()
            << " deg/s (equal event weight)\n";

        std::cout
            << "\n=== TEST BY MANEUVER ===\n";

        for (const auto& item : by_maneuver) {
            const auto& a = item.second;
            std::cout
                << item.first
                << " | Events=" << a.events
                << " | RMSE=" << a.rmse()
                << " deg/s"
                << " | MAE=" << a.mae()
                << " deg/s\n";
        }

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr
            << "[ERROR] " << e.what() << '\n';
        return 1;
    }
}
