
#include "p4/io/VBOReader.hpp"
#include "p4/identification/ManifestEvent.hpp"
#include "p4/models/ModelARollout.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

// ============================================================
// 1. Data structures
// ============================================================

using ManifestRow = p4::identification::ManifestEvent;

struct EventData {
    ManifestRow meta;
    std::vector<double> time;
    std::vector<double> steer;
    std::vector<double> yaw;
    double mean_speed_kmh = 0.0;
};

struct FitResult {
    double tau_s = 0.0;
    double gain = 0.0;
    double objective_mse =
        std::numeric_limits<double>::infinity();
    bool valid = false;
};

struct Metrics {
    double rmse = 0.0;
    double mae = 0.0;
    double r2 = 0.0;
};

// ============================================================
// 2. CSV parsing
// ============================================================

// Handles quoted fields, escaped double quotes,
// and commas within quoted file paths.
std::vector<std::string> parseCsvLine(
    const std::string& line
) {
    std::vector<std::string> fields;
    std::string current;
    bool quoted = false;

    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];

        if (c == '"') {
            if (quoted &&
                i + 1 < line.size() &&
                line[i + 1] == '"') {
                current += '"';
                ++i;
            } else {
                quoted = !quoted;
            }
        } else if (c == ',' && !quoted) {
            fields.push_back(current);
            current.clear();
        } else {
            current += c;
        }
    }

    if (quoted) {
        throw std::runtime_error(
            "Unterminated quoted CSV field"
        );
    }

    if (!current.empty() && current.back() == '\r') {
        current.pop_back();
    }

    fields.push_back(current);
    return fields;
}

double parseDouble(const std::string& s) {
    std::size_t used = 0;
    const double value = std::stod(s, &used);

    if (used != s.size() || !std::isfinite(value)) {
        throw std::runtime_error(
            "Invalid numeric value: " + s
        );
    }

    return value;
}

std::size_t parseSize(const std::string& s) {
    std::size_t used = 0;
    const unsigned long long value =
        std::stoull(s, &used);

    if (used != s.size() ||
        value == 0 ||
        value > std::numeric_limits<std::size_t>::max()) {
        throw std::runtime_error(
            "Invalid event ID: " + s
        );
    }

    return static_cast<std::size_t>(value);
}

std::vector<ManifestRow> readManifest(
    const fs::path& filename
) {
    std::ifstream in(filename);

    if (!in) {
        throw std::runtime_error(
            "Cannot open manifest: " + filename.string()
        );
    }

    std::string line;

    if (!std::getline(in, line)) {
        throw std::runtime_error("Empty manifest");
    }

    const auto headers = parseCsvLine(line);

    std::map<std::string, std::size_t> columns;

    for (std::size_t i = 0; i < headers.size(); ++i) {
        columns[headers[i]] = i;
    }

    for (const char* required : {
        "file", "group_id", "event_id",
        "start_s", "end_s", "maneuver"
    }) {
        if (!columns.count(required)) {
            throw std::runtime_error(
                std::string("Missing manifest column: ") +
                required
            );
        }
    }

    std::vector<ManifestRow> rows;
    std::set<std::pair<std::string, std::size_t>> seen;

    std::size_t line_number = 1;

    while (std::getline(in, line)) {
        ++line_number;

        if (line.empty() || line == "\r") {
            continue;
        }

        const auto values = parseCsvLine(line);

        if (values.size() != headers.size()) {
            throw std::runtime_error(
                "CSV column count mismatch at line " +
                std::to_string(line_number)
            );
        }

        auto get = [&](const std::string& key)
            -> const std::string& {
            return values.at(columns.at(key));
        };

        ManifestRow row;

        row.file = get("file");
        row.group_id = get("group_id");
        row.maneuver = get("maneuver");
        row.event_id = parseSize(get("event_id"));
        row.start_s = parseDouble(get("start_s"));
        row.end_s = parseDouble(get("end_s"));

        if (row.file.empty() ||
            row.group_id.empty() ||
            row.maneuver.empty() ||
            row.start_s < 0.0 ||
            row.end_s <= row.start_s) {
            throw std::runtime_error(
                "Invalid manifest row at line " +
                std::to_string(line_number)
            );
        }

        if (!seen.insert({
            row.group_id, row.event_id
        }).second) {
            throw std::runtime_error(
                "Duplicate group/event ID in manifest"
            );
        }

        rows.push_back(row);
    }

    if (rows.empty()) {
        throw std::runtime_error(
            "Manifest contains no events"
        );
    }

    return rows;
}

// ============================================================
// 3. Load event measurements
// ============================================================

EventData loadEvent(
    const p4::io::VBOData& data,
    const ManifestRow& row
) {
    EventData out;
    out.meta = row;

    for (const char* column : {
        "steer_ang2", "YawRate", "velocity"
    }) {
        if (!data.hasColumn(column)) {
            throw std::runtime_error(
                std::string("Missing VBO column: ") +
                column
            );
        }
    }

    if (data.sampleCount() < 2) {
        throw std::runtime_error(
            "VBO contains insufficient samples"
        );
    }

    const double t0 = data.timeAt(0);
    double speed_sum = 0.0;

    for (std::size_t i = 0;
         i < data.sampleCount(); ++i) {

        const double t = data.timeAt(i) - t0;

        // End point is included in this training
        // interval, consistent with the earlier
        // single-event identifier.
        if (t < row.start_s - 1e-9 ||
            t > row.end_s + 1e-9) {
            continue;
        }

        const double steer =
            data.value(i, "steer_ang2");
        const double yaw =
            data.value(i, "YawRate");
        const double speed =
            data.value(i, "velocity");

        if (!std::isfinite(t) ||
            !std::isfinite(steer) ||
            !std::isfinite(yaw) ||
            !std::isfinite(speed)) {
            throw std::runtime_error(
                "Non-finite measurement in " + row.file
            );
        }

        if (!out.time.empty() &&
            t <= out.time.back()) {
            throw std::runtime_error(
                "Non-increasing time in " + row.file
            );
        }

        out.time.push_back(t);
        out.steer.push_back(steer);
        out.yaw.push_back(yaw);
        speed_sum += speed;
    }

    if (out.time.size() < 10) {
        throw std::runtime_error(
            "Too few samples for event in " + row.file
        );
    }

    out.mean_speed_kmh =
        speed_sum / static_cast<double>(
            out.time.size()
        );

    return out;
}

std::vector<EventData> loadDataset(
    const std::vector<ManifestRow>& rows
) {
    p4::io::VBOReader reader;

    std::vector<EventData> result;
    result.reserve(rows.size());

    // Cache one parsed VBO per filename.
    // Processing in sorted order avoids repeated reads.
    std::map<std::string, std::vector<ManifestRow>> by_file;

    for (const auto& row : rows) {
        by_file[row.file].push_back(row);
    }

    for (const auto& item : by_file) {
        const auto data = reader.read(item.first);

        for (const auto& row : item.second) {
            result.push_back(loadEvent(data, row));
        }
    }

    return result;
}

// ============================================================
// 4. Simulate first-order yaw response
// ============================================================

std::vector<double> simulate(
    const EventData& e,
    double tau,
    double gain
) {
    return p4::models::simulateModelA(
        e.time, e.steer, e.yaw.at(0), tau, gain
    );
}

// ============================================================
// 5. Joint batch fit at a fixed tau
// ============================================================

FitResult fitFixedTau(
    const std::vector<EventData>& events,
    double tau
) {
    FitResult fit;
    fit.tau_s = tau;

    double numerator = 0.0;
    double denominator = 0.0;

    // First pass:
    // shared least-squares gain across all events.
    for (const auto& e : events) {
        const std::size_t n = e.time.size();

        double p = e.yaw[0];
        double q = 0.0;

        const double weight =
            1.0 / static_cast<double>(n);

        for (std::size_t k = 1; k < n; ++k) {
            const double dt =
                e.time[k] - e.time[k - 1];

            const double a = std::exp(-dt / tau);

            p = a * p;
            q = a * q +
                (1.0 - a) * e.steer[k - 1];

            numerator +=
                weight * q * (e.yaw[k] - p);

            denominator += weight * q * q;
        }
    }

    if (!std::isfinite(denominator) ||
        denominator <= 1e-12) {
        return fit;
    }

    fit.gain = numerator / denominator;

    if (!std::isfinite(fit.gain)) {
        return fit;
    }

    // Second pass:
    // average of per-event mean squared errors.
    double total_mse = 0.0;

    for (const auto& e : events) {
        const auto pred = simulate(
            e, tau, fit.gain
        );

        double sse = 0.0;

        for (std::size_t k = 0;
             k < e.yaw.size(); ++k) {

            const double error =
                e.yaw[k] - pred[k];

            sse += error * error;
        }

        total_mse +=
            sse / static_cast<double>(e.yaw.size());
    }

    fit.objective_mse =
        total_mse / static_cast<double>(events.size());

    fit.valid = std::isfinite(fit.objective_mse);
    return fit;
}

// ============================================================
// 6. Shared parameter optimization
// ============================================================

FitResult identifyBatch(
    const std::vector<EventData>& events
) {
    const double min_tau = 0.03;
    const double max_tau = 3.0;
    const int grid_count = 301;

    const double log_min = std::log(min_tau);
    const double log_max = std::log(max_tau);

    FitResult best;

    for (int i = 0; i < grid_count; ++i) {
        const double fraction =
            static_cast<double>(i) /
            static_cast<double>(grid_count - 1);

        const double tau = std::exp(
            log_min + fraction * (log_max - log_min)
        );

        const auto candidate =
            fitFixedTau(events, tau);

        if (candidate.valid &&
            candidate.objective_mse < best.objective_mse) {
            best = candidate;
        }
    }

    if (!best.valid) {
        throw std::runtime_error(
            "Batch identification failed"
        );
    }

    return best;
}

// ============================================================
// 7. Metrics
// ============================================================

Metrics evaluateEvent(
    const EventData& e,
    double tau,
    double gain
) {
    const auto pred = simulate(e, tau, gain);

    const double n =
        static_cast<double>(e.yaw.size());

    double mean = 0.0;

    for (double r : e.yaw) {
        mean += r;
    }

    mean /= n;

    double sse = 0.0;
    double sae = 0.0;
    double sst = 0.0;

    for (std::size_t k = 0;
         k < e.yaw.size(); ++k) {

        const double error =
            e.yaw[k] - pred[k];

        sse += error * error;
        sae += std::abs(error);

        const double centered =
            e.yaw[k] - mean;

        sst += centered * centered;
    }

    Metrics m;

    m.rmse = std::sqrt(sse / n);
    m.mae = sae / n;

    m.r2 = sst > 1e-12
        ? 1.0 - sse / sst
        : std::numeric_limits<double>::quiet_NaN();

    return m;
}

// ============================================================
// 8. Save fitted model
// ============================================================

void saveModel(
    const fs::path& path,
    const FitResult& fit,
    std::size_t event_count,
    std::size_t group_count
) {
    if (!path.parent_path().empty()) {
        fs::create_directories(path.parent_path());
    }

    std::ofstream out(path);

    if (!out) {
        throw std::runtime_error(
            "Cannot write model: " + path.string()
        );
    }

    out << std::setprecision(17);

    out
        << "{\n"
        << "  \"schema_version\": 1,\n"
        << "  \"model\": \"FirstOrderYawModel\",\n"
        << "  \"input_channel\": \"steer_ang2\",\n"
        << "  \"output_channel\": \"YawRate\",\n"
        << "  \"time_unit\": \"s\",\n"
        << "  \"input_unit\": \"deg_unverified\",\n"
        << "  \"output_unit\": \"deg/s\",\n"
        << "  \"parameters\": {\n"
        << "    \"Kr\": " << fit.gain << ",\n"
        << "    \"tau_r\": " << fit.tau_s << "\n"
        << "  },\n"
        << "  \"training\": {\n"
        << "    \"event_count\": " << event_count << ",\n"
        << "    \"group_count\": " << group_count << ",\n"
        << "    \"objective\": \"mean_event_mse\",\n"
        << "    \"train_objective_mse\": "
        << fit.objective_mse << ",\n"
        << "    \"tau_min_s\": 0.03,\n"
        << "    \"tau_max_s\": 3.0,\n"
        << "    \"tau_grid_count\": 301\n"
        << "  }\n"
        << "}\n";

    out.flush();

    if (!out) {
        throw std::runtime_error(
            "Failed to write model file"
        );
    }
}

} // namespace

// ============================================================
// 9. Main
// ============================================================

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr
            << "Usage: model_a_train "
            << "<train_manifest.csv> <model_output.json>\n";
        return 1;
    }

    try {
        const fs::path manifest_path = argv[1];
        const fs::path model_path = argv[2];

        const auto rows = readManifest(manifest_path);

        std::set<std::string> group_ids;
        for (const auto& row : rows) {
            group_ids.insert(row.group_id);
        }

        const auto events = loadDataset(rows);

        const auto fit = identifyBatch(events);

        std::cout
            << std::fixed << std::setprecision(6)
            << "\n=== BATCH MODEL A TRAINING ===\n"
            << "Events         : " << events.size() << '\n'
            << "Groups         : " << group_ids.size() << '\n'
            << "Shared Tau     : " << fit.tau_s << " s\n"
            << "Shared Gain    : " << fit.gain << " 1/s\n"
            << "Objective MSE  : "
            << fit.objective_mse << '\n'
            << "Objective RMSE : "
            << std::sqrt(fit.objective_mse)
            << " deg/s\n";

        if (fit.tau_s <= 0.030001 ||
            fit.tau_s >= 2.999999) {
            std::cout
                << "[WARNING] Tau reached search boundary\n";
        }

        std::cout
            << "\n=== TRAIN PERFORMANCE BY MANEUVER ===\n";

        struct Aggregate {
            std::size_t count = 0;
            double sum_rmse2 = 0.0;
            double sum_mae = 0.0;
        };

        std::map<std::string, Aggregate> aggregates;

        for (const auto& e : events) {
            const auto metrics = evaluateEvent(
                e, fit.tau_s, fit.gain
            );

            auto& a = aggregates[e.meta.maneuver];

            ++a.count;
            a.sum_rmse2 +=
                metrics.rmse * metrics.rmse;
            a.sum_mae += metrics.mae;
        }

        for (const auto& item : aggregates) {
            const auto& a = item.second;

            std::cout
                << item.first
                << " | Events=" << a.count
                << " | RMSE="
                << std::sqrt(a.sum_rmse2 / a.count)
                << " deg/s"
                << " | MAE="
                << a.sum_mae / a.count
                << " deg/s\n";
        }

        saveModel(
            model_path,
            fit,
            events.size(),
            group_ids.size()
        );

        std::cout
            << "\nModel saved: " << model_path.string()
            << '\n';

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr
            << "[ERROR] " << e.what() << '\n';
        return 1;
    }
}
