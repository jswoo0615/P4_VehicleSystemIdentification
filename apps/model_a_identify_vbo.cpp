
#include "p4/io/VBOReader.hpp"
#include "p4/models/ModelARollout.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Samples {
    std::vector<double> time;
    std::vector<double> steer;
    std::vector<double> yaw;
    std::vector<double> speed;
};

struct FitResult {
    double tau_s = 0.0;
    double gain = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
    double mae = 0.0;
    double r2 = 0.0;
    double basis_energy = 0.0;
    std::size_t samples = 0;
    bool valid = false;
};

// ============================================================
// 1. Extract continuous measurement interval
// ============================================================

Samples extract(
    const p4::io::VBOData& data,
    double start_s,
    double end_s
) {
    if (!(end_s > start_s)) {
        throw std::invalid_argument("Invalid time range");
    }

    for (const char* name :
         {"steer_ang2", "YawRate", "velocity"}) {
        if (!data.hasColumn(name)) {
            throw std::runtime_error(
                std::string("Missing column: ") + name
            );
        }
    }

    Samples out;

    const std::size_t n = data.sampleCount();

    if (n < 2) {
        throw std::runtime_error("Insufficient VBO samples");
    }

    const double t0 = data.timeAt(0);

    for (std::size_t i = 0; i < n; ++i) {
        const double t = data.timeAt(i) - t0;

        if (t < start_s || t > end_s) {
            continue;
        }

        const double u = data.value(i, "steer_ang2");
        const double r = data.value(i, "YawRate");
        const double v = data.value(i, "velocity");

        if (!std::isfinite(t) ||
            !std::isfinite(u) ||
            !std::isfinite(r) ||
            !std::isfinite(v)) {
            throw std::runtime_error(
                "Non-finite measurement"
            );
        }

        if (!out.time.empty() &&
            t <= out.time.back()) {
            throw std::runtime_error(
                "Non-increasing timestamps"
            );
        }

        out.time.push_back(t);
        out.steer.push_back(u);
        out.yaw.push_back(r);
        out.speed.push_back(v);
    }

    if (out.time.size() < 30) {
        throw std::runtime_error(
            "Insufficient samples in selected interval"
        );
    }

    return out;
}

// ============================================================
// 2. Simulate the first-order yaw model
// ============================================================

// The measured initial yaw rate is used only as the
// initial state of the continuous rollout.

std::vector<double> simulate(
    const Samples& s,
    double tau,
    double gain
) {
    return p4::models::simulateModelA(
        s.time, s.steer, s.yaw.at(0), tau, gain
    );
}

// ============================================================
// 3. Estimate optimal gain at a fixed tau
// ============================================================

FitResult fitFixedTau(
    const Samples& s,
    double tau
) {
    FitResult result;
    result.tau_s = tau;
    result.samples = s.time.size();

    const std::size_t n = s.time.size();

    // Predicted response = p + gain * q
    double p = s.yaw[0];
    double q = 0.0;

    double numerator = 0.0;
    double denominator = 0.0;

    for (std::size_t k = 1; k < n; ++k) {
        const double dt =
            s.time[k] - s.time[k - 1];

        const double a = std::exp(-dt / tau);

        p = a * p;
        q = a * q +
            (1.0 - a) * s.steer[k - 1];

        numerator += q * (s.yaw[k] - p);
        denominator += q * q;
    }

    result.basis_energy = denominator;

    if (denominator <= 1e-12 ||
        !std::isfinite(denominator)) {
        return result;
    }

    result.gain = numerator / denominator;

    const auto predicted = simulate(
        s, tau, result.gain
    );

    double sum_error2 = 0.0;
    double sum_abs_error = 0.0;

    double mean_yaw = 0.0;
    for (double value : s.yaw) {
        mean_yaw += value;
    }
    mean_yaw /= static_cast<double>(n);

    double total_variation = 0.0;

    for (std::size_t k = 0; k < n; ++k) {
        const double error =
            s.yaw[k] - predicted[k];

        sum_error2 += error * error;
        sum_abs_error += std::abs(error);

        const double deviation =
            s.yaw[k] - mean_yaw;

        total_variation += deviation * deviation;
    }

    result.rmse = std::sqrt(
        sum_error2 / static_cast<double>(n)
    );

    result.mae =
        sum_abs_error / static_cast<double>(n);

    result.r2 = total_variation > 1e-12
        ? 1.0 - sum_error2 / total_variation
        : std::numeric_limits<double>::quiet_NaN();

    result.valid =
        std::isfinite(result.gain) &&
        std::isfinite(result.rmse) &&
        std::isfinite(result.mae);

    return result;
}

// ============================================================
// 4. One-dimensional log-spaced tau search
// ============================================================

FitResult identify(
    const Samples& s,
    double min_tau = 0.03,
    double max_tau = 3.0,
    int grid_count = 301
) {
    if (!(min_tau > 0.0) ||
        !(max_tau > min_tau) ||
        grid_count < 3) {
        throw std::invalid_argument(
            "Invalid identification search settings"
        );
    }

    FitResult best;

    const double log_min = std::log(min_tau);
    const double log_max = std::log(max_tau);

    for (int i = 0; i < grid_count; ++i) {
        const double fraction =
            static_cast<double>(i) /
            static_cast<double>(grid_count - 1);

        const double tau = std::exp(
            log_min + fraction * (log_max - log_min)
        );

        const auto candidate =
            fitFixedTau(s, tau);

        if (candidate.valid &&
            candidate.rmse < best.rmse) {
            best = candidate;
        }
    }

    if (!best.valid) {
        throw std::runtime_error(
            "Model A identification failed"
        );
    }

    return best;
}


struct DiagnosticResult {
    double tau_s = 0.0;
    double gain = 0.0;
    double rmse = 0.0;
    double mae = 0.0;
    double r2 = 0.0;
};

// Evaluate fixed parameters without any refitting.
DiagnosticResult evaluateParameters(
    const Samples& s,
    double tau,
    double gain
) {
    const auto predicted = simulate(s, tau, gain);

    const std::size_t n = s.yaw.size();

    double mean = 0.0;
    for (double y : s.yaw) {
        mean += y;
    }
    mean /= static_cast<double>(n);

    double sse = 0.0;
    double sae = 0.0;
    double sst = 0.0;

    for (std::size_t i = 0; i < n; ++i) {
        const double error = s.yaw[i] - predicted[i];

        sse += error * error;
        sae += std::abs(error);

        const double centered = s.yaw[i] - mean;
        sst += centered * centered;
    }

    DiagnosticResult result;

    result.tau_s = tau;
    result.gain = gain;
    result.rmse = std::sqrt(sse / n);
    result.mae = sae / n;

    result.r2 = (sst > 1e-12)
        ? 1.0 - sse / sst
        : std::numeric_limits<double>::quiet_NaN();

    return result;
}

// Experiment C:
// Optimize tau only while keeping gain frozen.
DiagnosticResult identifyTauWithFixedGain(
    const Samples& s,
    double fixed_gain,
    double min_tau = 0.03,
    double max_tau = 3.0,
    int grid_count = 301
) {
    if (!(min_tau > 0.0) ||
        !(max_tau > min_tau) ||
        grid_count < 3) {
        throw std::invalid_argument(
            "Invalid tau search configuration"
        );
    }

    DiagnosticResult best;
    best.rmse = std::numeric_limits<double>::infinity();

    const double log_min = std::log(min_tau);
    const double log_max = std::log(max_tau);

    for (int i = 0; i < grid_count; ++i) {
        const double fraction =
            static_cast<double>(i) /
            static_cast<double>(grid_count - 1);

        const double tau = std::exp(
            log_min + fraction * (log_max - log_min)
        );

        const auto candidate = evaluateParameters(
            s, tau, fixed_gain
        );

        if (candidate.rmse < best.rmse) {
            best = candidate;
        }
    }

    return best;
}

// Print the four experiments on the same dataset.
void runParameterDiagnostics(
    const Samples& validation,
    double frozen_tau,
    double frozen_gain
) {
    // A: Both parameters frozen.
    const auto A = evaluateParameters(
        validation, frozen_tau, frozen_gain
    );

    // B: Refit gain, freeze tau.
    const auto gain_fit = fitFixedTau(
        validation, frozen_tau
    );

    if (!gain_fit.valid) {
        throw std::runtime_error(
            "Gain-only diagnostic fitting failed"
        );
    }

    const auto B = evaluateParameters(
        validation,
        frozen_tau,
        gain_fit.gain
    );

    // C: Refit tau, freeze gain.
    const auto C = identifyTauWithFixedGain(
        validation, frozen_gain
    );

    // D: Refit both parameters on validation data.
    const auto full_fit = identify(validation);

    const auto D = evaluateParameters(
        validation,
        full_fit.tau_s,
        full_fit.gain
    );

    std::cout
        << "\n=== PARAMETER GENERALIZATION DIAGNOSTICS ===\n"
        << "All metrics are evaluated on the same "
        << "validation interval.\n\n";

    std::cout
        << std::left
        << std::setw(12) << "Experiment"
        << std::right
        << std::setw(14) << "Tau(s)"
        << std::setw(14) << "Gain"
        << std::setw(14) << "RMSE"
        << std::setw(14) << "R2"
        << '\n';

    auto printRow = [](const char* name,
                       const DiagnosticResult& r) {
        std::cout
            << std::left
            << std::setw(12) << name
            << std::right
            << std::setw(14) << r.tau_s
            << std::setw(14) << r.gain
            << std::setw(14) << r.rmse
            << std::setw(14) << r.r2
            << '\n';
    };

    printRow("A_FROZEN", A);
    printRow("B_GAIN", B);
    printRow("C_TAU", C);
    printRow("D_BOTH", D);

    const auto improvement = [&](double rmse) {
        return 100.0 * (A.rmse - rmse) / A.rmse;
    };

    std::cout
        << "\n=== RMSE REDUCTION VS FROZEN MODEL ===\n"
        << "Gain only : " << improvement(B.rmse)
        << " %\n"
        << "Tau only  : " << improvement(C.rmse)
        << " %\n"
        << "Both      : " << improvement(D.rmse)
        << " %\n";

    if (C.tau_s <= 0.030001 ||
        C.tau_s >= 2.999999 ||
        D.tau_s <= 0.030001 ||
        D.tau_s >= 2.999999) {
        std::cout
            << "WARNING: A diagnostic tau is at "
            << "the search boundary.\n";
    }
}


// ============================================================
// 5. Independent interval evaluation
// ============================================================

void evaluate(
    const Samples& s,
    double tau,
    double gain,
    const std::string& label
) {
    const auto predicted = simulate(s, tau, gain);

    double squared_error = 0.0;
    double absolute_error = 0.0;

    for (std::size_t i = 0; i < s.yaw.size(); ++i) {
        const double error = s.yaw[i] - predicted[i];

        squared_error += error * error;
        absolute_error += std::abs(error);
    }

    const double n = static_cast<double>(s.yaw.size());

    std::cout
        << "\n=== " << label << " ===\n"
        << "Samples : " << s.yaw.size() << '\n'
        << "RMSE    : " << std::sqrt(squared_error / n)
        << " deg/s\n"
        << "MAE     : " << absolute_error / n
        << " deg/s\n";
}

// ============================================================
// 6. Main
// ============================================================

} // anonymous namespace


int main(int argc, char** argv) {
    // Train only:
    // model_a_identify_vbo train.vbo t0 t1
    //
    // Train + independent validation:
    // model_a_identify_vbo
    //     train.vbo t0 t1
    //     validation.vbo t0 t1

    if (argc != 4 && argc != 7) {
        std::cerr
            << "Usage:\n"
            << "  model_a_identify_vbo "
            << "<train.vbo> <train_start> <train_end>\n"
            << "\n"
            << "  model_a_identify_vbo "
            << "<train.vbo> <train_start> <train_end> "
            << "<validation.vbo> <valid_start> <valid_end>\n";

        return 1;
    }

    try {
        // ====================================================
        // 1. Training arguments
        // ====================================================

        const std::string train_file = argv[1];

        const double train_start = std::stod(argv[2]);
        const double train_end = std::stod(argv[3]);

        if (!std::isfinite(train_start) ||
            !std::isfinite(train_end) ||
            train_start < 0.0 ||
            train_end <= train_start) {
            throw std::invalid_argument(
                "Invalid training interval"
            );
        }

        p4::io::VBOReader reader;

        const auto train_data = reader.read(train_file);

        const auto train = extract(
            train_data,
            train_start,
            train_end
        );

        // ====================================================
        // 2. Parameter identification — ONLY ONCE
        // ====================================================

        const auto fit = identify(train);

        // Freeze fitted parameters.
        const double frozen_tau = fit.tau_s;
        const double frozen_gain = fit.gain;

        double train_speed_sum = 0.0;

        for (double speed : train.speed) {
            train_speed_sum += speed;
        }

        const double train_mean_speed =
            train_speed_sum /
            static_cast<double>(train.speed.size());

        std::cout
            << std::fixed
            << std::setprecision(6)
            << "\n=== MODEL A TRAINING ===\n"
            << "Train file : " << train_file << '\n'
            << "Interval   : ["
            << train_start << ", "
            << train_end << "] s\n"
            << "Samples    : " << fit.samples << '\n'
            << "Tau        : " << frozen_tau << " s\n"
            << "Gain       : " << frozen_gain << " 1/s\n"
            << "Train RMSE : " << fit.rmse << " deg/s\n"
            << "Train MAE  : " << fit.mae << " deg/s\n"
            << "Train R2   : " << fit.r2 << '\n'
            << "Mean speed : "
            << train_mean_speed << " km/h\n";

        if (frozen_tau <= 0.030001 ||
            frozen_tau >= 2.999999) {
            std::cout
                << "WARNING: Tau reached search boundary.\n";
        }

        // ====================================================
        // 3. Independent validation (optional)
        // ====================================================

        if (argc == 7) {
            const std::string validation_file = argv[4];

            const double valid_start = std::stod(argv[5]);
            const double valid_end = std::stod(argv[6]);

            if (!std::isfinite(valid_start) ||
                !std::isfinite(valid_end) ||
                valid_start < 0.0 ||
                valid_end <= valid_start) {
                throw std::invalid_argument(
                    "Invalid validation interval"
                );
            }

            // Disallow overlap when using the same file.
            if (train_file == validation_file) {
                const bool overlap =
                    valid_start < train_end &&
                    valid_end > train_start;

                if (overlap) {
                    throw std::invalid_argument(
                        "Training and validation "
                        "intervals overlap"
                    );
                }
            }

            const auto validation_data =
                reader.read(validation_file);

            const auto validation = extract(
                validation_data,
                valid_start,
                valid_end
            );

            double valid_speed_sum = 0.0;

            for (double speed : validation.speed) {
                valid_speed_sum += speed;
            }

            const double valid_mean_speed =
                valid_speed_sum /
                static_cast<double>(
                    validation.speed.size()
                );

            std::cout
                << "\n=== FROZEN PARAMETER VALIDATION ===\n"
                << "Validation file : "
                << validation_file << '\n'
                << "Interval        : ["
                << valid_start << ", "
                << valid_end << "] s\n"
                << "Mean speed      : "
                << valid_mean_speed << " km/h\n"
                << "Frozen Tau      : "
                << frozen_tau << " s\n"
                << "Frozen Gain     : "
                << frozen_gain << " 1/s\n";

            // IMPORTANT:
            // No identify(validation) here.
            //
            // Only simulate the validation data
            // using the frozen training parameters.

            evaluate(
                validation,
                frozen_tau,
                frozen_gain,
                "VALIDATION RESULT"
            );

            // Diagnostic experiments.
            // These are NOT independent validation results.
            // The validation dataset is used for diagnostic refitting.
            runParameterDiagnostics(
                validation,
                frozen_tau,
                frozen_gain
            );
        }

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr
            << "[ERROR] " << e.what() << '\n';

        return 1;
    }
}
