#include "p4/io/IONIQ5CsvReader.hpp"
#include "p4/models/FirstOrderYawModel.hpp"

#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
struct Result {
    double gain = 0, tau = 0, bias = 0;
    double rmse = std::numeric_limits<double>::infinity();
};

Result evaluate(const p4::io::ModelACsvSamples& s, Result parameters) {
    p4::models::FirstOrderYawModel model({parameters.gain, parameters.tau, parameters.bias});
    model.reset(s.yaw_rad_s.front());
    double error2 = 0;
    for (std::size_t k = 1; k < s.time_s.size(); ++k) {
        const double prediction = model.step(s.steering_rad[k - 1],
                                              s.time_s[k] - s.time_s[k - 1]);
        const double error = s.yaw_rad_s[k] - prediction;
        error2 += error * error;
    }
    parameters.rmse = std::sqrt(error2 / (s.time_s.size() - 1));
    return parameters;
}

Result fit(const p4::io::ModelACsvSamples& s) {
    Result best;
    // At fixed tau, the exact rollout is p + K*q + b*h.
    // Search tau on a log grid; solve K and b by two-variable least squares.
    for (int j = 0; j <= 250; ++j) {
        const double tau = 0.03 * std::pow(100.0, j / 250.0);
        double p = s.yaw_rad_s.front(), q = 0, h = 0;
        double qq = 0, qh = 0, hh = 0, qy = 0, hy = 0;
        for (std::size_t k = 1; k < s.time_s.size(); ++k) {
            const double a = std::exp(-(s.time_s[k] - s.time_s[k - 1]) / tau);
            p *= a;
            q = a * q + (1 - a) * s.steering_rad[k - 1];
            h = a * h + (1 - a);
            const double y = s.yaw_rad_s[k] - p;
            qq += q * q; qh += q * h; hh += h * h;
            qy += q * y; hy += h * y;
        }
        const double determinant = qq * hh - qh * qh;
        if (!(determinant > 1e-12 * qq * hh)) continue;
        const double gain = (qy * hh - hy * qh) / determinant;
        const double bias = (hy * qq - qy * qh) / determinant;
        const auto candidate = evaluate(s, {gain, tau, bias, 0});
        if (candidate.rmse < best.rmse) best = candidate;
    }
    if (!std::isfinite(best.rmse))
        throw std::runtime_error("Insufficient steering variation to identify K and b");
    return best;
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 4 && argc != 7) {
        std::cerr << "Usage: model_a_identify_csv train.csv start_s end_s "
                     "[validation.csv start_s end_s]\n";
        return 1;
    }
    try {
        p4::io::IONIQ5CsvReader reader;
        const auto train = reader.readModelA(argv[1], std::stod(argv[2]), std::stod(argv[3]));
        const auto fitted = fit(train);
        std::cout << std::setprecision(8) << "Train samples: " << train.time_s.size()
                  << "\nK: " << fitted.gain << " 1/s"
                  << "\ntau: " << fitted.tau << " s"
                  << "\nb: " << fitted.bias << " rad/s"
                  << "\nTrain RMSE: " << fitted.rmse << " rad/s\n";
        if (argc == 7) {
            const std::string train_file = argv[1], validation_file = argv[4];
            const double train_start = std::stod(argv[2]), train_end = std::stod(argv[3]);
            const double valid_start = std::stod(argv[5]), valid_end = std::stod(argv[6]);
            if (train_file == validation_file && valid_start < train_end && valid_end > train_start)
                throw std::invalid_argument("Training and validation intervals overlap");
            const auto validation = reader.readModelA(validation_file, valid_start, valid_end);
            const auto scored = evaluate(validation, fitted);
            std::cout << "Validation samples: " << validation.time_s.size()
                      << "\nValidation RMSE: " << scored.rmse << " rad/s\n";
        }
        if (fitted.tau < 0.0301 || fitted.tau > 2.999)
            std::cout << "Warning: tau reached search boundary\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
