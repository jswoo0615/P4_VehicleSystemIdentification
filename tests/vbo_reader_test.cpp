
#include "p4/io/VBOReader.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

bool near(double a, double b) {
    return std::abs(a - b) < 1e-6;
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            std::cerr
                << "Usage: vbo_reader_test <file.vbo>\n";
            return 1;
        }

        p4::io::VBOReader reader;
        const auto data = reader.read(argv[1]);

        std::cout << std::fixed
                  << std::setprecision(6);

        std::cout
            << "=== VBO Reader Test ===\n"
            << "File    : " << data.source_file << '\n'
            << "Columns : " << data.columns.size() << '\n'
            << "Samples : " << data.sampleCount() << '\n';

        std::cout << "\n=== Column Mapping ===\n";

        for (std::size_t i = 0;
             i < data.columns.size(); ++i) {
            std::cout
                << i << " : " << data.columns[i]
                << '\n';
        }

        const double time =
            data.samples.at(0).time;

        const double steering =
            data.value(0, "steer_ang2");

        const double yaw =
            data.value(0, "YawRate");

        const double yaw_other =
            data.value(0, "Yaw_Rate");

        std::cout << "\n=== First Sample ===\n"
                  << "Time       : " << time << " s\n"
                  << "Steering   : " << steering << '\n'
                  << "YawRate    : " << yaw << '\n'
                  << "Yaw_Rate   : " << yaw_other << '\n';

        const bool pass =
            near(time, 52304.170) &&
            near(steering, -8.46314) &&
            near(yaw, 11.36) &&
            near(yaw_other, 17.3981);

        std::cout
            << "\nFirst sample validation: "
            << (pass ? "PASS" : "FAIL") << '\n';

        return pass ? 0 : 1;
    }
    catch (const std::exception& e) {
        std::cerr
            << "[ERROR] " << e.what() << '\n';
        return 1;
    }
}
