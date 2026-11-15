
#include "p4/io/VBOReader.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {

bool near(double a, double b) {
    return std::abs(a - b) < 1e-5;
}

}

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            std::cerr
                << "Usage: vbo_reader_002_test <file>\n";
            return 1;
        }

        p4::io::VBOReader reader;
        const auto data = reader.read(argv[1]);

        const double t0 = data.samples.at(0).time;
        const double t1 = data.samples.at(1).time;

        const double elapsed0 =
            data.value(0, "time__2");

        const double elapsed1 =
            data.value(1, "time__2");

        std::cout << std::fixed
                  << std::setprecision(6);

        std::cout << "Columns : "
                  << data.columns.size() << '\n';

        std::cout << "Samples : "
                  << data.sampleCount() << '\n';

        std::cout << "Time 0  : " << t0 << '\n';
        std::cout << "Time 1  : " << t1 << '\n';

        std::cout << "Elapsed0: " << elapsed0 << '\n';
        std::cout << "Elapsed1: " << elapsed1 << '\n';

        const bool pass =
            data.columns.size() == 35 &&
            data.sampleCount() == 2395 &&
            near(t0, 52400.74) &&
            near(t1 - t0, 0.01) &&
            near(elapsed0, 0.0) &&
            near(elapsed1, 0.01) &&
            data.hasColumn("steer_ang2") &&
            data.hasColumn("YawRate") &&
            data.hasColumn("Yaw_Rate");

        std::cout
            << (pass ? "[PASS]" : "[FAIL]")
            << " VBO 002 format\n";

        return pass ? 0 : 1;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] "
                  << e.what() << '\n';
        return 1;
    }
}
