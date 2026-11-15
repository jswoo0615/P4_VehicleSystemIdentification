
#include "p4/io/VBOReader.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(double actual, double expected) {
    return std::abs(actual - expected) < 1e-5;
}

} // anonymous namespace

int main(int argc, char** argv) {
    try {
        require(argc == 3,
                "Usage: vbo_reader_integration_test "
                "<vblog_001> <vblog_002>");

        p4::io::VBOReader reader;

        const auto sectioned = reader.read(argv[1]);
        const auto tabular  = reader.read(argv[2]);

        // --------------------------------------------
        // Sectioned file
        // --------------------------------------------
        require(
            sectioned.format == p4::io::VBOFormat::Sectioned,
            "Sectioned format detection failed"
        );

        require(sectioned.columnCount() == 34,
                "Sectioned column count");

        require(sectioned.sampleCount() > 1,
                "Sectioned sample count");

        require(near(sectioned.timeAt(0), 52304.170),
                "Sectioned timestamp");

        require(near(sectioned.value(0, "steer_ang2"),
                     -8.46314),
                "Sectioned steering");

        require(near(sectioned.value(0, "YawRate"), 11.36),
                "Sectioned yaw");

        std::cout << "[PASS] VBO 001 Sectioned\n";

        // --------------------------------------------
        // Tabular file
        // --------------------------------------------
        require(
            tabular.format == p4::io::VBOFormat::Tabular,
            "Tabular format detection failed"
        );

        require(tabular.columnCount() == 35,
                "Tabular column count");

        require(tabular.sampleCount() == 2395,
                "Tabular sample count");

        require(near(tabular.timeAt(0), 52400.74),
                "Tabular timestamp");

        require(near(tabular.duration(), 23.94),
                "Tabular duration");

        require(near(tabular.value(0, "steer_ang2"), -2.34),
                "Tabular steering");

        require(near(tabular.value(0, "YawRate"), 3.08),
                "Tabular yaw");

        require(
            tabular.hasColumn("time__2") &&
            near(tabular.value(1, "time__2"), 0.01),
            "Duplicate time mapping"
        );

        std::cout << "[PASS] VBO 002 Tabular\n";

        std::cout
            << "\nALL INTEGRATION TESTS PASSED\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "[FAIL] " << e.what() << '\n';
        return 1;
    }
}
