
#include "VBOParsingUtils.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace utils = p4::io::detail;

constexpr double tolerance = 1e-8;

bool near(double a, double b) {
    return std::abs(a - b) < tolerance;
}

void require(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename F>
bool throwsException(F&& function) {
    try {
        function();
        return false;
    }
    catch (const std::exception&) {
        return true;
    }
}

void testTrimAndSplit() {
    require(
        utils::trim("  YawRate \t") == "YawRate",
        "trim failed"
    );

    const auto tokens =
        utils::split("sats   time\tYawRate");

    require(
        tokens == std::vector<std::string>{
            "sats", "time", "YawRate"
        },
        "split failed"
    );

    std::cout << "[PASS] P1 Trim / Split\n";
}

void testNumber() {
    require(
        near(utils::parseNumber("-8,46314E+00"),
             -8.46314),
        "scientific notation failed"
    );

    require(
        near(utils::parseNumber("+1,13600E+01"),
             11.36),
        "yaw rate parsing failed"
    );

    require(
        near(utils::parseNumber("0,01"), 0.01),
        "decimal comma failed"
    );

    require(
        throwsException([] {
            utils::parseNumber("12,3abc");
        }),
        "partial number was accepted"
    );

    require(
        throwsException([] {
            utils::parseNumber("NaN");
        }),
        "NaN was accepted"
    );

    std::cout << "[PASS] P2 Number Parsing\n";
}

void testTime() {
    require(
        near(utils::parseTime("143144,170"),
             52304.170),
        "vblog_001 time failed"
    );

    require(
        near(utils::parseTime("143320,74"),
             52400.74),
        "vblog_002 time failed"
    );

    require(
        near(utils::parseTime("000000,000"), 0.0),
        "midnight time failed"
    );

    require(
        throwsException([] {
            utils::parseTime("146199,000");
        }),
        "invalid minute/second accepted"
    );

    std::cout << "[PASS] P3 Time Parsing\n";
}

void testUniqueColumns() {
    const std::vector<std::string> original = {
        "sats", "time", "time",
        "YawRate", "YawRate"
    };

    const auto result =
        utils::makeUniqueColumns(original);

    require(
        result == std::vector<std::string>{
            "sats", "time", "time__2",
            "YawRate", "YawRate__2"
        },
        "duplicate columns failed"
    );

    // Explicit original name must not be overwritten.
    const auto collision =
        utils::makeUniqueColumns({
            "time", "time", "time__2"
        });

    require(
        collision == std::vector<std::string>{
            "time", "time__3", "time__2"
        },
        "column suffix collision"
    );

    std::cout << "[PASS] P4 Unique Columns\n";
}

} // anonymous namespace

int main() {
    try {
        testTrimAndSplit();
        testNumber();
        testTime();
        testUniqueColumns();

        std::cout
            << "\nALL PARSING UTILS TESTS PASSED\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr
            << "[FAIL] " << e.what() << '\n';

        return 1;
    }
}
