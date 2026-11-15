
#include "TabularVBOParser.hpp"

#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using p4::io::VBOData;
using p4::io::VBOFormat;
using p4::io::parsers::TabularVBOParser;

constexpr double kTolerance = 1e-5;

void require(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(double actual, double expected) {
    return std::abs(actual - expected) <= kTolerance;
}

template <typename Func>
void requireThrows(
    Func&& func,
    const std::string& expected_message
) {
    try {
        func();
    }
    catch (const std::exception& e) {
        const std::string actual = e.what();

        require(
            actual.find(expected_message) !=
                std::string::npos,
            "Unexpected exception.\nExpected: " +
                expected_message +
                "\nActual: " + actual
        );
        return;
    }

    throw std::runtime_error(
        "Expected exception was not thrown: " +
        expected_message
    );
}

VBOData parseText(const std::string& text) {
    std::istringstream input(text);
    TabularVBOParser parser;
    return parser.parse(input);
}

// TP1: Real vblog_002

void testRealVBO(const std::string& filename) {
    std::ifstream file(filename);

    require(
        file.is_open(),
        "Cannot open file: " + filename
    );

    TabularVBOParser parser;
    const VBOData data = parser.parse(file);

    std::cout << "\n=== Tabular VBO Debug ===\n";
    std::cout << "File    : " << filename << '\n';
    std::cout << "Columns : " << data.columnCount() << '\n';
    std::cout << "Samples : " << data.sampleCount() << '\n';

    require(
        data.format == VBOFormat::Tabular,
        "Incorrect format"
    );

    require(
        data.columnCount() == 35,
        "Expected 35 columns"
    );

    require(
        data.sampleCount() == 2395,
        "Sample count mismatch. Expected 2395, actual " +
            std::to_string(data.sampleCount())
    );

    require(
        data.columns.size() ==
            data.original_columns.size(),
        "Column metadata mismatch"
    );

    require(
        near(data.timeAt(0), 52400.74),
        "First timestamp mismatch"
    );

    require(
        near(data.duration(), 23.94),
        "Duration mismatch"
    );

    require(
        near(data.value(0, "steer_ang2"), -2.34),
        "Steering mismatch"
    );

    require(
        near(data.value(0, "YawRate"), 3.08),
        "YawRate mismatch"
    );

    require(
        near(data.value(0, "Yaw_Rate"), 3.90),
        "Yaw_Rate mismatch"
    );

    require(
        near(data.value(0, "time__2"), 0.0) &&
        near(data.value(1, "time__2"), 0.01),
        "Second time column mismatch"
    );

    std::cout << "[PASS] Real VBO validation\n";
}


// TP4: Column count mismatch
void testColumnCountMismatch() {
    requireThrows(
        [] {
            parseText(
                "sats time YawRate\n"
                "048 143320,740\n"
            );
        },
        "Column count mismatch"
    );
}

// TP5: Invalid number
void testInvalidNumber() {
    requireThrows(
        [] {
            parseText(
                "sats time YawRate\n"
                "048 143320,740 invalid\n"
            );
        },
        "Invalid VBO number"
    );
}

// TP6: Invalid HHMMSS
void testInvalidTime() {
    requireThrows(
        [] {
            parseText(
                "sats time YawRate\n"
                "048 146199,000 3,08\n"
            );
        },
        "Invalid HHMMSS timestamp"
    );
}

// TP7: Duplicate and reversed timestamps
void testNonIncreasingTime() {
    requireThrows(
        [] {
            parseText(
                "sats time YawRate\n"
                "048 143320,750 3,08\n"
                "048 143320,740 3,15\n"
            );
        },
        "Non-increasing timestamp"
    );

    requireThrows(
        [] {
            parseText(
                "sats time YawRate\n"
                "048 143320,740 3,08\n"
                "048 143320,740 3,15\n"
            );
        },
        "Non-increasing timestamp"
    );
}

// TP8: Missing time column
void testMissingTime() {
    requireThrows(
        [] {
            parseText(
                "sats steering YawRate\n"
                "048 -2,34 3,08\n"
            );
        },
        "Missing time column"
    );
}

// TP9: Header only
void testEmptyData() {
    requireThrows(
        [] {
            parseText(
                "sats time YawRate\n"
            );
        },
        "no data samples"
    );
}

// TP10: Sectioned VBO passed to wrong parser
void testWrongFormat() {
    requireThrows(
        [] {
            parseText(
                "[column names]\n"
                "sats time YawRate\n"
                "[data]\n"
                "048 143320,740 3,08\n"
            );
        },
        "Not a Tabular VBO file"
    );
}

void testSourceLine() {
    const auto data = parseText(
        "\n"
        "sats time time YawRate\n"
        "048 143320,740 0,00 3,08\n"
        "048 143320,750 0,01 3,15\n"
    );

    require(
        data.samples.at(0).source_line == 3 &&
        data.samples.at(1).source_line == 4,
        "Incorrect source line"
    );
}

void testDuplicateColumns() {
    const auto data = parseText(
        "sats time time YawRate\n"
        "048 143320,740 0,00 3,08\n"
    );

    require(
        data.hasColumn("time") &&
        data.hasColumn("time__2"),
        "Duplicate time not handled"
    );

    require(
        data.originalColumnIndices("time") ==
            std::vector<std::size_t>{1, 2},
        "Original names not preserved"
    );
}

} // anonymous namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr
            << "Usage: tabular_vbo_parser_test "
               "<vblog_002.vbo>\n";
        return 1;
    }

    try {
        testRealVBO(argv[1]);
        std::cout << "[PASS] TP1 Real VBO\n";

        testSourceLine();
        std::cout << "[PASS] TP2 Source Line\n";

        testDuplicateColumns();
        std::cout << "[PASS] TP3 Duplicate Columns\n";

        testColumnCountMismatch();
        std::cout << "[PASS] TP4 Column Count\n";

        testInvalidNumber();
        std::cout << "[PASS] TP5 Invalid Number\n";

        testInvalidTime();
        std::cout << "[PASS] TP6 Invalid Time\n";

        testNonIncreasingTime();
        std::cout << "[PASS] TP7 Time Continuity\n";

        testMissingTime();
        std::cout << "[PASS] TP8 Missing Time\n";

        testEmptyData();
        std::cout << "[PASS] TP9 Empty Data\n";

        testWrongFormat();
        std::cout << "[PASS] TP10 Wrong Format\n";

        std::cout
            << "\nALL TABULAR VBO TESTS PASSED\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr
            << "[FAIL] " << e.what() << '\n';
        return 1;
    }
}
