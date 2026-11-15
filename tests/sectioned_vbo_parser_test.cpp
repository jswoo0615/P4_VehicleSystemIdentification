
#include "SectionedVBOParser.hpp"

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
using p4::io::parsers::SectionedVBOParser;

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
        const std::string actual_message = e.what();

        require(
            actual_message.find(expected_message)
                != std::string::npos,
            "Unexpected exception.\nExpected: " +
                expected_message +
                "\nActual: " + actual_message
        );

        return;
    }

    throw std::runtime_error(
        "Expected exception was not thrown: " +
        expected_message
    );
}

void runTest(
    const std::string& name,
    void (*test)()
) {
    test();
    std::cout << "[PASS] " << name << '\n';
}

VBOData parseText(const std::string& text) {
    std::istringstream input(text);
    SectionedVBOParser parser;
    return parser.parse(input);
}

// ------------------------------------------------------------
// SP1: Real VBO
// ------------------------------------------------------------

void testRealVBO(const std::string& filename) {
    std::ifstream file(filename);

    require(
        file.is_open(),
        "Cannot open VBO file: " + filename
    );

    SectionedVBOParser parser;
    const auto data = parser.parse(file);

    require(
        data.format == VBOFormat::Sectioned,
        "Incorrect VBO format"
    );

    require(
        data.columnCount() == 34,
        "Column count mismatch"
    );

    require(
        data.sampleCount() > 1,
        "Insufficient samples"
    );

    require(
        data.original_columns.size() ==
            data.columns.size(),
        "Column metadata mismatch"
    );

    require(
        near(data.timeAt(0), 52304.170),
        "First timestamp mismatch"
    );

    require(
        near(data.value(0, "steer_ang2"), -8.46314),
        "Steering mismatch"
    );

    require(
        near(data.value(0, "YawRate"), 11.36),
        "YawRate mismatch"
    );

    require(
        near(data.value(0, "Yaw_Rate"), 17.3981),
        "Yaw_Rate mismatch"
    );
}

// ------------------------------------------------------------
// SP2: Source line tracking
// ------------------------------------------------------------

void testSourceLine(const std::string& filename) {
    std::ifstream source(filename);

    require(
        source.is_open(),
        "Cannot open source file"
    );

    std::size_t expected_line = 0;
    std::size_t line_number = 0;

    bool data_section = false;

    std::string line;

    while (std::getline(source, line)) {
        ++line_number;

        if (line == "[data]" || line == "[data]\r") {
            data_section = true;
            continue;
        }

        if (data_section &&
            line.find_first_not_of(" \t\r\n")
                != std::string::npos) {
            expected_line = line_number;
            break;
        }
    }

    require(
        expected_line != 0,
        "Cannot find first data row"
    );

    std::ifstream file(filename);
    SectionedVBOParser parser;

    const auto data = parser.parse(file);

    const auto actual_line =
        data.samples.at(0).source_line;

    require(
        actual_line == expected_line,
        "Source line mismatch. Expected: " +
            std::to_string(expected_line) +
            ", actual: " +
            std::to_string(actual_line)
    );
}

// ------------------------------------------------------------
// SP3: Duplicate columns
// ------------------------------------------------------------

void testDuplicateColumns() {
    const auto data = parseText(
        "[column names]\n"
        "sats time time YawRate\n"
        "[data]\n"
        "049 143144,170 0,00 11,36\n"
    );

    require(
        data.columnCount() == 4,
        "Unexpected column count"
    );

    require(
        data.hasColumn("time"),
        "Missing time"
    );

    require(
        data.hasColumn("time__2"),
        "Missing time__2"
    );

    const auto indices =
        data.originalColumnIndices("time");

    require(
        indices == std::vector<std::size_t>{1, 2},
        "Original column mapping incorrect"
    );

    require(
        near(data.value(0, "time__2"), 0.0),
        "Duplicate time value incorrect"
    );
}

// ------------------------------------------------------------
// SP4: Column count mismatch
// ------------------------------------------------------------

void testColumnCountMismatch() {
    requireThrows(
        [] {
            parseText(
                "[column names]\n"
                "sats time YawRate\n"
                "[data]\n"
                "049 143144,170\n"
            );
        },
        "Column count mismatch"
    );
}

// ------------------------------------------------------------
// SP5: Invalid number
// ------------------------------------------------------------

void testInvalidNumber() {
    requireThrows(
        [] {
            parseText(
                "[column names]\n"
                "sats time YawRate\n"
                "[data]\n"
                "049 143144,170 invalid\n"
            );
        },
        "Invalid VBO number"
    );
}

// ------------------------------------------------------------
// SP6: Invalid timestamp
// ------------------------------------------------------------

void testInvalidTimestamp() {
    requireThrows(
        [] {
            parseText(
                "[column names]\n"
                "sats time YawRate\n"
                "[data]\n"
                "049 146199,000 11,36\n"
            );
        },
        "Invalid HHMMSS timestamp"
    );
}

// ------------------------------------------------------------
// SP7: Non-increasing time
// ------------------------------------------------------------

void testNonIncreasingTime() {
    requireThrows(
        [] {
            parseText(
                "[column names]\n"
                "sats time YawRate\n"
                "[data]\n"
                "049 143144,180 11,36\n"
                "049 143144,170 11,36\n"
            );
        },
        "Non-increasing timestamp"
    );

    requireThrows(
        [] {
            parseText(
                "[column names]\n"
                "sats time YawRate\n"
                "[data]\n"
                "049 143144,170 11,36\n"
                "049 143144,170 11,36\n"
            );
        },
        "Non-increasing timestamp"
    );
}

// ------------------------------------------------------------
// SP8: Missing sections
// ------------------------------------------------------------

void testMissingSections() {
    requireThrows(
        [] {
            parseText(
                "[column names]\n"
                "sats time YawRate\n"
            );
        },
        "Missing [data] section"
    );

    requireThrows(
        [] {
            parseText(
                "[data]\n"
                "049 143144,170 11,36\n"
            );
        },
        "[data] without column names"
    );
}

// ------------------------------------------------------------
// SP9: Empty data
// ------------------------------------------------------------

void testEmptyData() {
    requireThrows(
        [] {
            parseText(
                "[column names]\n"
                "sats time YawRate\n"
                "[data]\n"
            );
        },
        "no data samples"
    );
}

} // anonymous namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr
            << "Usage: sectioned_vbo_parser_test "
               "<vblog_001.vbo>\n";
        return 1;
    }

    try {
        testRealVBO(argv[1]);
        std::cout << "[PASS] SP1 Real VBO\n";

        testSourceLine(argv[1]);
        std::cout << "[PASS] SP2 Source Line\n";

        runTest("SP3 Duplicate Columns",
                testDuplicateColumns);

        runTest("SP4 Column Count",
                testColumnCountMismatch);

        runTest("SP5 Invalid Number",
                testInvalidNumber);

        runTest("SP6 Invalid Timestamp",
                testInvalidTimestamp);

        runTest("SP7 Time Continuity",
                testNonIncreasingTime);

        runTest("SP8 Missing Sections",
                testMissingSections);

        runTest("SP9 Empty Data",
                testEmptyData);

        std::cout
            << "\nALL SECTIONED VBO TESTS PASSED\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "[FAIL] "
                  << e.what() << '\n';
        return 1;
    }
}
