
#include "p4/io/VBOData.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using p4::io::VBOData;
using p4::io::VBORawSample;
using p4::io::VBOFormat;

constexpr double tolerance = 1e-9;

bool near(double a, double b) {
    return std::abs(a - b) < tolerance;
}

bool report(
    const std::string& name,
    bool passed
) {
    std::cout
        << (passed ? "[PASS] " : "[FAIL] ")
        << name << '\n';

    return passed;
}

VBOData createTestData() {
    VBOData data;

    data.source_file = "synthetic.vbo";
    data.format = VBOFormat::Tabular;

    // 실제 vblog_002처럼 time이 두 번 등장
    data.original_columns = {
        "sats",
        "time",
        "time",
        "steer_ang2",
        "YawRate"
    };

    data.columns = {
        "sats",
        "time",
        "time__2",
        "steer_ang2",
        "YawRate"
    };

    VBORawSample sample0;
    sample0.time = 52400.74;
    sample0.source_line = 2;
    sample0.values = {
        48.0,
        143320.74,
        0.0,
        -2.34,
        3.08
    };

    VBORawSample sample1;
    sample1.time = 52400.75;
    sample1.source_line = 3;
    sample1.values = {
        48.0,
        143320.75,
        0.01,
        -2.20,
        3.15
    };

    data.samples.push_back(sample0);
    data.samples.push_back(sample1);

    return data;
}

bool testColumnIndex(const VBOData& data) {
    const bool pass =
        data.columnIndex("time") == 1 &&
        data.columnIndex("time__2") == 2 &&
        data.columnIndex("YawRate") == 4 &&
        data.hasColumn("steer_ang2") &&
        !data.hasColumn("not_existing");

    return report("D1 Column Index", pass);
}

bool testOriginalColumns(const VBOData& data) {
    const auto indices =
        data.originalColumnIndices("time");

    const bool pass =
        indices.size() == 2 &&
        indices[0] == 1 &&
        indices[1] == 2;

    return report("D2 Original Columns", pass);
}

bool testValues(const VBOData& data) {
    const bool pass =
        near(data.value(0, "steer_ang2"), -2.34) &&
        near(data.value(0, "YawRate"), 3.08) &&
        near(data.value(1, "YawRate"), 3.15) &&
        near(data.value(1, "time__2"), 0.01);

    return report("D3 Sample Values", pass);
}

bool testTime(const VBOData& data) {
    const bool pass =
        near(data.timeAt(0), 52400.74) &&
        near(data.timeAt(1), 52400.75) &&
        near(data.duration(), 0.01);

    return report("D4 Time and Duration", pass);
}

bool testMissingColumn(const VBOData& data) {
    try {
        (void)data.columnIndex("unknown");
    }
    catch (const std::out_of_range&) {
        return report("D5 Missing Column", true);
    }

    return report("D5 Missing Column", false);
}

bool testInvalidSample(const VBOData& data) {
    try {
        (void)data.value(100, "YawRate");
    }
    catch (const std::out_of_range&) {
        return report("D6 Invalid Sample", true);
    }

    return report("D6 Invalid Sample", false);
}

} // anonymous namespace

int main() {
    const VBOData data = createTestData();

    bool passed = true;

    passed &= testColumnIndex(data);
    passed &= testOriginalColumns(data);
    passed &= testValues(data);
    passed &= testTime(data);
    passed &= testMissingColumn(data);
    passed &= testInvalidSample(data);

    std::cout
        << (passed
            ? "\nALL VBO DATA TESTS PASSED\n"
            : "\nVBO DATA TEST FAILURE\n");

    return passed ? 0 : 1;
}
