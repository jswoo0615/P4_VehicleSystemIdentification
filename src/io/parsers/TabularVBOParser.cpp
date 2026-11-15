
#include "TabularVBOParser.hpp"
#include "VBOParsingUtils.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace p4::io::parsers {

namespace {

namespace utils = p4::io::detail;

std::runtime_error parseError(
    std::size_t line,
    const std::string& message
) {
    return std::runtime_error(
        "Tabular VBO line " +
        std::to_string(line) + ": " + message
    );
}

bool isNumeric(const std::string& token) {
    try {
        (void)utils::parseNumber(token);
        return true;
    }
    catch (const std::exception&) {
        return false;
    }
}

} // anonymous namespace

VBOData TabularVBOParser::parse(
    std::istream& input
) const {
    VBOData data;
    data.format = VBOFormat::Tabular;

    std::string line;
    std::size_t line_number = 0;

    bool header_initialized = false;
    std::size_t time_index = 0;

    while (std::getline(input, line)) {
        ++line_number;

        line = utils::trim(line);

        if (line.empty()) {
            continue;
        }

        auto tokens = utils::split(line);

        if (tokens.empty()) {
            continue;
        }

        // ============================================
        // 1. Initialize column definitions ONCE
        // ============================================
        if (!header_initialized) {

            if (tokens.front().front() == '[' ||
                tokens.front() == "File") {
                throw parseError(
                    line_number,
                    "Not a Tabular VBO file"
                );
            }

            const bool is_data_row =
                isNumeric(tokens.front());

            if (is_data_row) {
                // Headerless: use fixed 35-channel schema.
                data.original_columns =
                    utils::defaultVBOColumns();
            }
            else {
                // Header present: use its actual names.
                data.original_columns = tokens;
            }

            data.columns = utils::makeUniqueColumns(
                data.original_columns
            );

            if (!data.hasColumn("time")) {
                throw parseError(
                    line_number,
                    "Missing time column"
                );
            }

            time_index = data.columnIndex("time");
            header_initialized = true;

            // The current line is the header, not data.
            if (!is_data_row) {
                continue;
            }

            // Otherwise process this first data row below.
        }

        // ============================================
        // 2. Validate column count
        // ============================================
        if (tokens.size() != data.columnCount()) {
            throw parseError(
                line_number,
                "Column count mismatch: expected " +
                std::to_string(data.columnCount()) +
                ", actual " +
                std::to_string(tokens.size())
            );
        }

        // ============================================
        // 3. Parse sample
        // ============================================
        VBORawSample sample;
        sample.source_line = line_number;
        sample.values.reserve(tokens.size());

        try {
            for (const auto& token : tokens) {
                sample.values.push_back(
                    utils::parseNumber(token)
                );
            }

            sample.time = utils::parseTime(
                tokens.at(time_index)
            );
        }
        catch (const std::exception& e) {
            throw parseError(line_number, e.what());
        }

        // ============================================
        // 4. Check timestamp continuity
        // ============================================
        if (!data.samples.empty()) {
            if (sample.time <=
                data.samples.back().time) {
                throw parseError(
                    line_number,
                    "Non-increasing timestamp"
                );
            }
        }

        // Every valid row must be appended.
        data.samples.push_back(std::move(sample));
    }

    // ================================================
    // 5. Final validation
    // ================================================
    if (!input.eof() && input.fail()) {
        throw std::runtime_error(
            "Tabular VBO stream read failure"
        );
    }

    if (!header_initialized) {
        throw std::runtime_error(
            "Missing Tabular VBO header"
        );
    }

    if (data.samples.empty()) {
        throw std::runtime_error(
            "Tabular VBO contains no data samples"
        );
    }

    return data;
}

} // namespace p4::io::parsers
