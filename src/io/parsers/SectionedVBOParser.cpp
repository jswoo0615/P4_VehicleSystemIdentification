
#include "SectionedVBOParser.hpp"
#include "VBOParsingUtils.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace p4::io::parsers {

namespace {

namespace utils = p4::io::detail;

enum class Section {
    Other,
    ColumnNames,
    Data
};

std::runtime_error parseError(
    std::size_t line_number,
    const std::string& message
) {
    return std::runtime_error(
        "Sectioned VBO line " +
        std::to_string(line_number) +
        ": " + message
    );
}

} // anonymous namespace

VBOData SectionedVBOParser::parse(
    std::istream& input
) const {
    VBOData data;
    data.format = VBOFormat::Sectioned;

    Section section = Section::Other;

    bool found_columns = false;
    bool found_data = false;

    std::string line;
    std::size_t line_number = 0;

    while (std::getline(input, line)) {
        ++line_number;
        line = utils::trim(line);

        if (line.empty()) {
            continue;
        }

        // ------------------------------------------
        // 1. Section detection
        // ------------------------------------------
        if (line.front() == '[' &&
            line.back() == ']') {

            if (line == "[column names]") {
                if (found_columns || found_data) {
                    throw parseError(
                        line_number,
                        "Duplicate [column names] section"
                    );
                }

                section = Section::ColumnNames;
                found_columns = true;
            }
            else if (line == "[data]") {
                if (!found_columns ||
                    data.original_columns.empty()) {
                    throw parseError(
                        line_number,
                        "[data] without column names"
                    );
                }

                if (found_data) {
                    throw parseError(
                        line_number,
                        "Duplicate [data] section"
                    );
                }

                // Finalize column names
                data.columns = utils::makeUniqueColumns(
                    data.original_columns
                );

                section = Section::Data;
                found_data = true;
            }
            else {
                section = Section::Other;
            }

            continue;
        }

        // ------------------------------------------
        // 2. Read column names
        // ------------------------------------------
        if (section == Section::ColumnNames) {
            const auto names = utils::split(line);

            data.original_columns.insert(
                data.original_columns.end(),
                names.begin(),
                names.end()
            );

            continue;
        }

        if (section != Section::Data) {
            continue;
        }

        // ------------------------------------------
        // 3. Read data row
        // ------------------------------------------
        const auto tokens = utils::split(line);

        if (tokens.size() != data.columnCount()) {
            throw parseError(
                line_number,
                "Column count mismatch: expected " +
                std::to_string(data.columnCount()) +
                ", got " +
                std::to_string(tokens.size())
            );
        }

        VBORawSample sample;
        sample.source_line = line_number;
        sample.values.reserve(tokens.size());

        try {
            for (const auto& token : tokens) {
                sample.values.push_back(
                    utils::parseNumber(token)
                );
            }

            // HHMMSS is the first "time" channel.
            const std::size_t time_index =
                data.columnIndex("time");

            sample.time = utils::parseTime(
                tokens.at(time_index)
            );
        }
        catch (const std::exception& e) {
            throw parseError(
                line_number,
                e.what()
            );
        }

        // ------------------------------------------
        // 4. Time continuity validation
        // ------------------------------------------
        if (!data.samples.empty()) {
            const double previous_time =
                data.samples.back().time;

            if (sample.time <= previous_time) {
                throw parseError(
                    line_number,
                    "Non-increasing timestamp"
                );
            }
        }

        data.samples.push_back(std::move(sample));
    }

    // ----------------------------------------------
    // 5. Final validation
    // ----------------------------------------------
    if (!input.eof() && input.fail()) {
        throw std::runtime_error(
            "Sectioned VBO input stream error"
        );
    }

    if (!found_columns) {
        throw std::runtime_error(
            "Missing [column names] section"
        );
    }

    if (!found_data) {
        throw std::runtime_error(
            "Missing [data] section"
        );
    }

    if (data.samples.empty()) {
        throw std::runtime_error(
            "Sectioned VBO contains no data samples"
        );
    }

    return data;
}

} // namespace p4::io::parsers
