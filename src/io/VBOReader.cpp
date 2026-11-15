
#include "p4/io/VBOReader.hpp"

#include "SectionedVBOParser.hpp"
#include "TabularVBOParser.hpp"
#include "VBOParsingUtils.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace p4::io {

namespace {

// ============================================================
// 1. Whitespace Normalization
// ============================================================

// Collapse spaces, tabs and other whitespace into
// a single space. Leading/trailing whitespace removed.
//
// Example:
// "[column      names]" -> "[column names]"
// "File    created on"  -> "File created on"
std::string normalizeWhitespace(
    const std::string& line
) {
    std::istringstream iss(line);

    std::string token;
    std::string result;

    while (iss >> token) {
        if (!result.empty()) {
            result += ' ';
        }

        result += token;
    }

    return result;
}

// ============================================================
// 2. Section Header Detection
// ============================================================

bool isSectionHeader(
    const std::string& normalized
) {
    return normalized.size() >= 2 &&
           normalized.front() == '[' &&
           normalized.back() == ']';
}

// ============================================================
// 3. VBO Format Detection
// ============================================================

// Detect from section markers rather than assuming
// the first line has a fixed spelling.
//
// Sectioned:
//   File created on ...
//   [header]
//   [column names]
//   [data]
//
// Tabular:
//   sats time time lat long ...
//   data ...
VBOFormat detectFormat(std::istream& input) {
    std::string line;

    bool foundSection = false;
    bool foundDataSection = false;
    bool foundColumnNames = false;
    bool foundTabularHeader = false;

    while (std::getline(input, line)) {
        const std::string normalized =
            normalizeWhitespace(detail::trim(line));

        if (normalized.empty()) {
            continue;
        }

        if (isSectionHeader(normalized)) {
            foundSection = true;

            if (normalized == "[column names]") {
                foundColumnNames = true;
            }

            if (normalized == "[data]") {
                foundDataSection = true;
            }

            continue;
        }

        // Recognize the tabular header only when
        // no section markers have appeared.
        if (!foundSection &&
            normalized.rfind("sats ", 0) == 0) {
            foundTabularHeader = true;
        }
    }

    // Any actual section marker indicates a
    // sectioned-format candidate. The parser
    // will validate the required sections.
    if (foundSection) {
        // Explicitly retain the detected markers
        // for future format diagnostics.
        (void)foundDataSection;
        (void)foundColumnNames;

        return VBOFormat::Sectioned;
    }

    if (foundTabularHeader) {
        return VBOFormat::Tabular;
    }

    throw std::runtime_error(
        "Cannot detect VBO format: "
        "no recognized section or tabular header"
    );
}

// ============================================================
// 4. Normalize Sectioned VBO Stream
// ============================================================

// Only normalize section header lines.
//
// Do NOT normalize data rows, column definitions,
// comments, or measured signal values.
//
// This keeps original measurement data intact while
// allowing whitespace variations in section names.
std::string normalizeSectionedContent(
    std::istream& input
) {
    std::ostringstream output;

    std::string line;

    while (std::getline(input, line)) {
        // Windows CRLF support.
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        const std::string normalized =
            normalizeWhitespace(detail::trim(line));

        if (isSectionHeader(normalized)) {
            output << normalized << '\n';
        }
        else {
            output << line << '\n';
        }
    }

    return output.str();
}

} // anonymous namespace

// ============================================================
// 5. VBOReader::read()
// ============================================================

VBOData VBOReader::read(
    const std::string& filename
) const {
    // --------------------------------------------------------
    // 5.1 Open file
    // --------------------------------------------------------

    std::ifstream file(
        filename,
        std::ios::in | std::ios::binary
    );

    if (!file.is_open()) {
        throw std::runtime_error(
            "Cannot open VBO file: " + filename
        );
    }

    // --------------------------------------------------------
    // 5.2 Detect format
    // --------------------------------------------------------

    const VBOFormat format = detectFormat(file);

    // Rewind after format detection.
    file.clear();
    file.seekg(0, std::ios::beg);

    if (!file) {
        throw std::runtime_error(
            "Cannot rewind VBO file: " + filename
        );
    }

    VBOData data;

    // --------------------------------------------------------
    // 5.3 Parse by format
    // --------------------------------------------------------

    switch (format) {

        case VBOFormat::Sectioned: {

            // Normalize section headers in memory.
            // Original VBO file remains unchanged.
            const std::string normalizedContent =
                normalizeSectionedContent(file);

            std::istringstream normalizedStream(
                normalizedContent
            );

            parsers::SectionedVBOParser parser;

            data = parser.parse(normalizedStream);

            break;
        }

        case VBOFormat::Tabular: {

            parsers::TabularVBOParser parser;

            data = parser.parse(file);

            break;
        }

        default: {
            throw std::runtime_error(
                "Unsupported VBO format: " + filename
            );
        }
    }

    // --------------------------------------------------------
    // 5.4 Source metadata
    // --------------------------------------------------------

    data.source_file = filename;

    return data;
}

} // namespace p4::io
