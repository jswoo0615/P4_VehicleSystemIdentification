#ifndef TABULAR_VBO_PARSER_HPP_
#define TABULAR_VBO_PARSER_HPP_

#include "p4/io/VBOData.hpp"
#include <istream>

namespace p4::io::parsers {
    class TabularVBOParser {
        public:
            VBOData parse(std::istream& input) const;
    };
} // namespace p4::io::parsers

#endif // TABULAR_VBO_PARSER_HPP_