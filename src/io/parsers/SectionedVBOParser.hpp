#ifndef SECTIONED_VBO_PARSER_HPP_
#define SECTIONED_VBO_PARSER_HPP_

#include "p4/io/VBOData.hpp"
#include <istream>

namespace p4::io::parsers {
    class SectionedVBOParser {
        public:
            VBOData parse(std::istream& input) const;
    };
} // namespace p4::io::parsers

#endif // SECTIONED_VBO_PARSER_HPP_