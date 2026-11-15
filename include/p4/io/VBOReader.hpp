#ifndef VBO_READER_HPP_
#define VBO_READER_HPP_

#include "p4/io/VBOData.hpp"

#include <string>

namespace p4::io {

class VBOReader {
public:
    // 파일 형식을 자동으로 판단하여 데이터를 반환
    VBOData read(const std::string& filename) const;
};

} // namespace p4::io
#endif // VBO_READER_HPP_