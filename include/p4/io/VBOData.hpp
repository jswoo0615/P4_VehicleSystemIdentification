#ifndef VBODATA_HPP_
#define VBODATA_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace p4::io {
    // ============================================================
    // VBO File Format
    // ============================================================

    enum class VBOFormat {
        Unknown,
        Sectioned,
        Tabular
    };

    // ============================================================
    // VBO Raw Sample
    // ============================================================
    struct VBORawSample {
        // HHMMSS.sss를 초 단위로 변환한 시간
        // 예 : 143144,170 -> 52304.170
        double time = 0.0;

        // 원본 계측 채널값
        // columns와 동일한 순서로 저장
        std::vector<double> values;

        // 원본 파일에서 해당 샘플이 위치한 행 번호
        std::size_t source_line = 0;
    };

    // ============================================================
    // VBO Data Container
    // ============================================================
    struct VBOData {
        // 파일 경로
        std::string source_file;

        // 파일 형식
        VBOFormat format = VBOFormat::Unknown;

        // 고유하게 정리한 채널명
        // 예 : time, time__2
        std::vector<std::string> columns;

        // 파일에 기록된 실제 채널명
        // 예 : time, time
        std::vector<std::string> original_columns;

        // 계측 샘플
        std::vector<VBORawSample> samples;

        // ============================================================
        // Column Management
        // ============================================================

        // 채널명으로 인덱스 검색
        // 존재하지 않으면 std::out_of_range
        std::size_t columnIndex(const std::string& name) const;

        // 채널 존재 여부
        bool hasColumn(const std::string& name) const noexcept;

        // 원본 채널명으로 인덱스 목록 검색
        // 동일한 원본 채널명이 여러 개일 수 있음
        std::vector<std::size_t> originalColumnIndices(const std::string& name) const;

        // ============================================================
        // Data Access
        // ============================================================

        // 샘플 인덱스와 채널명으로 원본값 조회
        double value(std::size_t sample_index, const std::string& channel_name) const;

        // 샘플 시간 조회
        double timeAt(std::size_t sample_index) const;

        // ============================================================
        // Metadata
        // ============================================================
        std::size_t sampleCount() const noexcept {
            return samples.size();
        }
        std::size_t columnCount() const noexcept {
            return columns.size();
        }
        bool empty() const noexcept {
            return samples.empty();
        }
        // 전체 기록 시간 (초)
        double duration() const;
    };
} // namespace p4::io

#endif // VBODATA_HPP_
