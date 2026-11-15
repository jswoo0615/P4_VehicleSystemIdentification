#include "p4/io/VBOData.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace p4::io {
    // ============================================================
    // Column Management
    // ============================================================
    std::size_t VBOData::columnIndex(const std::string& name) const {
        const auto it = std::find(columns.begin(), columns.end(), name);
        if (it == columns.end()) {
            throw std::out_of_range("VBO column not found" + name);
        }

        return static_cast<std::size_t>(std::distance(columns.begin(), it));
    }

    bool VBOData::hasColumn(const std::string& name) const noexcept {
        return std::find(columns.begin(), columns.end(), name) != columns.end();
    }

    std::vector<std::size_t> VBOData::originalColumnIndices(const std::string& name) const {
        std::vector<std::size_t> indices;
        for (std::size_t i = 0; i < original_columns.size(); ++i) {
            if (original_columns[i] == name) {
                indices.push_back(i);
            }
        }
        return indices;
    }

    // ============================================================
    // Data Access
    // ============================================================
    double VBOData::value(std::size_t sample_index, const std::string& channel_name) const {
        const auto column_index = columnIndex(channel_name);
        const auto& sample = samples.at(sample_index);
        return sample.values.at(column_index);
    }

    double VBOData::timeAt(std::size_t sample_index) const {
        return samples.at(sample_index).time;
    }

    // ============================================================
    // Metadata
    // ============================================================
    double VBOData::duration() const {
        if (samples.size() < 2) {
            return 0.0;
        }
        return samples.back().time - samples.front().time;
    }
} // namespace p4::io