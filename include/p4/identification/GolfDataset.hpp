#ifndef P4_IDENTIFICATION_GOLF_DATASET_HPP_
#define P4_IDENTIFICATION_GOLF_DATASET_HPP_

#include "p4/io/IONIQ5CsvReader.hpp"

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace p4::identification {

using GolfFile = std::vector<io::ModelACsvSamples>;

struct GolfFold {
    std::vector<GolfFile> training;
    std::vector<GolfFile> validation;
    std::vector<std::size_t> held_indices;
    std::vector<std::string> held_names;
};

class GolfDataset {
public:
    static constexpr std::size_t fold_count = 5;

    static GolfDataset load(const std::filesystem::path& root);
    GolfFold fold(std::size_t zero_based_index) const;
    const std::vector<std::filesystem::path>& paths() const noexcept { return paths_; }
    const std::vector<GolfFile>& files() const noexcept { return files_; }
    const std::vector<std::string>& duplicates() const noexcept { return duplicates_; }

private:
    std::vector<std::filesystem::path> paths_;
    std::vector<GolfFile> files_;
    std::vector<std::string> duplicates_;
};

} // namespace p4::identification
#endif
