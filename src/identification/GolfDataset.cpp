#include "p4/identification/GolfDataset.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <unordered_map>

namespace p4::identification {
namespace {
std::uint64_t contentHash(const std::filesystem::path& path) {
    std::ifstream input(path,std::ios::binary);
    if(!input) throw std::runtime_error("Cannot read CSV: "+path.string());
    std::uint64_t hash=14695981039346656037ULL;
    char buffer[65536];
    while(input) {
        input.read(buffer,sizeof(buffer));
        for(std::streamsize i=0;i<input.gcount();++i) {
            hash^=static_cast<unsigned char>(buffer[i]);
            hash*=1099511628211ULL;
        }
    }
    return hash;
}
}

GolfDataset GolfDataset::load(const std::filesystem::path& root) {
    GolfDataset result;
    std::vector<std::filesystem::path> candidates;
    for(const auto& entry:std::filesystem::recursive_directory_iterator(root))
        if(entry.is_regular_file()&&entry.path().extension()==".csv")
            candidates.push_back(entry.path());
    std::sort(candidates.begin(),candidates.end());
    std::unordered_map<std::uint64_t,std::string> seen;
    for(const auto& path:candidates) {
        const auto hash=contentHash(path);
        const auto inserted=seen.emplace(hash,path.filename().string());
        if(inserted.second) result.paths_.push_back(path);
        else result.duplicates_.push_back(path.filename().string()+" = "+inserted.first->second);
    }
    if(result.paths_.size()<fold_count) throw std::runtime_error("Fewer than five unique files");
    io::IONIQ5CsvReader reader(io::CsvChannelMap::StanfordGolf);
    io::ModelACsvOptions options;
    options.minimum_speed_m_s=5.0;
    options.sample_period_s=0.02;
    for(const auto& path:result.paths_) {
        auto segments=reader.readModelASegments(path.string(),options);
        if(segments.empty()) throw std::runtime_error("No usable segments: "+path.string());
        result.files_.push_back(std::move(segments));
    }
    return result;
}

GolfFold GolfDataset::fold(std::size_t index) const {
    if(index>=fold_count) throw std::out_of_range("Golf fold index");
    GolfFold result;
    for(std::size_t i=0;i<files_.size();++i) {
        if(i%fold_count==index) {
            result.validation.push_back(files_[i]);
            result.held_indices.push_back(i);
            result.held_names.push_back(paths_[i].filename().string());
        } else result.training.push_back(files_[i]);
    }
    return result;
}
} // namespace p4::identification
