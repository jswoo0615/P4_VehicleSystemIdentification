#ifndef P4_IDENTIFICATION_MANIFEST_EVENT_HPP_
#define P4_IDENTIFICATION_MANIFEST_EVENT_HPP_

#include <cstddef>
#include <string>

namespace p4::identification {

// One measured maneuver; group_id is the split boundary.
struct ManifestEvent {
    std::string file;
    std::string group_id;
    std::string maneuver;
    std::size_t event_id = 0;
    double start_s = 0.0;
    double end_s = 0.0;
};

} // namespace p4::identification

#endif
