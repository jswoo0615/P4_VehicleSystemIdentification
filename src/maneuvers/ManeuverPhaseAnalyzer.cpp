
#include "p4/maneuvers/ManeuverPhaseAnalyzer.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace p4::maneuvers {
namespace {

using Phase = ManeuverPhase;

struct SignalData {
    std::vector<double> t, steer, yaw, speed;
};

double medianDt(const std::vector<double>& t) {
    std::vector<double> dt;
    for (std::size_t i = 1; i < t.size(); ++i) {
        const double d = t[i] - t[i - 1];
        if (!(d > 0.0) || !std::isfinite(d))
            throw std::runtime_error("Invalid timestamp");
        dt.push_back(d);
    }
    if (dt.empty())
        throw std::runtime_error("Insufficient samples");

    std::sort(dt.begin(), dt.end());
    const auto n = dt.size();
    return n % 2 ? dt[n / 2]
                 : 0.5 * (dt[n / 2 - 1] + dt[n / 2]);
}

SignalData load(const p4::io::VBOData& data) {
    const auto n = data.sampleCount();
    if (n < 3)
        throw std::runtime_error("Insufficient samples");

    for (const char* col :
         {"steer_ang2", "YawRate", "velocity"}) {
        if (!data.hasColumn(col))
            throw std::runtime_error(
                std::string("Missing column: ") + col);
    }

    SignalData s;
    const double t0 = data.timeAt(0);

    for (std::size_t i = 0; i < n; ++i) {
        s.t.push_back(data.timeAt(i) - t0);
        s.steer.push_back(data.value(i, "steer_ang2"));
        s.yaw.push_back(data.value(i, "YawRate"));
        s.speed.push_back(data.value(i, "velocity"));

        if (!std::isfinite(s.t.back()) ||
            !std::isfinite(s.steer.back()) ||
            !std::isfinite(s.yaw.back()) ||
            !std::isfinite(s.speed.back()))
            throw std::runtime_error("Non-finite sample");
    }

    medianDt(s.t); // Also validates increasing timestamps.
    return s;
}

// Same edge-padded moving average used in the
// legacy Python major-excursion analysis.
std::vector<double> smooth(
    const std::vector<double>& x,
    const std::vector<double>& t,
    double windowSeconds
) {
    int window = std::max(
        3, static_cast<int>(
            std::nearbyint(windowSeconds / medianDt(t))
        )
    );
    if (window % 2 == 0) ++window;

    const int n = static_cast<int>(x.size());
    const int pad = window / 2;
    std::vector<double> result(x.size());

    for (int i = 0; i < n; ++i) {
        double sum = 0.0;
        for (int j = -pad; j <= pad; ++j) {
            const int k = std::clamp(i + j, 0, n - 1);
            sum += x[static_cast<std::size_t>(k)];
        }
        result[static_cast<std::size_t>(i)] = sum / window;
    }
    return result;
}

// numpy.gradient equivalent for a nonuniform time axis.
std::vector<double> gradient(
    const std::vector<double>& t,
    const std::vector<double>& y
) {
    const std::size_t n = t.size();
    if (n < 2)
        throw std::runtime_error("Gradient requires 2 samples");

    std::vector<double> g(n);
    g.front() = (y[1] - y[0]) / (t[1] - t[0]);

    for (std::size_t i = 1; i + 1 < n; ++i) {
        const double h0 = t[i] - t[i - 1];
        const double h1 = t[i + 1] - t[i];

        g[i] =
            -h1 / (h0 * (h0 + h1)) * y[i - 1]
            + (h1 - h0) / (h0 * h1) * y[i]
            + h0 / (h1 * (h0 + h1)) * y[i + 1];
    }

    g.back() =
        (y[n - 1] - y[n - 2]) / (t[n - 1] - t[n - 2]);

    return g;
}

double percentile80(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const double index = 0.8 * (values.size() - 1);
    const auto lo = static_cast<std::size_t>(index);
    const auto hi = std::min(lo + 1, values.size() - 1);
    const double f = index - lo;
    return values[lo] * (1.0 - f) + values[hi] * f;
}

double interp(const std::vector<double>& t,
              const std::vector<double>& y,
              double query) {
    if (query <= t.front()) return y.front();
    if (query >= t.back()) return y.back();

    const auto it = std::lower_bound(
        t.begin(), t.end(), query
    );
    const auto i = static_cast<std::size_t>(
        it - t.begin()
    );

    if (t[i] == query) return y[i];

    const double f =
        (query - t[i - 1]) / (t[i] - t[i - 1]);
    return y[i - 1] + f * (y[i] - y[i - 1]);
}

bool crossing(const std::vector<double>& t,
              const std::vector<double>& y,
              std::size_t a, std::size_t b,
              double threshold, double& result) {
    for (std::size_t i = a; i < b; ++i) {
        const double y0 = y[i] - threshold;
        const double y1 = y[i + 1] - threshold;

        if (y0 * y1 <= 0.0) {
            const double delta = y[i + 1] - y[i];
            const double f = std::abs(delta) < 1e-12
                ? 0.0 : (threshold - y[i]) / delta;
            result = t[i] + f * (t[i + 1] - t[i]);
            return true;
        }
    }
    return false;
}

void append(std::vector<Phase>& parts,
            std::size_t eventId,
            std::size_t excursionId,
            double start, double end,
            PhaseType type) {
    if (end - start <= 1e-6) return;

    Phase p;
    p.event_id = eventId;
    p.excursion_ids = {excursionId};
    p.start_s = start;
    p.end_s = end;
    p.duration_s = end - start;
    p.type = type;
    parts.push_back(std::move(p));
}

std::vector<Phase> classify(
    const std::vector<double>& t,
    const std::vector<double>& y,
    std::size_t a, std::size_t b,
    std::size_t eventId, std::size_t excursionId,
    const ManeuverPhaseConfig& c
) {
    std::vector<Phase> parts;
    if (b <= a) return parts;

    const double d0 = y[a], d1 = y[b];
    const double t0 = t[a], t1 = t[b];

    // Recovery -> reversal -> entry across +/-3 deg.
    if (d0 * d1 < 0.0 &&
        std::min(std::abs(d0), std::abs(d1))
            >= c.active_steer_deg) {

        const double from = d0 > 0.0
            ? c.active_steer_deg : -c.active_steer_deg;
        const double to = -from;

        double tc1 = 0.0, tc2 = 0.0;
        if (crossing(t, y, a, b, from, tc1) &&
            crossing(t, y, a, b, to, tc2) &&
            t0 <= tc1 && tc1 <= tc2 && tc2 <= t1) {

            append(parts, eventId, excursionId,
                   t0, tc1, PhaseType::Recovery);
            append(parts, eventId, excursionId,
                   tc1, tc2, PhaseType::Reversal);
            append(parts, eventId, excursionId,
                   tc2, t1, PhaseType::Entry);
            return parts;
        }
    }

    double maxAbs = 0.0, meanAbs = 0.0;
    std::vector<double> localT, localY;

    for (std::size_t i = a; i <= b; ++i) {
        maxAbs = std::max(maxAbs, std::abs(y[i]));
        meanAbs += std::abs(y[i]);
        localT.push_back(t[i]);
        localY.push_back(y[i]);
    }
    meanAbs /= localY.size();

    double rateP80 = INFINITY;
    if (localY.size() >= 3) {
        const auto rate = gradient(localT, localY);
        std::vector<double> absRate;
        for (double v : rate) absRate.push_back(std::abs(v));
        rateP80 = percentile80(std::move(absRate));
    }

    PhaseType type = PhaseType::Transition;

    if (maxAbs < c.active_steer_deg &&
        meanAbs < 0.5 * c.active_steer_deg) {
        type = PhaseType::Neutral;
    }
    else if (meanAbs >= c.active_steer_deg &&
             rateP80 <= c.hold_rate_deg_s) {
        type = PhaseType::Hold;
    }
    else if (std::abs(d1) > std::abs(d0)) {
        type = PhaseType::Entry;
    }
    else if (std::abs(d1) < std::abs(d0)) {
        type = PhaseType::Recovery;
    }

    append(parts, eventId, excursionId, t0, t1, type);
    return parts;
}

// Python hold_interval(): find a plateau containing
// the major extremum (not merely a flat nearby segment).
bool holdInterval(
    const std::vector<double>& t,
    const std::vector<double>& y,
    const std::vector<double>& rate,
    std::size_t peak, std::size_t lo, std::size_t hi,
    const ManeuverPhaseConfig& c,
    double& start, double& end
) {
    const double peakValue = y[peak];

    const auto valid = [&](std::size_t i) {
        return std::abs(rate[i]) <= c.hold_rate_deg_s &&
               std::abs(y[i] - peakValue)
                   <= c.hold_peak_tolerance_deg &&
               std::abs(y[i]) >= c.active_steer_deg;
    };

    if (!valid(peak)) return false;

    std::size_t left = peak, right = peak;
    while (left > lo && valid(left - 1)) --left;
    while (right < hi && valid(right + 1)) ++right;

    if (t[right] - t[left] < c.min_hold_s)
        return false;

    start = t[left];
    end = t[right];
    return true;
}

std::vector<Phase> insertHold(
    const std::vector<Phase>& parts,
    double hs, double he
) {
    std::vector<Phase> out;

    for (const auto& p : parts) {
        if (p.end_s <= hs || p.start_s >= he) {
            out.push_back(p);
            continue;
        }

        auto addPart = [&](double a, double b, PhaseType type) {
            if (b - a <= 1e-6) return;
            Phase copy = p;
            copy.start_s = a;
            copy.end_s = b;
            copy.duration_s = b - a;
            copy.type = type;
            out.push_back(std::move(copy));
        };

        addPart(p.start_s, std::min(hs, p.end_s), p.type);
        addPart(std::max(hs, p.start_s),
                std::min(he, p.end_s), PhaseType::Hold);
        addPart(std::max(he, p.start_s), p.end_s, p.type);
    }
    return out;
}

std::vector<Phase> mergeAdjacent(
    const std::vector<Phase>& parts
) {
    std::vector<Phase> out;

    for (const auto& p : parts) {
        if (!out.empty() &&
            out.back().event_id == p.event_id &&
            out.back().type == p.type &&
            std::abs(out.back().end_s - p.start_s) <= 0.02) {

            auto& last = out.back();
            last.end_s = p.end_s;
            last.duration_s = last.end_s - last.start_s;

            for (auto id : p.excursion_ids) {
                if (std::find(last.excursion_ids.begin(),
                              last.excursion_ids.end(), id)
                    == last.excursion_ids.end())
                    last.excursion_ids.push_back(id);
            }
        }
        else {
            out.push_back(p);
        }
    }

    return out;
}

} // anonymous namespace

std::string toString(PhaseType type) {
    switch (type) {
        case PhaseType::Entry: return "ENTRY";
        case PhaseType::Hold: return "HOLD";
        case PhaseType::Recovery: return "RECOVERY";
        case PhaseType::Reversal: return "REVERSAL";
        case PhaseType::Neutral: return "NEUTRAL";
        case PhaseType::Transition: return "TRANSITION";
        case PhaseType::SustainedTurn: return "SUSTAINED_TURN";
        case PhaseType::InitialBoundary: return "INITIAL_BOUNDARY";
    }
    return "UNKNOWN";
}

ManeuverPhaseAnalyzer::ManeuverPhaseAnalyzer(
    ManeuverPhaseConfig config
) : config_(config) {
    if (!(config_.smooth_seconds > 0.0) ||
        !(config_.active_steer_deg > 0.0) ||
        !(config_.hold_rate_deg_s >= 0.0) ||
        !(config_.hold_peak_tolerance_deg >= 0.0) ||
        !(config_.min_hold_s > 0.0)) {
        throw std::invalid_argument("Invalid phase config");
    }
}

std::vector<ManeuverPhase> ManeuverPhaseAnalyzer::analyze(
    const p4::io::VBOData& data,
    const std::vector<ManeuverEvent>& events,
    const std::vector<MajorExcursion>& excursions
) const {
    const auto s = load(data);
    const auto filtered = smooth(
        s.steer, s.t, config_.smooth_seconds
    );

    std::vector<Phase> result;

    for (const auto& event : events) {
        const auto lo = event.start_index;
        const auto hi = event.end_index;

        if (lo >= hi || hi > s.t.size())
            throw std::invalid_argument("Invalid event bounds");

        std::vector<Phase> parts;

        // Legacy special cases.
        if (event.candidate == EventCandidate::Circle) {
            append(parts, event.event_id, 1,
                   s.t[lo], s.t[hi - 1],
                   PhaseType::SustainedTurn);
        }
        else if (event.start_s <= 0.05 &&
                 event.end_s <= 1.0) {
            append(parts, event.event_id, 1,
                   s.t[lo], s.t[hi - 1],
                   PhaseType::InitialBoundary);
        }
        else {
            // Gradient over the entire event.
            const std::vector<double> localT(
                s.t.begin() + lo, s.t.begin() + hi);
            const std::vector<double> localY(
                filtered.begin() + lo,
                filtered.begin() + hi);

            const auto localRate = gradient(localT, localY);

            for (const auto& e : excursions) {
                if (e.event_id != event.event_id) continue;

                const auto a = e.start_index;
                const auto b = e.end_index;

                if (a < lo || b >= hi || b <= a)
                    throw std::runtime_error(
                        "Excursion outside event");

                auto pieces = classify(
                    s.t, filtered, a, b,
                    event.event_id, e.excursion_id, config_
                );

                // In the Python version, a HOLD is
                // considered around either extremum.

                for (const auto peak : {a, b}) {
                    double hs = 0.0, he = 0.0;

                    // Convert global VBO index to event-local index.
                    const std::size_t localPeak = peak - lo;

                    if (!holdInterval(
                            localT,
                            localY,
                            localRate,
                            localPeak,
                            0,
                            localT.size() - 1,
                            config_,
                            hs,
                            he)) {
                        continue;
                    }

                    // Restrict HOLD to this major excursion.
                    hs = std::max(hs, s.t[a]);
                    he = std::min(he, s.t[b]);

                    if (he - hs >= config_.min_hold_s) {
                        pieces = insertHold(pieces, hs, he);
                    }
                }


                parts.insert(
                    parts.end(), pieces.begin(), pieces.end()
                );
            }

            std::stable_sort(
                parts.begin(), parts.end(),
                [](const Phase& a, const Phase& b) {
                    return a.start_s < b.start_s;
                }
            );
            parts = mergeAdjacent(parts);
        }

        for (auto& p : parts) {
            p.phase_id = result.size() + 1;

            p.steer_start_deg =
                interp(s.t, filtered, p.start_s);
            p.steer_end_deg =
                interp(s.t, filtered, p.end_s);

            // Mean statistics over samples in phase.
            double sumSpeed = 0.0, sumYaw = 0.0;
            std::size_t count = 0;

            for (std::size_t i = lo; i < hi; ++i) {
                if (s.t[i] >= p.start_s &&
                    s.t[i] <= p.end_s) {
                    sumSpeed += s.speed[i];
                    sumYaw += s.yaw[i];
                    ++count;
                }
            }

            if (count != 0) {
                p.speed_mean_kmh = sumSpeed / count;
                p.yaw_mean_deg_s = sumYaw / count;
            }

            result.push_back(std::move(p));
        }
    }

    return result;
}

} // namespace p4::maneuvers
