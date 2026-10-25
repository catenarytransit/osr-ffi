#include "include/router.h"
#include "osr-ffi/src/lib.rs.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/map_matching.h"
#include "osr/routing/parameters.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

namespace catenary_osr {
namespace {

static_assert(static_cast<std::uint8_t>(osr::search_profile::kFoot) == 0);
static_assert(static_cast<std::uint8_t>(osr::search_profile::kFerry) == 16);
static_assert(static_cast<std::uint8_t>(osr::routing_algorithm::kDijkstra) == 0);
static_assert(static_cast<std::uint8_t>(osr::routing_algorithm::kAStarBi) == 1);

osr::location to_location(Coordinate const& c) {
  if (!std::isfinite(c.lat) || !std::isfinite(c.lon) ||
      c.lat < -90.0 || c.lat > 90.0 || c.lon < -180.0 || c.lon > 180.0 ||
      (c.has_level && !std::isfinite(c.level))) {
    throw std::invalid_argument("invalid latitude, longitude or level");
  }
  return osr::location{
      {c.lat, c.lon},
      c.has_level ? osr::level_t{c.level} : osr::kNoLevel};
}

osr::search_profile get_profile(std::uint8_t code) {
  if (code >= osr::kNumProfiles) {
    throw std::invalid_argument("unknown OSR routing profile");
  }
  return static_cast<osr::search_profile>(code);
}

osr::routing_algorithm get_algorithm(std::uint8_t code) {
  if (code > 1U) {
    throw std::invalid_argument("unknown OSR routing algorithm");
  }
  return static_cast<osr::routing_algorithm>(code);
}

osr::profile_parameters get_params(osr::search_profile profile, float speed) {
  auto parameters = osr::get_parameters(profile);
  if (speed == 0.F) {
    return parameters;
  }
  if (!std::isfinite(speed) || speed <= 0.F) {
    throw std::invalid_argument("walking speed must be positive and finite");
  }
  if (profile == osr::search_profile::kFoot) {
    std::get<osr::foot<false, osr::elevator_tracking>::parameters>(
        parameters).speed_meters_per_second_ = speed;
  } else if (profile == osr::search_profile::kWheelchair) {
    std::get<osr::foot<true, osr::elevator_tracking>::parameters>(
        parameters).speed_meters_per_second_ = speed;
  } else {
    throw std::invalid_argument(
        "walking speed is only valid for foot/wheelchair");
  }
  return parameters;
}

osr::direction get_direction(std::uint8_t code) {
  if (code > 1U) {
    throw std::invalid_argument("unknown routing direction");
  }
  return code == 0U ? osr::direction::kForward : osr::direction::kBackward;
}

std::optional<osr::routing_time_t> get_start_time(RoutingOptions const& o) {
  if (!o.has_start_time) {
    return std::nullopt;
  }
  return osr::routing_time_t{std::chrono::seconds{o.start_time_unix_seconds}};
}

void validate_options(RoutingOptions const& o) {
  (void)get_profile(o.profile);
  (void)get_direction(o.direction);
  if (o.max_cost == 0U || !std::isfinite(o.max_match_distance_meters) ||
      o.max_match_distance_meters <= 0.0) {
    throw std::invalid_argument("positive search cost and match radius required");
  }
  if (o.has_start_time && o.algorithm != 0U) {
    // OSR's high-level bidirectional route overload does not forward the time.
    throw std::invalid_argument(
        "time-dependent routing requires the Dijkstra algorithm");
  }
}

Coordinate to_coordinate(geo::latlng const& p) {
  return Coordinate{p.lat(), p.lng(), false, 0.F};
}

RouteResult to_result(osr::ways const& w,
                      std::optional<osr::path> const& path,
                      bool with_segments) {
  RouteResult output{};
  if (!path.has_value()) {
    return output;
  }
  output.found = true;
  output.cost = path->cost_;
  output.duration_seconds = path->duration_.count();
  output.distance_meters = path->dist_;
  output.uses_elevator = path->uses_elevator_;
  output.elevation_up = cista::to_idx(path->elevation_.up_);
  output.elevation_down = cista::to_idx(path->elevation_.down_);

  if (with_segments) {
    output.segments.reserve(path->segments_.size());
    for (auto const& s : path->segments_) {
      PathSegment segment{};
      segment.cost = s.cost_;
      segment.duration_seconds = s.duration_.count();
      segment.distance_meters = s.dist_;
      segment.from_node = cista::to_idx(s.from_);
      segment.to_node = cista::to_idx(s.to_);
      segment.mode = static_cast<std::uint8_t>(s.mode_);
      segment.from_level = s.from_level_.to_float();
      segment.to_level = s.to_level_.to_float();
      segment.elevation_up = cista::to_idx(s.elevation_.up_);
      segment.elevation_down = cista::to_idx(s.elevation_.down_);
      if (auto id = w.get_osm_way(s.way_); id.has_value()) {
        segment.has_osm_way_id = true;
        segment.osm_way_id = *id;
      }
      segment.geometry.reserve(s.polyline_.size());
      for (auto const& c : s.polyline_) {
        segment.geometry.push_back(to_coordinate(c));
      }
      output.segments.push_back(std::move(segment));
    }
  }
  return output;
}

}  // namespace

struct Router::Impl {
  osr::ways ways_;
  osr::lookup lookup_;
  std::unique_ptr<osr::elevation_storage> elevations_;

  explicit Impl(std::filesystem::path const& dir)
      : ways_{dir, cista::mmap::protection::READ},
        lookup_{ways_, dir, cista::mmap::protection::READ},
        elevations_{osr::elevation_storage::try_open(dir)} {}
};

Router::Router(rust::Str dir)
    : impl_{std::make_unique<Impl>(
          std::filesystem::path{std::string(dir.data(), dir.size())})} {}

Router::~Router() = default;

std::unique_ptr<Router> open_router(rust::Str path) {
  return std::make_unique<Router>(path);
}

std::uint64_t Router::node_count() const noexcept {
  return impl_->ways_.n_nodes();
}

std::uint64_t Router::way_count() const noexcept {
  return impl_->ways_.n_ways();
}

bool Router::has_elevations() const noexcept {
  return impl_->elevations_ != nullptr;
}

RouteResult Router::one_to_one(
    Coordinate const& from, Coordinate const& to,
    RoutingOptions const& options,
    rust::Slice<std::int64_t const> blocked_ids) const {
  validate_options(options);
  auto const profile = get_profile(options.profile);
  auto const parameters =
      get_params(profile, options.foot_speed_meters_per_second);
  auto const& w = impl_->ways_;
  auto blocked = osr::bitvec<osr::node_idx_t>{};
  if (!blocked_ids.empty()) {
    blocked.resize(w.n_nodes());
    for (auto const id : blocked_ids) {
      auto const n = w.find_node_idx(osr::to_osm_node_idx(id));
      if (!n) {
        throw std::invalid_argument("blocked OSM node not found in this graph");
      }
      blocked.set(*n);
    }
  }
  auto const path = osr::route(
      parameters, w, impl_->lookup_, profile,
      to_location(from), to_location(to),
      std::chrono::seconds{options.max_cost},
      get_direction(options.direction), options.max_match_distance_meters,
      blocked_ids.empty() ? nullptr : &blocked,
      nullptr, impl_->elevations_.get(), get_algorithm(options.algorithm),
      get_start_time(options));
  return to_result(w, path, options.include_segments);
}

rust::Vec<RouteResult> Router::one_to_many(
    Coordinate const& from, rust::Slice<Coordinate const> destinations,
    RoutingOptions const& options,
    rust::Slice<std::int64_t const> blocked_ids) const {
  validate_options(options);
  if (options.algorithm != 0U) {
    throw std::invalid_argument(
        "one-to-many uses OSR Dijkstra; select Algorithm::Dijkstra");
  }
  if (destinations.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("too many destinations");
  }
  auto const profile = get_profile(options.profile);
  auto const parameters =
      get_params(profile, options.foot_speed_meters_per_second);
  auto const& w = impl_->ways_;
  auto blocked = osr::bitvec<osr::node_idx_t>{};
  if (!blocked_ids.empty()) {
    blocked.resize(w.n_nodes());
    for (auto const id : blocked_ids) {
      auto const n = w.find_node_idx(osr::to_osm_node_idx(id));
      if (!n) {
        throw std::invalid_argument("blocked OSM node not found in this graph");
      }
      blocked.set(*n);
    }
  }
  auto targets = std::vector<osr::location>{};
  targets.reserve(destinations.size());
  for (auto const& p : destinations) {
    targets.push_back(to_location(p));
  }
  auto const paths = osr::route(
      parameters, w, impl_->lookup_, profile, to_location(from), targets,
      std::chrono::seconds{options.max_cost}, get_direction(options.direction),
      options.max_match_distance_meters,
      blocked_ids.empty() ? nullptr : &blocked,
      nullptr, impl_->elevations_.get(),
      [&options](osr::path const&) { return options.include_segments; },
      get_start_time(options));
  auto out = rust::Vec<RouteResult>{};
  out.reserve(paths.size());
  for (auto const& p : paths) {
    out.push_back(to_result(w, p, options.include_segments));
  }
  return out;
}

rust::Vec<SnapCandidate> Router::snap(
    Coordinate const& point, double radius, bool expand_search) const {
  if (!std::isfinite(radius) || radius <= 0.0) {
    throw std::invalid_argument("positive matching radius required");
  }
  auto const location = to_location(point);
  // get_raw_way_candidates is private in OSR's public lookup API.
  // get_raw_match is the supported public method; strict-radius mode
  // filters its expanded candidates without relying on private internals.
  auto const matches = impl_->lookup_.get_raw_match(location, radius);
  auto out = rust::Vec<SnapCandidate>{};
  out.reserve(matches.size());
  for (auto const& m : matches) {
    if (!expand_search && m.dist_to_way_ >= radius) {
      continue;
    }
    SnapCandidate c{};
    if (auto id = impl_->ways_.get_osm_way(m.way_)) {
      c.osm_way_id = *id;
    }
    c.distance_to_way_meters = m.dist_to_way_;
    if (m.left_.valid()) {
      if (auto id = impl_->ways_.get_osm_node(m.left_.node_)) {
        c.left_osm_node_id = *id;
        c.has_left_node = true;
      }
      c.left_distance_meters = m.left_.dist_to_node_;
    }
    if (m.right_.valid()) {
      if (auto id = impl_->ways_.get_osm_node(m.right_.node_)) {
        c.right_osm_node_id = *id;
        c.has_right_node = true;
      }
      c.right_distance_meters = m.right_.dist_to_node_;
    }
    out.push_back(std::move(c));
  }
  return out;
}

MapMatchResult Router::map_match(
    rust::Slice<Coordinate const> trace, std::uint8_t profile_code,
    float foot_speed, rust::Slice<std::int64_t const> blocked_ids) const {
  if (trace.empty()) {
    throw std::invalid_argument("map_match requires coordinates");
  }
  auto const profile = get_profile(profile_code);
  auto const parameters = get_params(profile, foot_speed);
  auto locations = std::vector<osr::location>{};
  locations.reserve(trace.size());
  for (auto const& p : trace) {
    locations.push_back(to_location(p));
  }
  auto blocked = osr::bitvec<osr::node_idx_t>{};
  if (!blocked_ids.empty()) {
    blocked.resize(impl_->ways_.n_nodes());
    for (auto const id : blocked_ids) {
      auto const n = impl_->ways_.find_node_idx(osr::to_osm_node_idx(id));
      if (!n) {
        throw std::invalid_argument("blocked OSM node not found in this graph");
      }
      blocked.set(*n);
    }
  }
  auto const matched = osr::map_match(
      impl_->ways_, impl_->lookup_, profile, parameters, locations,
      blocked_ids.empty() ? nullptr : &blocked,
      impl_->elevations_.get());
  MapMatchResult out{};
  out.path = to_result(impl_->ways_,
                       std::optional<osr::path>{matched.path_}, true);
  for (auto const offset : matched.segment_offsets_) {
    out.segment_offsets.push_back(offset);
  }
  out.routed_count = matched.n_routed_;
  out.beeline_count = matched.n_beelined_;
  out.dijkstra_early_terminations =
      matched.n_dijkstra_early_terminations_;
  out.dijkstra_full_runs = matched.n_dijkstra_full_runs_;
  out.computation_milliseconds = matched.total_duration_.count();
  return out;
}

}  // namespace catenary_osr
