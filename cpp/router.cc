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
      static_cast<osr::cost_t>(options.max_cost),
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
      static_cast<osr::cost_t>(options.max_cost), get_direction(options.direction),
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


namespace {

void validate_match_options(MatchOptions const& o) {
  (void)get_profile(o.profile);
  (void)get_direction(o.direction);
  if (!std::isfinite(o.max_match_distance_meters) ||
      o.max_match_distance_meters <= 0.0) {
    throw std::invalid_argument("positive finite match radius required");
  }
  (void)get_params(get_profile(o.profile), o.foot_speed_meters_per_second);
}

osr::bitvec<osr::node_idx_t> build_blocked(
    osr::ways const& w, rust::Slice<std::int64_t const> blocked_ids) {
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
  return blocked;
}

osr::raw_node_candidate import_raw_node(RawNodeCandidate const& n,
                                         osr::ways const& w) {
  auto const invalid = std::numeric_limits<std::uint32_t>::max();
  if (n.node_index != invalid && n.node_index >= w.n_nodes()) {
    throw std::invalid_argument("raw match node index outside graph");
  }
  if (!std::isfinite(n.distance_to_node) || n.distance_to_node < 0.F) {
    throw std::invalid_argument("invalid raw node distance");
  }
  return {osr::node_idx_t{n.node_index}, n.distance_to_node};
}

std::vector<osr::raw_way_candidate> import_raw(
    rust::Slice<RawWayCandidate const> input, osr::ways const& w) {
  auto out = std::vector<osr::raw_way_candidate>{};
  out.reserve(input.size());
  for (auto const& r : input) {
    if (r.way_index >= w.n_ways() ||
        !std::isfinite(r.distance_to_way) || r.distance_to_way < 0.F) {
      throw std::invalid_argument("invalid raw way index/distance");
    }
    out.push_back(osr::raw_way_candidate{
        r.distance_to_way, osr::way_idx_t{r.way_index},
        import_raw_node(r.left, w), import_raw_node(r.right, w)});
  }
  return out;
}

RawNodeCandidate export_raw_node(osr::raw_node_candidate const& n) {
  return {cista::to_idx(n.node_), n.dist_to_node_};
}

RawMatchResult export_matches(osr::match_result const& m) {
  auto out = RawMatchResult{};
  out.offsets.push_back(0U);
  for (auto i = std::uint32_t{0}; i < m.size(); ++i) {
    auto const v = m[osr::match_idx_t{i}];
    out.level_codes.push_back(cista::to_idx(v.lvl_));
    for (auto j = std::size_t{0}; j < v.size(); ++j) {
      out.distance_to_way.push_back(v.dist_to_way_[j]);
      out.way_indices.push_back(cista::to_idx(v.way_[j]));
      auto const convert = [](osr::match_result::node const& n) {
        return MatchedNode{cista::to_idx(n.node_), n.dist_to_node_, n.cost_};
      };
      out.nodes.push_back(MatchedNodes{
          convert(v.nodes_[j].left_), convert(v.nodes_[j].right_)});
    }
    if (out.way_indices.size() > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument("too many match candidates");
    }
    out.offsets.push_back(static_cast<std::uint32_t>(out.way_indices.size()));
  }
  return out;
}

osr::match_result import_matches(RawMatchResult const& r,
                                 osr::ways const& w) {
  if (r.offsets.size() != r.level_codes.size() + 1 ||
      r.offsets.empty() || r.offsets.front() != 0 ||
      r.way_indices.size() != r.distance_to_way.size() ||
      r.way_indices.size() != r.nodes.size() ||
      r.offsets.back() != r.way_indices.size()) {
    throw std::invalid_argument("malformed match_result columns or offsets");
  }
  auto out = osr::match_result{};
  auto const check_node = [&](MatchedNode const& n) {
    if (n.node_index != std::numeric_limits<std::uint32_t>::max() &&
        n.node_index >= w.n_nodes()) {
      throw std::invalid_argument("matched node outside graph");
    }
    if (!std::isfinite(n.distance_to_node) || n.distance_to_node < 0.F) {
      throw std::invalid_argument("invalid matched node distance");
    }
    return osr::match_result::node{
        osr::node_idx_t{n.node_index}, n.distance_to_node, n.cost};
  };
  for (auto i = std::size_t{0}; i < r.level_codes.size(); ++i) {
    auto const first = r.offsets[i], last = r.offsets[i + 1];
    if (first > last || last > r.way_indices.size() ||
        r.level_codes[i] > cista::to_idx(osr::level_t{osr::kMaxLevel})) {
      throw std::invalid_argument("invalid match bucket or level");
    }
    out.start(osr::level_t{r.level_codes[i]});
    for (auto j = first; j < last; ++j) {
      if (r.way_indices[j] >= w.n_ways() ||
          !std::isfinite(r.distance_to_way[j]) ||
          r.distance_to_way[j] < 0.F) {
        throw std::invalid_argument("invalid matched way index/distance");
      }
      auto const& pair = r.nodes[j];
      out.add(r.distance_to_way[j], osr::way_idx_t{r.way_indices[j]},
              osr::match_result::nodes{
                  check_node(pair.left), check_node(pair.right)});
    }
    out.finish();
  }
  return out;
}

bool same_matches(MatchOptions const& a, MatchOptions const& b) {
  return a.profile == b.profile && a.direction == b.direction &&
         a.reverse == b.reverse &&
         a.max_match_distance_meters == b.max_match_distance_meters &&
         a.foot_speed_meters_per_second == b.foot_speed_meters_per_second;
}

}  // namespace

struct MatchStore {
  Router const* owner_;
  osr::match_result matches_;
  std::vector<osr::location> locations_;
  std::optional<MatchOptions> options_;

  explicit MatchStore(Router const* owner) : owner_{owner} {}
};

rust::Vec<RawWayCandidate> Router::get_raw_match(
    Coordinate const& point, double max_distance) const {
  if (!std::isfinite(max_distance) || max_distance <= 0.0) {
    throw std::invalid_argument("positive finite raw match radius required");
  }
  auto const native = impl_->lookup_.get_raw_match(
      to_location(point), max_distance);
  auto out = rust::Vec<RawWayCandidate>{};
  out.reserve(native.size());
  for (auto const& r : native) {
    out.push_back(RawWayCandidate{r.dist_to_way_, cista::to_idx(r.way_),
                                 export_raw_node(r.left_),
                                 export_raw_node(r.right_)});
  }
  return out;
}

RawMatchResult Router::lookup_match(
    Coordinate const& point, MatchOptions const& opts,
    rust::Slice<RawWayCandidate const> raw_candidates,
    rust::Slice<std::int64_t const> blocked_ids) const {
  validate_match_options(opts);
  auto const p = get_profile(opts.profile);
  auto const blocked = build_blocked(impl_->ways_, blocked_ids);
  auto const raw = import_raw(raw_candidates, impl_->ways_);
  auto m = osr::match_result{};
  impl_->lookup_.match(
      get_params(p, opts.foot_speed_meters_per_second),
      to_location(point), opts.reverse, get_direction(opts.direction),
      opts.max_match_distance_meters,
      blocked_ids.empty() ? nullptr : &blocked, p, raw, m);
  return export_matches(m);
}

rust::Vec<RouteResult> Router::route_one_to_many_matched(
    Coordinate const& origin, rust::Slice<Coordinate const> destinations,
    RawMatchResult const& origin_match,
    RawMatchResult const& destination_matches,
    RoutingOptions const& options,
    rust::Slice<std::int64_t const> blocked_ids) const {
  validate_options(options);
  if (options.algorithm != 0U ||
      options.max_cost > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("one-to-many requires Dijkstra and <=65535 max cost");
  }
  auto const origin_native = import_matches(origin_match, impl_->ways_);
  auto const targets_native = import_matches(destination_matches, impl_->ways_);
  if (origin_native.size() != 1U ||
      targets_native.size() != destinations.size()) {
    throw std::invalid_argument("match buckets must align with origin and destinations");
  }
  auto targets = std::vector<osr::location>{};
  targets.reserve(destinations.size());
  for (auto const& c : destinations) targets.push_back(to_location(c));
  auto const blocked = build_blocked(impl_->ways_, blocked_ids);
  auto const states = osr::route_one_to_many(
      get_params(get_profile(options.profile), options.foot_speed_meters_per_second),
      impl_->ways_, impl_->lookup_, get_profile(options.profile),
      to_location(origin), targets,
      origin_native[osr::match_idx_t{0U}], targets_native,
      options.max_cost, get_direction(options.direction),
      blocked_ids.empty() ? nullptr : &blocked, nullptr,
      impl_->elevations_.get(),
      [&options](osr::path const&) { return options.include_segments; },
      get_start_time(options));
  auto out = rust::Vec<RouteResult>{};
  for (auto const& p : states->results()) {
    out.push_back(to_result(impl_->ways_, p, options.include_segments));
  }
  return out;
}

std::unique_ptr<MatchStore> Router::create_match_store() const {
  return std::make_unique<MatchStore>(this);
}

std::uint32_t Router::append_lookup_match(
    MatchStore& store, Coordinate const& point, MatchOptions const& opts,
    rust::Slice<RawWayCandidate const> raw_candidates) const {
  if (store.owner_ != this) {
    throw std::invalid_argument("match store belongs to another router");
  }
  validate_match_options(opts);
  if (store.options_ && !same_matches(*store.options_, opts)) {
    throw std::invalid_argument("inconsistent match store options");
  }
  if (store.matches_.size() >= std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("match store full");
  }
  auto const loc = to_location(point);
  auto const raw = import_raw(raw_candidates, impl_->ways_);
  impl_->lookup_.match(
      get_params(get_profile(opts.profile), opts.foot_speed_meters_per_second),
      loc, opts.reverse, get_direction(opts.direction),
      opts.max_match_distance_meters, nullptr,
      get_profile(opts.profile), raw, store.matches_);
  store.locations_.push_back(loc);
  store.options_ = opts;
  return static_cast<std::uint32_t>(store.locations_.size() - 1U);
}

RawMatchResult Router::export_match(
    MatchStore const& store, std::uint32_t index) const {
  if (store.owner_ != this || index >= store.matches_.size()) {
    throw std::invalid_argument("match store owner/index mismatch");
  }
  auto m = osr::match_result{};
  m.append(store.matches_, osr::match_idx_t{index});
  return export_matches(m);
}

rust::Vec<RouteResult> Router::route_one_to_many_cached(
    MatchStore const& origins, std::uint32_t origin_index,
    MatchStore const& destinations, rust::Slice<std::uint32_t const> indices,
    RoutingOptions const& options) const {
  validate_options(options);
  if (options.algorithm != 0U || options.has_start_time ||
      options.max_cost > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("cached routing requires static Dijkstra and <=65535 cost");
  }
  if (origins.owner_ != this || destinations.owner_ != this ||
      !origins.options_ || !destinations.options_ ||
      origin_index >= origins.matches_.size()) {
    throw std::invalid_argument("invalid cached match store or origin");
  }
  auto const& a = *origins.options_;
  auto const& b = *destinations.options_;
  if (a.profile != options.profile || b.profile != options.profile ||
      a.direction != options.direction || b.direction != options.direction ||
      a.reverse || !b.reverse ||
      a.foot_speed_meters_per_second != options.foot_speed_meters_per_second ||
      b.foot_speed_meters_per_second != options.foot_speed_meters_per_second) {
    throw std::invalid_argument("cached origin/destination matching settings differ from routing");
  }
  auto to = std::vector<osr::location>{};
  auto matches = osr::match_result{};
  to.reserve(indices.size());
  for (auto const idx : indices) {
    if (idx >= destinations.matches_.size()) {
      throw std::invalid_argument("cached destination index out of range");
    }
    to.push_back(destinations.locations_[idx]);
    matches.append(destinations.matches_, osr::match_idx_t{idx});
  }
  auto const states = osr::route_one_to_many(
      get_params(get_profile(options.profile), options.foot_speed_meters_per_second),
      impl_->ways_, impl_->lookup_, get_profile(options.profile),
      origins.locations_[origin_index], to,
      origins.matches_[osr::match_idx_t{origin_index}], matches,
      options.max_cost, get_direction(options.direction),
      nullptr, nullptr, impl_->elevations_.get(),
      [&options](osr::path const&) { return options.include_segments; },
      std::nullopt);
  auto out = rust::Vec<RouteResult>{};
  for (auto const& p : states->results()) {
    out.push_back(to_result(impl_->ways_, p, options.include_segments));
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
