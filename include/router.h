#pragma once

#include <cstdint>
#include <memory>
#include "rust/cxx.h"

namespace catenary_osr {
struct Coordinate;
struct RoutingOptions;
struct RouteResult;
struct SnapCandidate;
struct PlatformLevelMatch;
struct MapMatchResult;
struct RawWayCandidate;
struct RawMatchResult;
struct MatchOptions;
class Router;

// Complete C++ type required by cxx for UniquePtr<MatchStore>.
// OSR internals remain hidden so cxx's generated source needs no OSR headers.
class MatchStore {
public:
  explicit MatchStore(Router const*);
  ~MatchStore();
  MatchStore(MatchStore const&) = delete;
  MatchStore& operator=(MatchStore const&) = delete;

private:
  friend class Router;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class Router {
public:
  explicit Router(rust::Str);
  ~Router();
  Router(Router const&) = delete;
  Router& operator=(Router const&) = delete;

  std::uint64_t node_count() const noexcept;
  std::uint64_t way_count() const noexcept;
  bool has_elevations() const noexcept;

  // Resolve a named GTFS stop against extracted OSM platform geometry.
  // Unavailable platform metadata or no reliable name match returns found=false.
  PlatformLevelMatch platform_level(
      Coordinate const&, rust::Str stop_name, double max_distance_meters) const;

  RouteResult one_to_one(Coordinate const&, Coordinate const&,
                         RoutingOptions const&,
                         rust::Slice<std::int64_t const>) const;

  rust::Vec<RouteResult> one_to_many(
      Coordinate const&, rust::Slice<Coordinate const>,
      RoutingOptions const&, rust::Slice<std::int64_t const>) const;

  rust::Vec<SnapCandidate> snap(
      Coordinate const&, double, bool) const;

  rust::Vec<RawWayCandidate> get_raw_match(
      Coordinate const&, double) const;
  RawMatchResult lookup_match(
      Coordinate const&, MatchOptions const&,
      rust::Slice<RawWayCandidate const>,
      rust::Slice<std::int64_t const>) const;
  rust::Vec<RouteResult> route_one_to_many_matched(
      Coordinate const&, rust::Slice<Coordinate const>,
      RawMatchResult const&, RawMatchResult const&,
      RoutingOptions const&, rust::Slice<std::int64_t const>) const;

  std::unique_ptr<MatchStore> create_match_store() const;
  std::uint32_t append_lookup_match(
      MatchStore&, Coordinate const&, MatchOptions const&,
      rust::Slice<RawWayCandidate const>) const;
  RawMatchResult export_match(MatchStore const&, std::uint32_t) const;
  rust::Vec<RouteResult> route_one_to_many_cached(
      MatchStore const&, std::uint32_t, MatchStore const&,
      rust::Slice<std::uint32_t const>, RoutingOptions const&) const;

  MapMatchResult map_match(
      rust::Slice<Coordinate const>, std::uint8_t, float,
      rust::Slice<std::int64_t const>) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

std::unique_ptr<Router> open_router(rust::Str);
// Shares one immutable, read-only graph across concurrent routing threads.
std::shared_ptr<Router> open_shared_router(rust::Str);
}  // namespace catenary_osr
