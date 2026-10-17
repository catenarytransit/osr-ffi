#pragma once

#include <cstdint>
#include <memory>
#include "rust/cxx.h"

namespace catenary_osr {
struct Coordinate;
struct RoutingOptions;
struct RouteResult;
struct SnapCandidate;
struct MapMatchResult;

class Router {
public:
  explicit Router(rust::Str);
  ~Router();
  Router(Router const&) = delete;
  Router& operator=(Router const&) = delete;

  std::uint64_t node_count() const noexcept;
  std::uint64_t way_count() const noexcept;
  bool has_elevations() const noexcept;

  RouteResult one_to_one(Coordinate const&, Coordinate const&,
                         RoutingOptions const&,
                         rust::Slice<std::int64_t const>) const;

  rust::Vec<RouteResult> one_to_many(
      Coordinate const&, rust::Slice<Coordinate const>,
      RoutingOptions const&, rust::Slice<std::int64_t const>) const;

  rust::Vec<SnapCandidate> snap(
      Coordinate const&, double, bool) const;

  MapMatchResult map_match(
      rust::Slice<Coordinate const>, std::uint8_t, float,
      rust::Slice<std::int64_t const>) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

std::unique_ptr<Router> open_router(rust::Str);
}  // namespace catenary_osr
