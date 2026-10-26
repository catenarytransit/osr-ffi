//! Typed native bindings to MOTIS Open Street Router (OSR).
//!
//! The routing graph and coordinate index remain in C++ and are opened once.
//! OSR's original Dijkstra, bidirectional A*, one-to-many and map-matching
//! implementations are invoked directly. No Rust reimplementation is involved.

#[cxx::bridge(namespace = "catenary_osr")]
pub mod ffi {
    /// Geographic coordinate in degrees, with an optional OSM indoor level.
    #[derive(Debug, Clone)]
    pub struct Coordinate {
        pub lat: f64,
        pub lon: f64,
        pub has_level: bool,
        pub level: f32,
    }

    /// Input for a routing search. Profile and algorithm codes are defined
    /// by the Rust Profile and Algorithm enums, not the OSR numeric ABI.
    #[derive(Debug, Clone)]
    pub struct RoutingOptions {
        pub profile: u8,
        pub algorithm: u8,
        /// 0 = forward, 1 = backward (reverse search).
        pub direction: u8,
        /// OSR's generalized maximum cost (not necessarily elapsed seconds).
        pub max_cost: u32,
        pub max_match_distance_meters: f64,
        /// > 0 overrides the default for walking/wheelchair profiles.
        pub foot_speed_meters_per_second: f32,
        pub has_start_time: bool,
        pub start_time_unix_seconds: i64,
        /// One-to-many: reconstruct each path and its segments when true.
        pub include_segments: bool,
    }

    #[derive(Debug, Clone)]
    pub struct PathSegment {
        pub osm_way_id: i64,
        pub has_osm_way_id: bool,
        pub from_node: u32,
        pub to_node: u32,
        pub mode: u8,
        pub from_level: f32,
        pub to_level: f32,
        pub cost: u32,
        pub duration_seconds: u32,
        pub distance_meters: u32,
        pub elevation_up: u32,
        pub elevation_down: u32,
        /// Empty when reconstruction is disabled.
        pub geometry: Vec<Coordinate>,
    }

    #[derive(Debug, Clone)]
    pub struct RouteResult {
        /// false = no path within search cost/matching constraints.
        pub found: bool,
        pub cost: u32,
        pub duration_seconds: u32,
        /// For one-to-many cost-only results this may be zero.
        pub distance_meters: f64,
        pub uses_elevator: bool,
        pub elevation_up: u32,
        pub elevation_down: u32,
        pub segments: Vec<PathSegment>,
    }

    #[derive(Debug, Clone)]
    pub struct SnapCandidate {
        pub osm_way_id: i64,
        pub distance_to_way_meters: f64,
        pub left_osm_node_id: i64,
        pub right_osm_node_id: i64,
        pub has_left_node: bool,
        pub has_right_node: bool,
        pub left_distance_meters: f64,
        pub right_distance_meters: f64,
    }

    /// Native OSR node index (u32::MAX means invalid), not an OSM node ID.
    #[derive(Debug, Clone)]
    pub struct RawNodeCandidate {
        pub node_index: u32,
        pub distance_to_node: f32,
    }

    /// Exact fields of OSR's profile-independent raw_way_candidate.
    /// Indices belong to the graph used for matching and must not be reused
    /// against a different OSR graph.
    #[derive(Debug, Clone)]
    pub struct RawWayCandidate {
        pub distance_to_way: f32,
        pub way_index: u32,
        pub left: RawNodeCandidate,
        pub right: RawNodeCandidate,
    }

    /// OSR match_result::node (the endpoint cost is profile-specific).
    #[derive(Debug, Clone)]
    pub struct MatchedNode {
        pub node_index: u32,
        pub distance_to_node: f32,
        pub cost: u32,
    }

    #[derive(Debug, Clone)]
    pub struct MatchedNodes {
        pub left: MatchedNode,
        pub right: MatchedNode,
    }

    /// Owned, inspectable struct-of-arrays copy of osr::match_result at the
    /// OSR_REV pinned by build.rs. Matches are bucketed by offsets.
    /// offsets.len() == level_codes.len() + 1; offsets[0] == 0;
    /// offsets.last() == way_indices.len() == distance_to_way.len() == nodes.len().
    /// level_codes are OSR level_t encodings (0 = unspecified, not level 0).
    /// The native graph must be identical when this structure is reused.
    #[derive(Debug, Clone)]
    pub struct RawMatchResult {
        pub offsets: Vec<u32>,
        pub level_codes: Vec<u8>,
        pub distance_to_way: Vec<f32>,
        pub way_indices: Vec<u32>,
        pub nodes: Vec<MatchedNodes>,
    }

    /// Arguments to osr::lookup::match; reverse=false for an origin,
    /// reverse=true for a destination during forward one-to-many routing.
    #[derive(Debug, Clone)]
    pub struct MatchOptions {
        pub profile: u8,
        pub direction: u8,
        pub reverse: bool,
        pub max_match_distance_meters: f64,
        pub foot_speed_meters_per_second: f32,
    }

    #[derive(Debug, Clone)]
    pub struct MapMatchResult {
        pub path: RouteResult,
        pub segment_offsets: Vec<u64>,
        pub routed_count: u32,
        pub beeline_count: u32,
        pub dijkstra_early_terminations: u32,
        pub dijkstra_full_runs: u32,
        pub computation_milliseconds: u64,
    }

    unsafe extern "C++" {
        include!("include/router.h");

        type Router;
        type MatchStore;

        fn open_router(data_dir: &str) -> Result<UniquePtr<Router>>;
        fn node_count(self: &Router) -> u64;
        fn way_count(self: &Router) -> u64;
        fn has_elevations(self: &Router) -> bool;

        /// All native search profiles, both single-destination algorithms,
        /// forward/backward direction, level, time and matching options.
        fn one_to_one(
            self: &Router,
            from: &Coordinate,
            to: &Coordinate,
            options: &RoutingOptions,
            blocked_osm_nodes: &[i64],
        ) -> Result<RouteResult>;

        /// Actual native one-to-many Dijkstra; never a loop of single searches.
        /// The result order matches the destinations slice.
        fn one_to_many(
            self: &Router,
            from: &Coordinate,
            destinations: &[Coordinate],
            options: &RoutingOptions,
            blocked_osm_nodes: &[i64],
        ) -> Result<Vec<RouteResult>>;

        /// Raw coordinate-to-way candidates, with optional OSR search expansion.
        fn snap(
            self: &Router,
            point: &Coordinate,
            max_match_distance_meters: f64,
            expand_search: bool,
        ) -> Result<Vec<SnapCandidate>>;

        /// OSR lookup::get_raw_match: reusable profile-independent way/node
        /// candidates, without converting graph indexes to OSM IDs.
        fn get_raw_match(
            self: &Router,
            point: &Coordinate,
            max_match_distance_meters: f64,
        ) -> Result<Vec<RawWayCandidate>>;

        /// OSR lookup::match: resolve profile, direction, level and costs.
        /// Empty raw_candidates asks OSR to perform geometric matching.
        /// Returns exactly one match bucket, even when it has zero candidates.
        fn lookup_match(
            self: &Router,
            point: &Coordinate,
            options: &MatchOptions,
            raw_candidates: &[RawWayCandidate],
            blocked_osm_nodes: &[i64],
        ) -> Result<RawMatchResult>;

        /// Exact pre-matched OSR route_one_to_many overload. Origin matches
        /// must have one bucket, destination matches one per destination.
        /// The caller must supply results made for this exact OSR graph,
        /// profile, direction, parameters and endpoint role; mutable raw
        /// arrays are validated for index/bucket safety on C++ import.
        fn route_one_to_many_matched(
            self: &Router,
            origin: &Coordinate,
            destinations: &[Coordinate],
            origin_match: &RawMatchResult,
            destination_matches: &RawMatchResult,
            options: &RoutingOptions,
            blocked_osm_nodes: &[i64],
        ) -> Result<Vec<RouteResult>>;

        /// Reusable native match buckets, owned in C++ and bound to this Router.
        /// A separate store is required for origin vs destination matches.
        fn create_match_store(self: &Router) -> UniquePtr<MatchStore>;

        /// Append one lookup::match bucket to the native cache. All buckets in
        /// a store must share matching options; no blocked nodes or dynamic
        /// start times are supported in cached matches.
        fn append_lookup_match(
            self: &Router,
            store: Pin<&mut MatchStore>,
            point: &Coordinate,
            options: &MatchOptions,
            raw_candidates: &[RawWayCandidate],
        ) -> Result<u32>;

        /// Export a single cached match bucket as the public owned raw format.
        fn export_match(self: &Router, store: &MatchStore, index: u32) -> Result<RawMatchResult>;

        /// Route using cached origin/destination match buckets without
        /// repeated FFI copies of all the raw arrays or rematching.
        /// Supports static matching (has_start_time=false).
        fn route_one_to_many_cached(
            self: &Router,
            origins: &MatchStore,
            origin_index: u32,
            destinations: &MatchStore,
            destination_indices: &[u32],
            options: &RoutingOptions,
        ) -> Result<Vec<RouteResult>>;

        /// OSR's path map-matching implementation (not coordinate snapping).
        fn map_match(
            self: &Router,
            trace: &[Coordinate],
            profile: u8,
            foot_speed_meters_per_second: f32,
            blocked_osm_nodes: &[i64],
        ) -> Result<MapMatchResult>;
    }
}

/// Numeric codes used at the FFI boundary. The C++ adapter checks every code.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Profile {
    Foot = 0,
    Wheelchair = 1,
    Bike = 2,
    BikeFast = 3,
    BikeElevationLow = 4,
    BikeElevationHigh = 5,
    Car = 6,
    Hgv = 7,
    CarDropOff = 8,
    CarDropOffWheelchair = 9,
    CarParking = 10,
    CarParkingWheelchair = 11,
    BikeSharing = 12,
    CarSharing = 13,
    Bus = 14,
    Railway = 15,
    Ferry = 16,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Algorithm {
    Dijkstra = 0,
    BidirectionalAStar = 1,
    AStar = 2,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Forward = 0,
    Backward = 1,
}

impl ffi::Coordinate {
    pub fn outdoor(lat: f64, lon: f64) -> Self {
        Self {
            lat,
            lon,
            has_level: false,
            level: 0.0,
        }
    }

    pub fn indoor(lat: f64, lon: f64, level: f32) -> Self {
        Self {
            lat,
            lon,
            has_level: true,
            level,
        }
    }
}

impl Default for ffi::MatchOptions {
    fn default() -> Self {
        Self {
            profile: Profile::Foot as u8,
            direction: Direction::Forward as u8,
            reverse: false,
            max_match_distance_meters: 25.0,
            foot_speed_meters_per_second: 0.0,
        }
    }
}

impl ffi::RawMatchResult {
    /// Validate the struct-of-arrays shape before passing it to native code.
    /// The native adapter additionally validates levels and graph indexes.
    pub fn has_valid_shape(&self) -> bool {
        let n = self.way_indices.len();
        self.offsets.len() == self.level_codes.len() + 1
            && self.offsets.first() == Some(&0)
            && self.offsets.last().copied() == u32::try_from(n).ok()
            && self.distance_to_way.len() == n
            && self.nodes.len() == n
            && self.offsets.windows(2).all(|x| x[0] <= x[1])
    }

    pub fn match_count(&self) -> usize {
        self.level_codes.len()
    }
}

impl Default for ffi::RoutingOptions {
    fn default() -> Self {
        Self {
            profile: Profile::Foot as u8,
            algorithm: Algorithm::Dijkstra as u8,
            direction: Direction::Forward as u8,
            max_cost: 3600,
            max_match_distance_meters: 100.0,
            foot_speed_meters_per_second: 0.0,
            has_start_time: false,
            start_time_unix_seconds: 0,
            include_segments: true,
        }
    }
}

impl ffi::RoutingOptions {
    pub fn profile(mut self, value: Profile) -> Self {
        self.profile = value as u8;
        self
    }

    pub fn algorithm(mut self, value: Algorithm) -> Self {
        self.algorithm = value as u8;
        self
    }

    pub fn direction(mut self, value: Direction) -> Self {
        self.direction = value as u8;
        self
    }

    pub fn at_unix_seconds(mut self, value: i64) -> Self {
        self.has_start_time = true;
        self.start_time_unix_seconds = value;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_profiles_have_stable_distinct_codes() {
        let profiles = [
            Profile::Foot,
            Profile::Wheelchair,
            Profile::Bike,
            Profile::BikeFast,
            Profile::BikeElevationLow,
            Profile::BikeElevationHigh,
            Profile::Car,
            Profile::Hgv,
            Profile::CarDropOff,
            Profile::CarDropOffWheelchair,
            Profile::CarParking,
            Profile::CarParkingWheelchair,
            Profile::BikeSharing,
            Profile::CarSharing,
            Profile::Bus,
            Profile::Railway,
            Profile::Ferry,
        ];
        for (i, profile) in profiles.iter().enumerate() {
            assert_eq!(*profile as usize, i);
        }
    }

    #[test]
    fn raw_match_result_validates_offsets_and_column_lengths() {
        let mut m = ffi::RawMatchResult {
            offsets: vec![0, 0, 0],
            level_codes: vec![0, 33],
            distance_to_way: vec![],
            way_indices: vec![],
            nodes: vec![],
        };
        assert!(m.has_valid_shape());
        assert_eq!(m.match_count(), 2);
        m.offsets[1] = 1;
        assert!(!m.has_valid_shape());
    }

    #[test]
    fn match_defaults_use_motis_foot_radius() {
        let m = ffi::MatchOptions::default();
        assert_eq!(m.max_match_distance_meters, 25.0);
        assert_eq!(m.profile, Profile::Foot as u8);
    }

    #[test]
    fn defaults_are_bounded_and_have_geometry() {
        let o = ffi::RoutingOptions::default();
        assert_eq!(o.profile, Profile::Foot as u8);
        assert!(o.include_segments);
        assert!(o.max_cost > 0);
        assert!(o.max_match_distance_meters > 0.0);
    }
}
