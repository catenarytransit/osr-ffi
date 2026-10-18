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
    fn defaults_are_bounded_and_have_geometry() {
        let o = ffi::RoutingOptions::default();
        assert_eq!(o.profile, Profile::Foot as u8);
        assert!(o.include_segments);
        assert!(o.max_cost > 0);
        assert!(o.max_match_distance_meters > 0.0);
    }
}
