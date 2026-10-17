# osr-ffi

Native Rust/C++ bridge to [MOTIS OSR](https://github.com/motis-project/osr).
It calls the original native routing code using the `cxx` crate: the OSR graph,
R-tree and elevation data remain owned and memory-mapped by C++.

## Build

Requirements: a C++23 toolchain, CMake, Git, a working linker and OSR's
native `pkg` build dependencies.

Native dependencies are compiled from source. This can take substantial time on a
first build, especially on laptops. By default the bridge uses two native build
jobs and disables bulky debug symbols for OSR's C/C++ dependencies. You can set
`OSR_BUILD_JOBS=1` on memory-constrained machines, or increase it when RAM
allows. Repeated builds reuse the Cargo/CMake output and fetched dependencies.
 For a reproducible production build, pass
a known revision via `OSR_SOURCE_DIR` or `OSR_REV`.

```sh
# Default: import pinned MOTIS OSR (d53c174408a859f29be7bd5cfbf176cf9649cd06).
cargo test

# Use an existing checkout to avoid refetching native sources:
git clone https://github.com/motis-project/osr.git ../osr
OSR_SOURCE_DIR=../osr cargo test
```

`OSR_GIT_URL` overrides the MOTIS source URL; `OSR_REV` selects
a source revision. The build checks the cached checkout's current HEAD
against the requested revision, and refetches when the revision changes.
`OSR_SOURCE_DIR` points to an externally managed checkout. Its source files are staged under Cargo's `OUT_DIR` before filtering `[net]`, so the original checkout is never modified. The build compiles OSR with its own top-level CMake
project and injects a native shared bridge target, without editing OSR files.
Dependencies are fetched through OSR's pinned `.pkg` configuration. The build excludes the root `[net]` dependency because routing does not link OSR's HTTP backend; this avoids LibreSSL's broken OpenBSD submodule. OSR's root `.pkg` omits a required `date` declaration even though its `.pkg.lock` pins it. The FFI build adds that pinned `motis-project/date` dependency so `date-tz`, `date/tz.h` and `date/iso_week.h` are available.

On Linux, the generated `libcatenary_osr_bridge.so` is under Cargo's
build `OUT_DIR/native-lib`. Deploy the shared library with consuming
applications and set the appropriate runtime library search path
(`LD_LIBRARY_PATH`, rpath or linker packaging). macOS/Windows will
similarly require platform-specific dynamic-library packaging.

## Compile an OSR dataset

```sh
osr-extract -i region.osm.pbf -o /srv/osr/region
```

The Rust bridge **opens an extracted data directory**, not raw OSM.
For cross-border routing use a single extracted, merged OSM dataset.
Multiple independent `Router` objects cannot route across their
graph boundaries without another cross-graph integration layer.

## Rust API

```rust,no_run
use osr_ffi::{ffi, Profile, Algorithm};

fn example() -> Result<(), Box<dyn std::error::Error>> {
    let router = ffi::open_router("/srv/osr/region")?;
    let graph = router.as_ref().expect("native router");
    let from = ffi::Coordinate::outdoor(33.6846, -117.8265);
    let to = ffi::Coordinate::outdoor(33.6490, -117.8380);
    let opts = ffi::RoutingOptions::default()
        .profile(Profile::Foot)
        .algorithm(Algorithm::BidirectionalAStar);
    let result = graph.one_to_one(&from, &to, &opts, &[])?;
    if result.found {
        println!("duration: {} seconds", result.duration_seconds);
    }

    // Real one-to-many Dijkstra, one search for every destination.
    let batch = ffi::RoutingOptions::default()
        .profile(Profile::Wheelchair);
    let routes = graph.one_to_many(&from, &[to], &batch, &[])?;
    println!("{} destinations", routes.len());

    // Coordinate-to-OSM-way candidates, and polyline map matching.
    let candidates = graph.snap(&from, 100.0, false)?;
    let matched = graph.map_match(&[from, to], Profile::Foot as u8, 0.0, &[])?;
    println!("{} matches; {} segments", candidates.len(), matched.path.segments.len());
    Ok(())
}
```

## Coverage and semantics

- All **17 OSR profiles** are enumerated: foot, wheelchair, four bike
  variants, car, HGV, four car parking/drop-off variants, bike/car sharing,
  bus, railway and ferry. The native adapter forwards each profile to OSR.
  Bike/car sharing require OSR mobility overlays, which this initial
  read-only bridge does **not** construct; these profiles should only
  be considered usable after their overlay prerequisites are integrated.
- **One-to-one**: native Dijkstra, direct A*, or OSR bidirectional A*/fallback, forward
  or backward direction, cost cutoff, matching radius, indoor level,
  custom walking speed, optional Unix start time (Dijkstra or direct A*), and
  OSM-node blocklist. Individual path segments preserve mode, geometry,
  cost, duration, distance, OSM way ID and elevation change.
- **One-to-many**: one native Dijkstra execution, not repeated individual
  searches. Cost-only mode (`include_segments=false`) avoids geometry
  reconstruction and can have zero distance. Result order matches targets.
- **Coordinate snapping**: native lookup with strict-radius candidates or
  the OSR fallback radius-expansion behavior. Strict mode filters the expanded lookup results to the original radius because OSR's unexpanded lookup method is private.
- **Map matching**: OSR's trace matcher, including routed/beeline counts,
  segment offsets, search diagnostics and reconstructed geometry.
- The native routing graph is opened read-only once, with optional elevation
  data discovered automatically. Errors cross the bridge as `cxx::Exception`.
- OSM node IDs in blocklists must be part of the extracted *routing nodes*;
  unknown IDs return an error. Do not pass raw OSR internal node indices.
- Search `cost` is a generalized profile weight; `duration_seconds` is
  elapsed travel duration. Do not assume these are always identical.
- `Router` is not unsafely declared `Sync`: own one per routing worker
  until concurrent access to every OSR lookup/cache has been audited.

## Tests

`cargo fmt --check && cargo test` checks bridge and profile mapping;
OSR's native test suite must also be run for routing correctness.
For deterministic integration testing, extract a small upstream OSM fixture
and compare native C++ routes and Rust bridge outputs for all profiles
supported by that fixture. In particular test unreachable pairs, one-way
turn restrictions, indoor levels, blocked routing nodes, Dijkstra vs
bidirectional routing and one-to-many cost-only vs reconstructed paths.

### Deliberate exclusions

This is a safe, stable facade over public OSR operations, **not** an
unsafe raw-C++-object binding. Internal search-state classes, sharing
overlay creation, arbitrary `Profile` template specializations, custom
`sharing_data` and unrestricted mutable `ways` access are not exposed.
High-level repeated-query precomputed `match_result` handles and
`route_one_to_many` reconstruction state are follow-on optimization APIs.
