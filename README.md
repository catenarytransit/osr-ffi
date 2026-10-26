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

## Linux runtime linking in downstream Rust executables

The native bridge is currently a shared library (`libcatenary_osr_bridge.so`),
built in `$OUT_DIR/native-lib`. Cargo's `rustc-link-search` lets a downstream
Rust crate link it **at build time**, but does **not** install the library or
add its directory to the final binary's runtime loader search path.

The CMake shared library uses CXX's `rust::Str` and `rust::Vec` types, whose
C++ implementations come from the generated `cxx-build` archive linked into
the Rust executable. Accordingly, the shared library has intentionally unresolved
`rust::cxxbridge1` references until it is linked with the consuming Rust
executable. Do not enforce `-Wl,--no-undefined` when linking this CMake target
or try to load this shared object independently of its Rust consumer. CI checks
the final executable with eager symbol resolution (`LD_BIND_NOW=1`) and
`ldd -r`, and verifies native dependency resolution separately.

In particular, a `cargo:rustc-link-arg` emitted by **this dependency's**
`build.rs` cannot set the rpath of an unrelated binary like `osr-tester`.
Without an explicit runtime path, launching the binary outside Cargo may fail
with `libcatenary_osr_bridge.so: cannot open shared object file`.

### Application integration (Linux)

In the **consuming binary crate**, add `build.rs`:

```rust
use std::env;

fn main() {
    // Exported by osr-ffi's links="catenary_osr_bridge" build metadata.
    #[cfg(target_os = "linux")]
    {
        let dir = env::var("DEP_CATENARY_OSR_BRIDGE_LIB_DIR")
            .expect("osr-ffi must be a direct dependency of this crate");

        // Relocatable production layout: <binary dir>/lib/libcatenary_osr_bridge.so.
        println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/lib");
        // Local development: work with Cargo's prebuilt native library.
        println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
    }
}
```

This is executable-specific: `cargo::metadata=lib_dir=...` above makes the
path available to the consumer's `build.rs` as
`DEP_CATENARY_OSR_BRIDGE_LIB_DIR`. Cargo does not forward rpath linker
arguments automatically from library dependencies to binary crates. Keep
`osr-ffi` as a direct dependency.

Run your binary normally after building:

```sh
cargo build --release
./target/release/your-binary
readelf -d target/release/your-binary | grep -E 'RUNPATH|RPATH'
```

For deployment, copy `libcatenary_osr_bridge.so` into `lib/` next to
the executable and ship **both**. The `$ORIGIN/lib` runtime path then
works independently of Cargo's build directory. Other native dynamic
dependencies, if any, must also be packaged or installed appropriately.

The regression fixture under `tests/downstream-consumer` checks a standalone
downstream executable **without** `LD_LIBRARY_PATH` (rather than `cargo run`,
which can modify the dynamic-loader environment).

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

## Share a single graph across query threads

Use `open_shared_router` to eagerly open a region once at startup, then clone
the handle for each worker. The C++ graph is shared; each query still creates
its own temporary search state and `MatchStore` objects. Never share a mutable
match store between simultaneous queries.

```rust,no_run
use osr_ffi::ffi;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let router = ffi::open_shared_router("/srv/osr/region")?;
    let threads: Vec<_> = (0..3).map(|_| {
        let graph = router.clone(); // cheap C++ shared_ptr copy, not a graph copy
        std::thread::spawn(move || {
            let graph = graph.as_ref().expect("non-null shared router");
            graph.node_count() // routing methods can also run concurrently
        })
    }).collect();
    for thread in threads {
        println!("{} nodes", thread.join().expect("worker panicked"));
    }
    Ok(())
}
```

Optional integration coverage: set `OSR_TEST_DATA_DIR` to an extracted graph
directory and run `cargo test --test shared_router`. For meaningful routing
coverage, set `OSR_TEST_LATITUDE` and `OSR_TEST_LONGITUDE` to a coordinate
inside that graph. Without a dataset the concurrency integration test skips.

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
- `open_shared_router` returns `cxx::SharedPtr<ffi::Router>` for concurrent
  workers: opening one graph per region and cloning the shared pointer does
  not clone the graph. `open_router` remains available for existing callers.
- `Router` is `Send + Sync` under the **read-only query contract**: the C++
  facade contains only const query methods; graph, lookup, and elevation
  mappings are READ-only; match stores and Dijkstra state are allocated per
  request. Any new mutable cache, native API, or OSR revision must be
  reviewed for concurrent-read safety before reuse.

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
