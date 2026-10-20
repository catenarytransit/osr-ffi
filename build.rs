use std::{
    env, fs,
    path::{Path, PathBuf},
    process::Command,
};

fn run_command(cmd: &mut Command, what: &str) {
    let status = cmd.status().unwrap_or_else(|e| panic!("{what}: {e}"));
    assert!(status.success(), "{what} failed with {status}");
}

/// Copy a user-provided OSR checkout into OUT_DIR before removing backend-only
/// dependencies. Never edit the caller's checked-out source tree.
fn stage_external_source(source: &Path, target: &Path) {
    if target.exists() {
        fs::remove_dir_all(target).expect("remove previous staged OSR checkout");
    }
    fn copy_tree(source: &Path, target: &Path) {
        fs::create_dir_all(target).expect("create staged OSR directory");
        for entry in fs::read_dir(source).expect("read OSR source directory") {
            let entry = entry.expect("read OSR source entry");
            let name = entry.file_name();
            if matches!(name.to_str(), Some(".git" | "deps" | "build" | "target")) {
                continue;
            }
            let src = entry.path();
            let dst = target.join(name);
            if entry.file_type().expect("OSR source file type").is_dir() {
                copy_tree(&src, &dst);
            } else {
                fs::copy(&src, &dst).expect("copy staged OSR source file");
            }
        }
    }
    copy_tree(source, target);
}

/// OSR's CMakeLists unconditionally defines backend and test executables, but
/// Cargo only builds the routing library and its bridge. The [net] entry pulls
/// in LibreSSL (and a broken OpenBSD submodule), even though the router target
/// does not link to net or web-server.
fn exclude_backend_dependencies(source: &Path) {
    let pkg = source.join(".pkg");
    let input = fs::read_to_string(&pkg).expect("read pinned OSR .pkg manifest");
    let mut filtered = String::with_capacity(input.len());
    let mut skip = false;
    for line in input.split_inclusive('\n') {
        let trimmed = line.trim();
        if trimmed.starts_with('[') && trimmed.ends_with(']') {
            skip = trimmed == "[net]";
        }
        if !skip {
            filtered.push_str(line);
        }
    }
    // The upstream OSR lockfile already pins this dependency, but its root
    // .pkg does not list it. Without the date-tz target, both date/tz.h
    // and date/iso_week.h are missing when OSR's library is compiled.
    const DATE_DEP: &str = "[date]\n  url=git@github.com:motis-project/date.git\n  branch=master\n  commit=ce88cc33b5551f66655614eeebb7c5b7189025fb\n";
    if !filtered.contains("[date]") {
        if !filtered.ends_with('\n') {
            filtered.push('\n');
        }
        filtered.push_str(DATE_DEP);
    }
    if filtered != input {
        fs::write(pkg, filtered).expect("write FFI-only OSR .pkg manifest");
        println!(
            "cargo:warning=Using routing-only OSR dependency manifest, with pinned date-tz and without net/LibreSSL"
        );
    }
}

fn resolve_source(out: &Path) -> PathBuf {
    if let Some(dir) = env::var_os("OSR_SOURCE_DIR") {
        let path = fs::canonicalize(&dir).expect("canonicalize OSR_SOURCE_DIR");
        assert!(
            path.join("CMakeLists.txt").is_file()
                && path.join("include/osr/routing/route.h").is_file(),
            "OSR_SOURCE_DIR must point to the root of a MOTIS OSR checkout"
        );
        println!(
            "cargo:rerun-if-changed={}",
            path.join("CMakeLists.txt").display()
        );
        println!("cargo:rerun-if-changed={}", path.join(".pkg").display());
        let staged = out.join("osr-source");
        stage_external_source(&path, &staged);
        exclude_backend_dependencies(&staged);
        return staged;
    }

    // Keep the import deterministic: never silently build the tip of master.
    const UPSTREAM: &str = "https://github.com/motis-project/osr.git";
    const PINNED: &str = "d53c174408a859f29be7bd5cfbf176cf9649cd06";
    let url = env::var("OSR_GIT_URL").unwrap_or_else(|_| UPSTREAM.into());
    let revision = env::var("OSR_REV").unwrap_or_else(|_| PINNED.into());
    let path = out.join("osr-source");

    if !path.join(".git").exists() {
        if path.exists() {
            fs::remove_dir_all(&path).expect("remove incomplete OSR checkout");
        }
        run_command(
            Command::new("git").arg("init").arg(&path),
            "initialize OSR source",
        );
        run_command(
            Command::new("git")
                .arg("-C")
                .arg(&path)
                .arg("remote")
                .arg("add")
                .arg("origin")
                .arg(&url),
            "set OSR origin",
        );
    }

    // This also repairs an interrupted clone or an OSR_REV change.
    run_command(
        Command::new("git")
            .arg("-C")
            .arg(&path)
            .arg("remote")
            .arg("set-url")
            .arg("origin")
            .arg(&url),
        "update OSR origin",
    );
    let actual = Command::new("git")
        .arg("-C")
        .arg(&path)
        .args(["rev-parse", "HEAD"])
        .output();
    let at_requested_revision = actual
        .ok()
        .filter(|o| o.status.success())
        .is_some_and(|o| String::from_utf8_lossy(&o.stdout).trim() == revision);
    if !at_requested_revision {
        run_command(
            Command::new("git")
                .arg("-C")
                .arg(&path)
                .arg("fetch")
                .arg("--depth=1")
                .arg("origin")
                .arg(&revision),
            "fetch pinned OSR source",
        );
        run_command(
            Command::new("git")
                .arg("-C")
                .arg(&path)
                .arg("checkout")
                .arg("--force")
                .arg("--detach")
                .arg("FETCH_HEAD"),
            "checkout pinned OSR source",
        );
    }
    assert!(
        path.join("CMakeLists.txt").is_file(),
        "OSR checkout missing CMakeLists.txt"
    );
    exclude_backend_dependencies(&path);
    path
}

fn main() {
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=include/router.h");
    println!("cargo:rerun-if-changed=cpp/router.cc");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=OSR_SOURCE_DIR");
    println!("cargo:rerun-if-env-changed=OSR_GIT_URL");
    println!("cargo:rerun-if-env-changed=OSR_REV");
    println!("cargo:rerun-if-env-changed=OSR_BUILD_JOBS");

    // Generate the header before CMake compiles router.cc.
    let mut bridge = cxx_build::bridge("src/lib.rs");
    bridge.include(".").std("c++23").compile("osr_ffi_cxx_glue");

    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    let root = fs::canonicalize(env::var("CARGO_MANIFEST_DIR").expect("manifest dir"))
        .expect("canonicalize crate");
    let source = resolve_source(&out);
    let include_dir = out.join("cxxbridge").join("include");
    let lib_dir = out.join("native-lib");
    fs::create_dir_all(&lib_dir).expect("create native library output");
    let deferred = out.join("catenary-late.cmake");
    let project = out.join("catenary-project.cmake");

    // OSR must remain the top-level CMake project: it initializes its own
    // pinned pkg dependencies from cmake/pkg.cmake. Defer the extra target
    // until after OSR's CMakeLists.txt defines `osr`.
    // Raw string with REAL newlines. CMake cannot parse backslash-n text.
    // The guard prevents OSR's dependency projects from registering the hook.
    fs::write(
        &project,
        r#"
if(PROJECT_NAME STREQUAL "osr" AND CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  cmake_language(DEFER CALL include "${OSR_FFI_LATE_INCLUDE}")
endif()
"#,
    )
    .expect("write CMAKE_PROJECT_INCLUDE hook");
    fs::write(
        &deferred,
        r#"
# Pinned motis-project/osm 2311ac6 predates stricter GCC C++23 checks.
# These changes affect only Cargo's checked-out native dependency.
function(osr_ffi_fix_osm relative_path previous corrected)
  set(path "${CMAKE_SOURCE_DIR}/deps/osm/${relative_path}")
  if(NOT EXISTS "${path}")
    message(FATAL_ERROR "Pinned OSM dependency missing: ${path}")
  endif()
  file(READ "${path}" source)
  string(FIND "${source}" "${previous}" position)
  if(position EQUAL -1)
    string(FIND "${source}" "${corrected}" already_fixed)
    if(already_fixed EQUAL -1)
      message(FATAL_ERROR "Unexpected pinned OSM source in ${path}")
    endif()
    return()
  endif()
  string(REPLACE "${previous}" "${corrected}" updated "${source}")
  file(WRITE "${path}" "${updated}")
endfunction()

osr_ffi_fix_osm("include/osm/assembler/vec.h"
    "  constexpr explicit vec(const node_ref& nr) noexcept"
    "  explicit vec(const node_ref& nr) noexcept")
osr_ffi_fix_osm("include/osm/assembler/assembler_types.h"
    "      location location;"
    "      osm::location location;")
osr_ffi_fix_osm("include/osm/assembler/assembler_types.h"
    "  location location;"
    "  osm::location location;")
osr_ffi_fix_osm("include/osm/assembler/assembler_types.h"
    "  location_to_ring_map(location l,"
    "  location_to_ring_map(osm::location l,")
osr_ffi_fix_osm("include/osm/assembler/assembler_types.h"
    "  explicit location_to_ring_map(location l) noexcept"
    "  explicit location_to_ring_map(osm::location l) noexcept")
osr_ffi_fix_osm("include/osm/assembler/assembler_types.h"
    "  location location(const segment_list& segment_list) const noexcept"
    "  osm::location location(const segment_list& segment_list) const noexcept")
osr_ffi_fix_osm("include/osm/assembler/assembler_types.h"
    "  const node_ref& node_ref(const segment_list& segment_list) const noexcept"
    "  const osm::node_ref& node_ref(const segment_list& segment_list) const noexcept")
osr_ffi_fix_osm("include/osm/assembler/state.h"
    "  segment_list segment_list{};"
    "  osm::segment_list segment_list{};")
osr_ffi_fix_osm("include/osm/assembler/state.h"
    "  problem_reporter problem_reporter;"
    "  osm::problem_reporter problem_reporter;")

add_library(catenary_osr_bridge SHARED "${OSR_FFI_ROOT}/cpp/router.cc")
target_include_directories(catenary_osr_bridge PRIVATE
    "${OSR_FFI_ROOT}"
    "${OSR_FFI_CXX_INCLUDES}")
target_link_libraries(catenary_osr_bridge PRIVATE osr)
target_compile_features(catenary_osr_bridge PRIVATE cxx_std_23)
set_target_properties(catenary_osr_bridge PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${OSR_FFI_LIB_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${OSR_FFI_LIB_DIR}")
"#,
    )
    .expect("late cmake include");

    let mut config = cmake::Config::new(&source);
    config
        .build_target("catenary_osr_bridge")
        .define("CMAKE_PROJECT_INCLUDE", &project)
        .define("OSR_FFI_LATE_INCLUDE", &deferred)
        .define("OSR_FFI_ROOT", &root)
        .define("OSR_FFI_CXX_INCLUDES", &include_dir)
        .define("OSR_FFI_LIB_DIR", &lib_dir)
        .define("CMAKE_POSITION_INDEPENDENT_CODE", "ON")
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("OSR_MIMALLOC", "OFF")
        .define("CMAKE_EXPORT_COMPILE_COMMANDS", "ON")
        .define("CMAKE_CXX_FLAGS_DEBUG", "-O0 -g0")
        .define("CMAKE_C_FLAGS_DEBUG", "-O0 -g0");

    // Large C++23 templates can exhaust RAM under unbounded Make/CMake -j.
    // Default to two native workers; callers may tune OSR_BUILD_JOBS.
    let native_jobs = env::var("OSR_BUILD_JOBS").unwrap_or_else(|_| "2".to_owned());
    let parsed_jobs = native_jobs
        .parse::<usize>()
        .expect("OSR_BUILD_JOBS must be a positive integer");
    assert!(
        (1..=64).contains(&parsed_jobs),
        "OSR_BUILD_JOBS must be between 1 and 64"
    );
    println!("cargo:warning=Building OSR with {parsed_jobs} native jobs (-O0 -g0 in Debug)");
    config.env("CMAKE_BUILD_PARALLEL_LEVEL", &native_jobs);

    // OSR's .pkg pins git@github.com dependencies. CI must clone over HTTPS.
    // CMake's pkg download and source dependency fetch run as subprocesses.
    config
        .env("GIT_CONFIG_COUNT", "1")
        .env("GIT_CONFIG_KEY_0", "url.https://github.com/.insteadOf")
        .env("GIT_CONFIG_VALUE_0", "git@github.com:");
    let _ = config.build();

    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=dylib=catenary_osr_bridge");

    // A dependency's rustc-link-arg does NOT propagate to its reverse dependencies.
    // Cargo makes this metadata available to a direct dependent's build.rs as
    // DEP_CATENARY_OSR_BRIDGE_LIB_DIR, letting the final binary emit its own RUNPATH.
    // This is deliberately the directory of the completed .so, not a CMake build dir.
    println!("cargo::metadata=lib_dir={}", lib_dir.display());

    if cfg!(target_os = "linux") {
        // Useful only for examples/tests/binaries built *in this package*.
        // Downstream binaries must set their own rpath using the metadata above.
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
    }
}
