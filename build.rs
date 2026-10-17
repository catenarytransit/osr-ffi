use std::{
    env, fs,
    path::{Path, PathBuf},
    process::Command,
};

fn run_command(cmd: &mut Command, what: &str) {
    let status = cmd.status().unwrap_or_else(|e| panic!("{what}: {e}"));
    assert!(status.success(), "{what} failed with {status}");
}

fn resolve_source(out: &Path) -> PathBuf {
    if let Some(path) = env::var_os("OSR_SOURCE_DIR") {
        let path = PathBuf::from(path);
        assert!(path.join("CMakeLists.txt").exists(), "OSR_SOURCE_DIR must point at the OSR source root");
        return fs::canonicalize(path).expect("canonicalize OSR_SOURCE_DIR");
    }
    let path = out.join("osr-source");
    if path.join("CMakeLists.txt").exists() {
        return path;
    }
    let primary = env::var("OSR_GIT_URL")
        .unwrap_or_else(|_| "https://github.com/catenarytransit/osr.git".into());
    let fallback = "https://github.com/motis-project/osr.git";
    let clone = |url: &str| Command::new("git")
        .arg("clone").arg("--depth=1").arg("--").arg(url).arg(&path)
        .status().is_ok_and(|s| s.success());
    if !clone(&primary) {
        let _ = fs::remove_dir_all(&path);
        println!("cargo:warning=Could not clone Catenary OSR fork {primary}; using pinned upstream fallback");
        assert!(clone(fallback), "could not clone Catenary fork or upstream OSR");
        run_command(
            Command::new("git").arg("-C").arg(&path).arg("fetch")
                .arg("--depth=1").arg("origin")
                .arg("d53c174408a859f29be7bd5cfbf176cf9649cd06"),
            "fetch pinned upstream OSR commit",
        );
        run_command(
            Command::new("git").arg("-C").arg(&path).arg("checkout")
                .arg("--detach").arg("d53c174408a859f29be7bd5cfbf176cf9649cd06"),
            "checkout pinned upstream OSR commit",
        );
    } else if let Ok(rev) = env::var("OSR_REV") {
        run_command(
            Command::new("git").arg("-C").arg(&path).arg("fetch")
                .arg("--depth=1").arg("origin").arg(&rev),
            "fetch requested Catenary OSR revision",
        );
        run_command(
            Command::new("git").arg("-C").arg(&path).arg("checkout")
                .arg("--detach").arg(&rev),
            "checkout requested Catenary OSR revision",
        );
    }
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
    fs::write(&project, "if(PROJECT_NAME STREQUAL osr AND CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)\\n  cmake_language(DEFER CALL include ${OSR_FFI_LATE_INCLUDE})\\nendif()\\n")
        .expect("project include");
    fs::write(&deferred, r#"
add_library(catenary_osr_bridge SHARED "${OSR_FFI_ROOT}/cpp/router.cc")
target_include_directories(catenary_osr_bridge PRIVATE
    "${OSR_FFI_ROOT}"
    "${OSR_FFI_CXX_INCLUDES}")
target_link_libraries(catenary_osr_bridge PRIVATE osr)
target_compile_features(catenary_osr_bridge PRIVATE cxx_std_23)
set_target_properties(catenary_osr_bridge PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${OSR_FFI_LIB_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${OSR_FFI_LIB_DIR}")
"#).expect("late cmake include");

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
        .define("OSR_MIMALLOC", "OFF");

    // OSR's .pkg pins git@github.com deps; allow unattended HTTPS builds.
    config
        .env("GIT_CONFIG_COUNT", "1")
        .env("GIT_CONFIG_KEY_0", "url.https://github.com/.insteadOf")
        .env("GIT_CONFIG_VALUE_0", "git@github.com:");
    let _ = config.build();

    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=dylib=catenary_osr_bridge");

    if cfg!(target_os = "linux") {
        // This path also needs to be deployed for downstream binaries.
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
    }
}
