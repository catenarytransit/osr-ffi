fn main() {
    // Actually invoke the C++ bridge so that --as-needed cannot remove the
    // dependency. Opening a deliberately nonexistent data directory should
    // return a C++ exception, not a loader error at process startup.
    match osr_ffi::ffi::open_router("/osr-ffi-test-graph-does-not-exist") {
        Err(_) => println!("native bridge loaded successfully"),
        Ok(_) => panic!("nonexistent OSR graph unexpectedly opened"),
    }
}
