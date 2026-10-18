// Runs as the final application's build script, not as osr-ffi's build script.
fn main() {
    #[cfg(target_os = "linux")]
    {
        let native_lib_dir = std::env::var("DEP_CATENARY_OSR_BRIDGE_LIB_DIR")
            .expect("osr-ffi must export its native runtime library directory");

        println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/lib");
        println!("cargo:rustc-link-arg=-Wl,-rpath,{native_lib_dir}");
    }
}
