//! Opt-in full-Switzerland integration benchmark. Requires an osr-extract dataset.
use osr_ffi::{ffi, Algorithm, Profile};
use std::{env, fmt::Write as _, fs, time::Instant};

struct Route {
    city: &'static str,
    name: &'static str,
    from: (f64,f64),
    to: (f64,f64),
    alpine: bool,
}

const ROUTES: &[Route] = &[
    Route { city:"Zurich", name:"HB to Grossmunster", from:(47.3782,8.5402), to:(47.3701,8.5440), alpine:false },
    Route { city:"Geneva", name:"Cornavin to Jardin Anglais", from:(46.2101,6.1425), to:(46.2044,6.1518), alpine:false },
    Route { city:"Basel", name:"SBB to Marktplatz", from:(47.5474,7.5896), to:(47.5594,7.5884), alpine:false },
    Route { city:"Bern", name:"Bundesplatz to Barengraben", from:(46.9476,7.4431), to:(46.9480,7.4597), alpine:false },
    Route { city:"Lausanne", name:"Gare to Cathedral", from:(46.5168,6.6292), to:(46.5227,6.6358), alpine:false },
    Route { city:"Lucerne", name:"Bahnhof to Lion Monument", from:(47.0497,8.3105), to:(47.0584,8.3105), alpine:false },
    Route { city:"St Gallen", name:"Bahnhof to Abbey", from:(47.4231,9.3694), to:(47.4231,9.3766), alpine:false },
    Route { city:"Lugano", name:"Station to Parco Ciani", from:(46.0049,8.9479), to:(46.0068,8.9585), alpine:false },
    Route { city:"Winterthur", name:"Bahnhof to Stadthaus", from:(47.5014,8.7249), to:(47.4994,8.7296), alpine:false },
    Route { city:"Biel", name:"Bahnhof to Altstadt", from:(47.1328,7.2429), to:(47.1404,7.2455), alpine:false },
    Route { city:"Fribourg", name:"Gare to St Nicholas", from:(46.8032,7.1516), to:(46.8053,7.1633), alpine:false },
    Route { city:"Chur", name:"Bahnhof to Cathedral", from:(46.8535,9.5290), to:(46.8486,9.5357), alpine:false },
    Route { city:"Interlaken", name:"Ost to Hohematte", from:(46.6905,7.8697), to:(46.6862,7.8584), alpine:false },
    Route { city:"Grindelwald", name:"Bahnhof to First gondola", from:(46.6242,8.0347), to:(46.6281,8.0409), alpine:true },
    Route { city:"Lauterbrunnen", name:"Station to Staubbach trail", from:(46.5980,7.9091), to:(46.5930,7.9077), alpine:true },
    Route { city:"Wengen", name:"Station to village church", from:(46.6054,7.9214), to:(46.6067,7.9206), alpine:true },
    Route { city:"Jungfraujoch", name:"Station to Sphinx", from:(46.5474,7.9851), to:(46.5476,7.9857), alpine:true },
    Route { city:"Kleine Scheidegg", name:"Station to mountain hotel", from:(46.5851,7.9605), to:(46.5858,7.9601), alpine:true },
];

fn memory_kb(label: &str) -> Option<u64> {
    fs::read_to_string("/proc/self/smaps_rollup").ok()?.lines().find_map(|line| {
        let (key, value) = line.split_once(':')?;
        if key == label { value.split_whitespace().next()?.parse().ok() } else { None }
    })
}

fn percentile(us: &[u128], percentage: usize) -> f64 {
    us[(us.len() * percentage).div_ceil(100).saturating_sub(1)] as f64 / 1000.0
}

#[test]
fn switzerland_landmarks() {
    let Ok(dir) = env::var("SWISS_OSR_DIR") else {
        eprintln!("Skipping Switzerland benchmark (SWISS_OSR_DIR unset)");
        return;
    };
    let out = env::var("SWISS_BENCH_OUTPUT_DIR").unwrap_or_else(|_| "switzerland-results".into());
    fs::create_dir_all(&out).expect("create output directory");
    let repeats: usize = env::var("SWISS_BENCH_REPEATS").unwrap_or_else(|_| "10".into()).parse().expect("repetitions");
    assert!((3..=100).contains(&repeats));

    let t0 = Instant::now();
    let handle = ffi::open_shared_router(&dir).expect("open Switzerland graph");
    let open_ms = t0.elapsed().as_secs_f64() * 1000.0;
    let router = handle.as_ref().expect("non-null graph");
    assert!(router.node_count() > 0 && router.way_count() > 0);
    let pss_initial = memory_kb("Pss");
    let private_initial = memory_kb("Private_Dirty");

    let mut csv = String::from("city,route,profile,alpine,found,duration_seconds,first_ms,p50_ms,p95_ms,min_ms,max_ms,repetitions\n");
    let mut report = format!("# Swiss OSR benchmarks\n\nGraph open: **{open_ms:.1} ms**, {} nodes, {} ways. Warm measurements: {repeats} repetitions per route.\n\n| City | Landmark route | Mode | Found | Seconds | First ms | p50 ms | p95 ms |\n|---|---|---|---|---:|---:|---:|---:|\n", router.node_count(), router.way_count());
    let mut urban_found = 0;
    let mut urban_total = 0;
    for route in ROUTES {
        for profile in [Profile::Foot, Profile::Bike] {
            // Mountain indoor/alpine footways may not support cycle access.
            if route.alpine && profile == Profile::Bike { continue; }
            let mode = if profile == Profile::Foot { "walking" } else { "cycling" };
            let options = ffi::RoutingOptions {
                max_cost: if route.alpine { 1800 } else { 3600 },
                max_match_distance_meters: 100.0,
                include_segments: true,
                ..ffi::RoutingOptions::default().profile(profile).algorithm(Algorithm::BidirectionalAStar)
            };
            let from = ffi::Coordinate::outdoor(route.from.0, route.from.1);
            let to = ffi::Coordinate::outdoor(route.to.0, route.to.1);
            let start = Instant::now();
            let first = router.one_to_one(&from, &to, &options, &[]).unwrap_or_else(|e| panic!("{} {mode}: {e}", route.city));
            let first_ms = start.elapsed().as_secs_f64() * 1000.0;
            let mut timings = Vec::with_capacity(repeats);
            for _ in 0..repeats {
                let start = Instant::now();
                let next = router.one_to_one(&from, &to, &options, &[]).unwrap_or_else(|e| panic!("{} {mode}: {e}", route.city));
                timings.push(start.elapsed().as_micros());
                assert_eq!(first.found, next.found, "inconsistent existence: {} {mode}", route.city);
                if next.found { assert_eq!(first.duration_seconds, next.duration_seconds); }
            }
            timings.sort_unstable();
            let p50 = percentile(&timings, 50);
            let p95 = percentile(&timings, 95);
            let min = percentile(&timings, 1);
            let max = percentile(&timings, 100);
            if !route.alpine {
                urban_total += 1;
                if first.found { urban_found += 1; }
            }
            let duration = if first.found { first.duration_seconds.to_string() } else { String::new() };
            writeln!(csv, "{},{},{},{},{},{},{first_ms:.3},{p50:.3},{p95:.3},{min:.3},{max:.3},{repeats}", route.city, route.name, mode, route.alpine, first.found, duration).unwrap();
            writeln!(report, "| {} | {} | {} | {} | {} | {first_ms:.2} | {p50:.2} | {p95:.2} |", route.city, route.name, mode, first.found, duration).unwrap();
            eprintln!("{} {:7} found={} first={first_ms:.2}ms p50={p50:.2}ms p95={p95:.2}ms", route.city, mode, first.found);
        }
    }
    writeln!(report, "\nUrban routes found: **{urban_found}/{urban_total}**. Alpine paths may be legitimately unreachable, especially around Jungfraujoch.\n\nMemory (KiB): open PSS {:?}, private dirty {:?}; after PSS {:?}, private dirty {:?}.", pss_initial, private_initial, memory_kb("Pss"), memory_kb("Private_Dirty")).unwrap();
    fs::write(format!("{out}/switzerland_routes.csv"), csv).unwrap();
    fs::write(format!("{out}/switzerland_summary.md"), report).unwrap();
    assert!(urban_found * 4 >= urban_total * 3, "Less than 75% urban fixtures routed: inspect CSV");
}
