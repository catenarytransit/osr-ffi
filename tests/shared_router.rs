//! Concurrent read-only access to one OSR graph, with independently
//! allocated native route/matching state in every thread.

use osr_ffi::{Profile, ffi};
use std::sync::Arc;

#[test]
fn concurrent_shared_graph_queries() {
    let Ok(data_dir) = std::env::var("OSR_TEST_DATA_DIR") else {
        eprintln!("skipping: set OSR_TEST_DATA_DIR to a built OSR graph");
        return;
    };
    let shared = ffi::open_shared_router(&data_dir).expect("open native OSR graph");
    let graph = shared.as_ref().expect("non-null graph");
    let node_count = graph.node_count();
    assert!(node_count > 0);
    assert!(graph.way_count() > 0);

    let coordinate = std::env::var("OSR_TEST_LATITUDE")
        .ok()
        .zip(std::env::var("OSR_TEST_LONGITUDE").ok())
        .map(|(lat, lon)| {
            ffi::Coordinate::outdoor(
                lat.parse().expect("OSR_TEST_LATITUDE must be numeric"),
                lon.parse().expect("OSR_TEST_LONGITUDE must be numeric"),
            )
        });
    let shared = Arc::new(shared);
    std::thread::scope(|scope| {
        let mut threads = Vec::new();
        for _ in 0..8 {
            let shared = Arc::clone(&shared);
            let coordinate = coordinate.clone();
            threads.push(scope.spawn(move || {
                let router = shared.as_ref().as_ref().expect("non-null router");
                assert_eq!(router.node_count(), node_count);
                for _ in 0..16 {
                    assert_eq!(router.node_count(), node_count);
                    if let Some(ref point) = coordinate {
                        let _candidates = router.get_raw_match(point, 25.0).expect("raw matching");
                        let options = ffi::RoutingOptions {
                            max_cost: 900,
                            include_segments: false,
                            ..ffi::RoutingOptions::default().profile(Profile::Foot)
                        };
                        let matches = router.create_match_store();
                        assert!(!matches.is_null());
                        let results = router
                            .one_to_many(point, std::slice::from_ref(point), &options, &[])
                            .expect("concurrent route");
                        assert_eq!(results.len(), 1);
                    }
                }
            }));
        }
        for thread in threads {
            thread.join().expect("shared router thread panicked");
        }
    });
}
