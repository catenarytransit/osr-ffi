# Low-level OSR match and routing API

This crate pins MOTIS OSR at \`d53c174408a859f29be7bd5cfbf176cf9649cd06\`.
The public \`ffi::RawNodeCandidate\`, \`ffi::RawWayCandidate\`,
\`ffi::MatchedNode\`, and \`ffi::RawMatchResult\` faithfully retain the
fields used by **that revision** of \`osr::raw_way_candidate\` and
\`osr::match_result\`. These types are owned Rust values, not borrowed
references into C++ vectors.

## Raw and profile-aware matching

\`\`\`rust
use osr_ffi::{ffi, Profile, Direction};

let router = ffi::open_router("/path/to/osr/region")?;
let point = ffi::Coordinate::outdoor(33.65, -117.83);
let raw = router.get_raw_match(&point, 250.0)?;
let opts = ffi::MatchOptions {
    profile: Profile::Foot as u8,
    direction: Direction::Forward as u8,
    reverse: false, // origin
    ..Default::default()
};
let origin_match = router.lookup_match(&point, &opts, &raw, &[])?;
assert!(origin_match.has_valid_shape());
# Ok::<(), Box<dyn std::error::Error>>(())
\`\`\`

The empty raw-candidate input means **no precomputed candidates provided**,
not "do not match": OSR will perform a lookup. Both \`get_raw_match\` and
\`lookup_match\` may expand the search radius when matching fails. They are
not strict snapping-radius enforcement APIs.

For a forward query, the origin is matched with \`reverse=false\` and the
destinations with \`reverse=true\`. The routing direction, speed, profile,
blocked nodes and time-dependent parameters must agree with how the matches
were computed. \`lookup_match\` does not currently accept start times;
do not reuse these static matches in time-dependent routing unless you have
established equivalent semantics.

The returned \`RawMatchResult\` is an OSR-style struct-of-arrays:
- \`offsets.len() == level_codes.len() + 1\`
- \`offsets[0] == 0\`
- \`offsets.last() == way_indices.len() == nodes.len() == distance_to_way.len()\`
- \`offsets[i]..offsets[i+1]\` are the candidates for location \`i\`
- \`level_codes\` are OSR-encoded indoor levels; \`0\` is **unspecified**
- all way and node indexes are **graph-local**, not OSM IDs
- a node index of \`u32::MAX\` means invalid

## Pre-matched one-to-many

\`route_one_to_many_matched(origin, destinations, origin_match,
destination_matches, routing_options, blocked_nodes)\` calls the native
pre-matched \`osr::route_one_to_many\` overload, **not** the higher-level
overload that snaps every destination. The origin match must have one bucket;
the destination matches must contain one bucket per destination in identical
order. Input shape and index ranges are checked before native OSR access.

\`RawMatchResult\` is directly editable from Rust. Thus, the caller is
responsible for ensuring that manually edited matches are semantically
consistent with the graph, profile, direction, costs, coordinates, levels,
and blocking settings. Syntactic validation cannot prove semantic validity.

## Reusable native MatchStore

\`create_match_store()\`, \`append_lookup_match()\`, \`export_match()\`,
and \`route_one_to_many_cached()\` keep an OSR \`match_result\` with many
buckets in C++, avoiding repeated Rust/C++ allocation and conversion in the
hot one-to-many loop. Use separate stores for origins and destinations:
\`MatchOptions.reverse=false\` for origin matches, \`reverse=true\` for
destination matches. Stores must be used with the Router that created them
and must not outlive that Router.

Each store is mutable during population and immutable during cached routing.
For parallel jobs, finish building the store first, and verify thread safety
of the native OSR graph and routing internals before sharing the Router.
The cached path only handles static matching and unblocked searches for now.
Use the explicit pre-matched path for blocked-node routing.

## Cache compatibility

Do not persist native graph indexes across OSR data rebuilds. Persisted
copies of these raw structures require a **content-based OSR graph identity,
OSR revision, match-role/profile/options identity and a format version**,
checked by the consuming cache implementation. This PR intentionally does
not introduce a serialization format; Gentian can add one when adopting this API.

## Upstream versions

Recent OSR \`master\` revisions added endpoint-state and penalty-reference
columns to \`match_result\`, and changed the max routing duration argument.
The conversion here targets the pinned revision only. Upgrade the OSR
revision and all corresponding field conversions together, with round-trip
and routed-parity tests.
