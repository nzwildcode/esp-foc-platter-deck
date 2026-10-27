//! One monotonic microsecond clock shared by the link, the sim and the
//! audio callback, so frame age can be computed anywhere.
use std::sync::OnceLock;
use std::time::Instant;

static EPOCH: OnceLock<Instant> = OnceLock::new();

/// Microseconds since the first call in this process.
pub fn now_us() -> u64 {
    EPOCH.get_or_init(Instant::now).elapsed().as_micros() as u64
}
