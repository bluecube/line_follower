#![cfg_attr(not(test), no_std)]

pub mod line_detection;
pub mod velocity_controller;

/// Initialize the line follower runtime from a binary's `main`.
///
/// Runs `esp_hal::init`, builds the [`lf_hal::Hal`].
/// Evaluates to the initialised `Hal`.
#[macro_export]
macro_rules! init {
    () => {{
        let p = ::esp_hal::init(
            ::esp_hal::Config::default().with_cpu_clock(::esp_hal::clock::CpuClock::max()),
        );
        ::lf_hal::Hal::setup(p)
    }};
}
