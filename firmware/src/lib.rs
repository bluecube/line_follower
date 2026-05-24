#![cfg_attr(not(test), no_std)]

#[cfg(target_arch = "xtensa")]
pub mod ble;
#[cfg(target_arch = "xtensa")]
pub mod ble_logger;
pub mod line_detection;
pub mod utilities;
pub mod velocity_controller;

/// Initialize the line follower runtime from a binary's `main`.
///
/// Declares the heap regions (sized to match `memory.x`), runs `esp_hal::init`,
/// builds the [`lf_hal::Hal`], spawns the BLE task, and starts the BLE log
/// forwarder. Evaluates to the initialised `Hal`.
#[macro_export]
macro_rules! init {
    ($spawner:expr) => {{
        // Heap is required by BLE; size 98768 matches bootloader memory in memory.x.
        ::esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
        ::esp_alloc::heap_allocator!(size: 64 * 1024);
        let p = ::esp_hal::init(
            ::esp_hal::Config::default().with_cpu_clock(::esp_hal::clock::CpuClock::max()),
        );
        let mut hal = ::lf_hal::Hal::setup(p);
        $crate::ble::init_ble($spawner, &mut hal);
        $crate::ble_logger::init();
        hal
    }};
}
