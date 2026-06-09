# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Line follower v2 robot firmware for ESP32. The robot uses N20 geared motors (1:30 ratio), a 10-sensor IR line array, MPU6050 IMU, and an IR range sensor on an 8×10cm custom PCB.

## Firmware

Uses `esp-hal` v1.0 directly (no ESP-IDF/FreeRTOS abstraction) with the Xtensa ESP toolchain.

- After finishing a change run the tests and format the code (`cargo fmt`).
- Verify that binaries build correctly with `cargo check --bins`.
- `cargo clippy -W pedantic` is a good source of ideas to consider, but should not be considered authoritative.
- Don't mention the C++ version in the rust code, it is intended to be completely standalone.
- Don't run `cargo doc` with `--open`. It will not help you at all and it keeps popping up in my browser windows.
- Use `cargo add` for adding packages, your version information is out of date.

### Build & flash
```bash
cd firmware
cargo build --release
# Flash (configured as cargo runner via .cargo/config.toml):
cargo run --release --bin motor_test
cargo run --release --bin line_follower
```
Target: `xtensa-esp32-none-elf`. Toolchain is pinned in `rust-toolchain.toml` (`esp` channel). CI runs `cargo build --release`, `cargo fmt`, and `cargo clippy -- -D warnings`.

### Structure
- `src/` — "Behavior" part of the robot firmware. Does not touch hardware directly.
  - `ble` module — behavior-specific GATT services (NUS) and the concrete-typed `ble_task` Embassy task. `init_ble(spawner, &mut hal)` spawns the task.
  - `ble_logger` module — custom `log::Log` that writes to serial and a 1 KB ring buffer.
  - `init!(spawner)` macro — heap alloc + `esp_hal::init` + `Hal::setup` + BLE spawn + logger init in one call. Evaluates to the `Hal`. Use in `#[esp_rtos::main]` entry.
- `src/bin/` — Main binary and various testing binaries. Each calls `line_follower::init!(spawner)` at the top of `main`.
- `lf-hal/` — HAL library (`Hal` struct, coordinates all hardware access)
  - `Hal::setup(p)` — hardware + RTOS scheduler init (no heap required).
  - `Hal::init_bt_controller::<N>()` — returns `BleController<N>` (alias for `ExternalController<BleConnector<'static>, N>`). Requires a heap.
- `lf-hal-types/` — Public types used in the interface of lf-hal, to allow us to isolate the behavior API from HAL for testing.

### Main binary entry point
All binaries (main and test) use `#[esp_rtos::main]` (async Embassy entry) and `line_follower::init!(spawner)`.
Log output goes to both UART and BLE NUS TX (Nordic UART Service, UUID `6E400003-...`).
Connect with nRF Connect (Android) or `bleak` (Python/Linux) and subscribe to the TX characteristic.


### Environment
- The environment should already be set up for the compiler to work, if there are PATH problems, ask the user to source `~/export-esp.sh`.

## Unit testing guidelines

- Pure logic unit tests live under `src/` in the main crate.
- Run with stable toolchain (ESP toolchain can't compile for x86): `cargo +stable test --workspace --exclude lf-hal --target x86_64-unknown-linux-gnu` in directory `firmware/`.
    - `lf-hal` is excluded because it pulls in ESP-specific dependencies.
    - The `unit_test.sh` script runs this command.
- Tests cover pure control-logic — anything that doesn't touch hardware directly. Hardware-dependent code is excluded and tested manually using test binaries.
- Include edge cases that probe numerical limits: overflow, zero input, sudden stops, sign changes.
- Make sure the tested behavior is actually intended and not just an implementation detail.
- Use `proptest` crate to test different value inputs where it makes sense.


## General guidelines

- Keep this file up to date in case of relevant changes.
- Don't use unicode characters in the code needlessly (eg. `—`).
- Self-documenting code: clear naming, minimal inline comments (only where logic isn't self-evident). Docstrings on public items are welcome and should be kept.
- TODOs mark real outstanding work — don't remove them unless the issue is actually resolved.
