# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Line follower v2 robot firmware for ESP32. The robot uses N20 geared motors (1:30 ratio), a 10-sensor IR line array, MPU6050 IMU, and an IR range sensor on an 8×10cm custom PCB.

There are two parallel implementations:
- **C++ (root)** — PlatformIO + ESP-IDF, ~80% complete; has control algorithms and line-following state machine
- **Rust (`line_follower-rs/`)** — in-progress rewrite using `esp-hal` directly (no ESP-IDF); currently only motor HAL is implemented

## Build & Flash Commands

```bash
# Setup (one-time)
python -m venv venv
. venv/bin/activate
pip install platformio

# Build for ESP32
make build

# Flash to device (default port: /dev/ttyUSB1)
make upload
make upload serial=/dev/ttyUSBx   # specify port

# Serial monitor at 115200 baud
make terminal

# Run native unit tests
make test

# Clean build artifacts
make clean
```

All `make` targets activate the virtualenv automatically. `platformio test -e native` runs tests on the host via the native environment.

## Architecture

The codebase uses a layered HAL abstraction:

```
src/main.cpp
    └─ lib/LF/         (control algorithms)
        └─ lib/RobotHal/   (ESP32 hardware drivers, selected at compile time)
            or lib/MockHal/ (stub HAL used by native test builds)
```

HAL selection is done via preprocessor in `lib/LF/include/Hal.h` — it includes `RobotHal/Hal.h` for target builds and `MockHal/Hal.h` for native/test builds.

### Key Libraries

**`lib/RobotHal/`** — ESP32 hardware drivers:
- `Hal.h` — singleton coordinating all hardware; initialize first in `main.cpp`
- `Motors.h` — MCPWM-based DC motor PWM + pulse-counter encoders (25 kHz, ~40mm wheels)
- `LineSensor.h` — 10-sensor charlieplexed IR array with ADC, ambient light suppression, calibration
- `Mpu6050.h` — I2C IMU (accel + gyro)
- `Button.h` — interrupt + debounce (50 ms), press/release callbacks
- `PeriodicTask.h` — FreeRTOS mutex-protected periodic task template
- `Pins.h` — all GPIO pin assignments

**`lib/LF/`** — control logic:
- `EncoderObserver` — least-squares velocity/acceleration estimator from encoder ticks (adaptive window 2–16 samples)
- `MotorVelocityControler` — PID velocity control, 10 ms update period
- `Pid.h` — generic PID template
- `behaviors/line_follower.h` — state machine states: PreStart → FollowingLine → AvoidingObstacle → Bridging → Stopped

**`lib/MockHal/`** — minimal stub HAL + `std::thread`-based PeriodicTask for native unit tests

**`lib/Behavior/`** — generic state machine base classes (`Behavior.h`, `StateTransition`)

### Pin Assignments (from `Pins.h`)

| Function | Pins |
|---|---|
| Motors L/R (PWM) | 13,4 / 23,19 |
| Encoders L/R | 16,17 / 36,39 |
| Line sensor ADC | 33,14,35,25,34 |
| Line LEDs (charlieplex) | 27,32,26 |
| Distance sensor (IR) | 12 |
| I2C (MPU6050) | SCL=22, SDA=21 |
| Buttons (Deck/Boot) | 5 / 0 |
| Battery sense | 15 |
| Indicator LED | 2 |

> **GPIO 12 conflict:** Pin 12 (MTDI strapping pin) is used by the distance sensor. This requires permanently setting VDD_SDIO to 3.3V via `espfuse.py` (see `docs/instructions.md`). This is irreversible.

## Testing

Tests live in `test/` and use the **doctest** framework. They run on the native platform with UBSan enabled (`-fsanitize=undefined`). The main test file is `test/EncoderObserver.cpp`, which uses a `Simulator` class to drive velocity profiles and validate observer accuracy.

```bash
make test   # equivalent to: platformio test -e native
```

## Notable Design Patterns

- **Singleton HAL:** `Hal::get()` returns the hardware singleton; MockHal provides the same interface for tests.
- **Periodic tasks:** Template `PeriodicTask<Data>` wraps a callback + mutex; FreeRTOS on target, `std::thread` on native.
- **C++17 throughout:** Both `esp32` and `native` environments enforce `-std=c++17 -Wall -Wextra -Werror` (Werror only for `src/`).

## Coding style
- Self-documenting code: clear naming, minimal comments (only where logic
  isn't self-evident).
- Focus on testability, HAL layer will later solve for plugging into a simulator.

## Unit testing guidelines

Tests cover pure control-logic — anything that doesn't touch hardware directly. Hardware-dependent code is excluded; use the mock/stub HAL layer to satisfy interface dependencies when a class under test requires it.

**What to test:** Algorithms, estimators, PID controllers, state machines, and any numerical logic in the control libraries.

- One test case per logical scenario; use sub-cases/parameterized variants to vary a single parameter (e.g., different velocity values) rather than duplicating test bodies.
- Include edge cases that probe numerical limits: overflow, zero input, sudden stops, sign changes.
- Make sure the tested behavior is actually intended and not just an implementation detail.

**Assertions:** Prefer hard-failing assertions (stop on first failure) for invariants that make further checks meaningless; use accumulating/soft assertions when collecting multiple failures in one run is more informative.

## General guidelines

Keep this file up to date in case of relevant changes.

## Rust Rewrite (`line_follower-rs/`)

Uses `esp-hal` v1.0 directly (no ESP-IDF/FreeRTOS abstraction) with the Xtensa ESP toolchain.

### Build & flash
```bash
cd line_follower-rs
cargo build --release
# Flash (configured as cargo runner via .cargo/config.toml):
cargo run --release --bin motor_test
cargo run --release --bin line_follower
```
Target: `xtensa-esp32-none-elf`. Toolchain is pinned in `rust-toolchain.toml` (`esp` channel). CI runs `cargo build --release`, `cargo fmt`, and `cargo clippy -- -D warnings`.

### Structure
- `src/bin/main.rs` — main binary (skeleton: init, Wi-Fi/BLE stubs)
- `src/bin/motor_test.rs` — motor validation binary
- `src/hal/motors.rs` — MCPWM + PCNT motor/encoder HAL (mirrors C++ `Motors.h` pin assignments exactly); `encoders()` intentionally returns raw `i16` hardware counts (no accumulation) — callers use `wrapping_sub` for velocity deltas

### General

After finishing a change run the tests (`cargo test`) and format the code (`cargo fmt`).
`cargo clippy -W pedantic` is a good source of ideas to consider, but should not be considered authoritative.

Don't mention the C++ version in the rust code, it is intended to be completely standalone.

