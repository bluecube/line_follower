#!/bin/bash
set -e
cargo check --bins  # Not unit tests, strictly speaking, but a useful check anyway
cargo +stable test --workspace --exclude lf-hal --target x86_64-unknown-linux-gnu
