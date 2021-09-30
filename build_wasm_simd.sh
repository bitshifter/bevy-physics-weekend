#!/bin/sh
RUSTFLAGS="-C target-feature=+simd128" cargo make --profile=release serve
