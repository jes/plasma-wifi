#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
cargo build --locked --release --target x86_64-unknown-linux-musl
mkdir -p dist
cp target/x86_64-unknown-linux-musl/release/plasma-wifi dist/plasma-wifi
file dist/plasma-wifi
if readelf -l dist/plasma-wifi | rg -q INTERP; then
    echo "Unexpected dynamic loader in portable build" >&2
    exit 1
fi
sha256sum dist/plasma-wifi > dist/SHA256SUMS
echo "Portable binary: dist/plasma-wifi"
