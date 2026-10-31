#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
cargo build --locked --release --target x86_64-unknown-linux-musl
mkdir -p dist
build_binary=$(mktemp dist/.plasma-wifi.XXXXXX)
trap 'rm -f "$build_binary"' EXIT
cp target/x86_64-unknown-linux-musl/release/plasma-wifi "$build_binary"
chmod 755 "$build_binary"
mv -f "$build_binary" dist/plasma-wifi
file dist/plasma-wifi
if readelf -l dist/plasma-wifi | rg -q INTERP; then
    echo "Unexpected dynamic loader in portable build" >&2
    exit 1
fi
sha256sum dist/plasma-wifi > dist/SHA256SUMS
echo "Portable binary: dist/plasma-wifi"
