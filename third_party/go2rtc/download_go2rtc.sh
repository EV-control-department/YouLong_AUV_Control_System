#!/usr/bin/env bash
set -eu

third_party_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "$(uname -m)" in
    x86_64|amd64)
        asset="go2rtc_linux_amd64"
        ;;
    aarch64|arm64)
        asset="go2rtc_linux_arm64"
        ;;
    armv7l|armv7)
        asset="go2rtc_linux_armv7"
        ;;
    *)
        echo "Unsupported CPU architecture: $(uname -m)" >&2
        exit 1
        ;;
esac

url="https://github.com/AlexxIT/go2rtc/releases/latest/download/${asset}"
output="${third_party_dir}/go2rtc"

echo "Downloading ${asset} to ${output}"
if command -v curl >/dev/null 2>&1; then
    curl -fL --retry 3 "${url}" -o "${output}"
elif command -v wget >/dev/null 2>&1; then
    wget -O "${output}" "${url}"
else
    echo "curl or wget is required" >&2
    exit 1
fi

chmod +x "${output}"
echo "Installed: ${output}"
"${output}" -version
