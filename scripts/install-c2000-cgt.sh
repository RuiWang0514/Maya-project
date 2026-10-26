#!/usr/bin/env bash
# Idempotent install of the C2000 code-generation tools pinned by the CCS project.
set -euo pipefail

CGT_VERSION="${CGT_VERSION:-22.6.1.LTS}"
PREFIX="${CGT_PREFIX:-/opt/ti}"
DEST="${PREFIX}/ti-cgt-c2000_${CGT_VERSION}"
URL="https://dr-download.ti.com/software-development/ide-configuration-compiler-or-debugger/MD-xqxJ05PLfM/${CGT_VERSION}/ti_cgt_c2000_${CGT_VERSION}_linux-x64_installer.bin"

if ! command -v make >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y make
fi

if [[ ! -x "${DEST}/bin/cl2000" ]]; then
  echo "Installing C2000 CGT ${CGT_VERSION} into ${PREFIX}"
  tmp="$(mktemp -d)"
  trap 'rm -rf "${tmp}"' EXIT
  curl -fL --retry 3 --retry-delay 2 -o "${tmp}/installer.bin" "${URL}"
  chmod +x "${tmp}/installer.bin"
  sudo "${tmp}/installer.bin" --mode unattended --prefix "${PREFIX}"
  trap - EXIT
  rm -rf "${tmp}"
else
  echo "C2000 CGT ${CGT_VERSION} already installed at ${DEST}"
fi

if [[ ! -x "${DEST}/bin/cl2000" ]]; then
  echo "cl2000 was not installed at ${DEST}/bin/cl2000" >&2
  exit 1
fi

for tool in cl2000 ofd2000 nm2000 hex2000 dis2000; do
  sudo ln -sfn "${DEST}/bin/${tool}" "/usr/local/bin/${tool}"
done

"${DEST}/bin/cl2000" -version
