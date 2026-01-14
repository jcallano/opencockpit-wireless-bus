#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/arduino-cli.env"

cmd=${1:-build}
shift || true

case "$cmd" in
  build)
    if [[ -n "${EXTRA_FLAGS}" ]]; then
      arduino-cli compile \
        --fqbn "${FQBN}" \
        --build-property build.extra_flags="${EXTRA_FLAGS}" \
        "${script_dir}"
    else
      arduino-cli compile \
        --fqbn "${FQBN}" \
        "${script_dir}"
    fi
    ;;
  upload)
    arduino-cli upload \
      -p "${PORT}" \
      --fqbn "${FQBN}" \
      "${script_dir}"
    ;;
  build-upload)
    if [[ -n "${EXTRA_FLAGS}" ]]; then
      arduino-cli compile \
        --fqbn "${FQBN}" \
        --build-property build.extra_flags="${EXTRA_FLAGS}" \
        "${script_dir}"
    else
      arduino-cli compile \
        --fqbn "${FQBN}" \
        "${script_dir}"
    fi
    arduino-cli upload \
      -p "${PORT}" \
      --fqbn "${FQBN}" \
      "${script_dir}"
    ;;
  *)
    echo "Usage: $0 {build|upload|build-upload}" >&2
    exit 1
    ;;
esac
