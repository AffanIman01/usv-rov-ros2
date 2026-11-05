#!/usr/bin/env bash
# Compatibility for earlier instructions. The default environment is now Sydney.
set -euo pipefail
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec bash "$repo_root/tools/run_sim.sh" "$@"
