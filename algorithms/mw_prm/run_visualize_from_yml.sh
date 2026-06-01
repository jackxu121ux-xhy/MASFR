#!/usr/bin/env bash
# Run visualize_pcd.py with YAML defaults (path configured below).
# Keep next to visualize_pcd.py / main_pcd.py (repo root). Requires Python 3 + Open3D (+ PyYAML if using --yml).
#
set -euo pipefail

# --- YAML under yml/visualize (repo-relative or absolute) ---
YML_REL="yml/visualize/cleaned_scans_e.yml"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${YML_REL}" = /* ]]; then
	YML_PATH="${YML_REL}"
else
	YML_PATH="${REPO_ROOT}/${YML_REL}"
fi

cd "${REPO_ROOT}"
exec python3 "${REPO_ROOT}/visualize_pcd.py" --yml "${YML_PATH}" "$@"
