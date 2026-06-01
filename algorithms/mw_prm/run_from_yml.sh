#!/usr/bin/env bash
# Run main_pcd.py with defaults from a YAML file (path configured below).
#
# Keep this script next to main_pcd.py (repo root). Requires: Python 3, PyYAML, Open3D, SciPy.
#
set -euo pipefail

# Open3D/GLFW on Linux Wayland: use X11/XWayland path before Python starts
# (must be set before the interpreter loads Open3D).
export GLFW_PLATFORM="${GLFW_PLATFORM:-x11}"
# Fall back to Mesa software rendering if the GPU driver can't give GLEW a GL context.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

# --- configure YAML (repo-relative or absolute path) ---
YML_REL="yml/pcd/scans_260514.yml"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${YML_REL}" = /* ]]; then
	YML_PATH="${YML_REL}"
else
	YML_PATH="${REPO_ROOT}/${YML_REL}"
fi

cd "${REPO_ROOT}"
exec python3 "${REPO_ROOT}/main_pcd.py" --yml "${YML_PATH}" "$@"
