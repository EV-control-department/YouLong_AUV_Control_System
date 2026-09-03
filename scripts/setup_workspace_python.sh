#!/usr/bin/env bash
set -Eeuo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${repo_root}/workspace_auv/.venv"
requirements_file="${repo_root}/workspace_sim/requirements.txt"

if [[ ! -x "${venv_dir}/bin/python" ]]; then
    python3 -m venv --system-site-packages "${venv_dir}"
fi

"${venv_dir}/bin/python" -m pip install --disable-pip-version-check \
    --requirement "${requirements_file}"

"${venv_dir}/bin/python" -c \
    'import numpy; print(f"workspace NumPy {numpy.__version__}: {numpy.__file__}")'
"${venv_dir}/bin/python" -c \
    'from cv_bridge import CvBridge; print("workspace cv_bridge import: OK")'

cat <<EOF

Workspace Python runtime is ready:
  ${venv_dir}/bin/python

sim_bringup.py will use it automatically when the venv exists.
EOF
