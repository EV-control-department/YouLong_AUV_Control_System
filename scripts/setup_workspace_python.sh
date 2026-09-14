#!/usr/bin/env bash
set -Eeuo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${repo_root}/workspace_auv/.venv"
requirements_file="${repo_root}/workspace_sim/requirements.txt"
ai_requirements_file="${repo_root}/requirements-ai.txt"

if [[ ! -x "${venv_dir}/bin/python" ]]; then
    python3 -m venv --system-site-packages "${venv_dir}"
fi

"${venv_dir}/bin/python" -m pip install --disable-pip-version-check \
    --requirement "${requirements_file}"

if [[ "${INSTALL_WORKSPACE_AI:-false}" == "true" ]]; then
    "${venv_dir}/bin/python" -m pip install --disable-pip-version-check \
        --requirement "${ai_requirements_file}"
fi

"${venv_dir}/bin/python" -c \
    'import numpy; print(f"workspace NumPy {numpy.__version__}: {numpy.__file__}")'
"${venv_dir}/bin/python" -c \
    'from cv_bridge import CvBridge; print("workspace cv_bridge import: OK")'

cat <<EOF

Workspace Python runtime is ready:
  ${venv_dir}/bin/python

AI dependencies:
  $(if [[ "${INSTALL_WORKSPACE_AI:-false}" == "true" ]]; then
        printf 'installed from %s' "${ai_requirements_file}"
    else
        printf 'not installed (set INSTALL_WORKSPACE_AI=true to install)'
    fi)

sim.launch.py and the other formal bringup presets will use it automatically
when the venv exists.
EOF
