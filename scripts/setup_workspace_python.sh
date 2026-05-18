#!/usr/bin/env bash
set -Eeuo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${repo_root}/workspace_auv/.venv"
requirements_file="${repo_root}/workspace_sim/requirements.txt"
ai_requirements_file="${repo_root}/requirements-ai.txt"

system_python_version="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [[ -x "${venv_dir}/bin/python" ]]; then
    venv_python_version="$(${venv_dir}/bin/python -c \
        'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")' \
        2>/dev/null || true)"
    if [[ "${venv_python_version}" != "${system_python_version}" ]]; then
        echo "Removing incompatible workspace venv ${venv_python_version:-unknown} (container uses ${system_python_version})" >&2
        rm -rf -- "${venv_dir}"
    fi
fi

if [[ ! -x "${venv_dir}/bin/python" ]]; then
    python3 -m venv --system-site-packages "${venv_dir}"
fi

# The Ubuntu 20.04/Foxy image may create the venv with an old pip.  That pip
# does not recognize modern manylinux tags such as manylinux_2_28, causing the
# iceoryx2 abi3 wheel produced by maturin to be reported as unsupported.
# pip 25.0.1 is the last compatible line for Python 3.8. Do not contact
# PyPI when a sufficiently new pip is already present: some target machines
# have restricted or intercepted TLS access to pypi.org.
if ! "${venv_dir}/bin/python" - <<'PY'
import pip
from pip._vendor.packaging.version import Version
raise SystemExit(0 if Version(pip.__version__) >= Version("25.0") else 1)
PY
then
    "${venv_dir}/bin/python" -m pip install \
        --disable-pip-version-check --upgrade 'pip<25.1'
fi

"${venv_dir}/bin/python" - <<'PY'
import platform
import sys
import pip
from pip._vendor.packaging.tags import sys_tags

if sys.version_info < (3, 8):
    raise SystemExit(f"Python 3.8 or newer is required, got {sys.version}")

machine = platform.machine().lower()
if machine not in {"x86_64", "aarch64"}:
    raise SystemExit(f"unsupported wheel architecture: {machine}")

# Match the target machine as well as the interpreter and ABI. Jetson uses
# aarch64; desktop/container builds commonly use x86_64.
required_tag = f"manylinux_2_28_{machine}"
supported = {str(tag) for tag in sys_tags()}
if not any(required_tag in tag for tag in supported):
    raise SystemExit(f"pip cannot install {required_tag} wheels")

print(f"workspace pip {pip.__version__}")
print(f"{required_tag} wheel tag: supported")
PY

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
