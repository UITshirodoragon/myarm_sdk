#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
VENV_DIR="${VENV_DIR:-$ROOT_DIR/../myarm_venv}"

if [[ ! -x "$VENV_DIR/bin/python" ]]; then
  "$PYTHON_BIN" -m venv --system-site-packages "$VENV_DIR"
fi
"$VENV_DIR/bin/python" -m pip install --upgrade pip

requirements_file="$ROOT_DIR/requirements/runtime.txt"
extras="pycore"

case "${1:-}" in
  "")
    ;;
  --kinematics)
    requirements_file="$ROOT_DIR/requirements/kinematics.txt"
    extras="pycore,kinematics"
    ;;
  --robot-arm)
    extras="pycore,robot-arm"
    ;;
  --robot-arm-kinematics)
    requirements_file="$ROOT_DIR/requirements/kinematics.txt"
    extras="pycore,robot-arm,kinematics"
    ;;
  --dev)
    requirements_file="$ROOT_DIR/requirements/dev.txt"
    extras="pycore,dev"
    ;;
  --dev-kinematics)
    requirements_file="$ROOT_DIR/requirements/dev.txt"
    extras="pycore,dev,kinematics"
    ;;
  --dev-robot-arm-kinematics)
    requirements_file="$ROOT_DIR/requirements/dev.txt"
    extras="pycore,dev,robot-arm,kinematics"
    ;;
  *)
    echo "Usage: $0 [--kinematics|--robot-arm|--robot-arm-kinematics|--dev|--dev-kinematics|--dev-robot-arm-kinematics]" >&2
    exit 2
    ;;
esac

"$VENV_DIR/bin/python" -m pip install -r "$requirements_file"
"$VENV_DIR/bin/python" -m pip install -e "$ROOT_DIR[$extras]"
echo "Installed dependencies in $VENV_DIR"
