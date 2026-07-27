#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
VENV_DIR="${VENV_DIR:-$ROOT_DIR/.venv}"

"$PYTHON_BIN" -m venv "$VENV_DIR"
"$VENV_DIR/bin/python" -m pip install --upgrade pip

requirements_file="$ROOT_DIR/requirements/runtime.txt"
if [[ "${1:-}" == "--dev" ]]; then
  requirements_file="$ROOT_DIR/requirements/dev.txt"
fi

"$VENV_DIR/bin/python" -m pip install -r "$requirements_file"
echo "Installed dependencies in $VENV_DIR"
