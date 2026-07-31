VENV_DIR ?= ../myarm_venv

.PHONY: install install-dev install-kinematics install-dev-kinematics test clean

install:
	VENV_DIR="$(VENV_DIR)" ./install.sh

install-dev:
	VENV_DIR="$(VENV_DIR)" ./install.sh --dev

install-kinematics:
	VENV_DIR="$(VENV_DIR)" ./install.sh --kinematics

install-dev-kinematics:
	VENV_DIR="$(VENV_DIR)" ./install.sh --dev-kinematics

test:
	$(VENV_DIR)/bin/python -m pytest

install-editable:
	$(VENV_DIR)/bin/python -m pip install -e '.[pycore,kinematics]'

clean:
	rm -rf .pytest_cache .ruff_cache .mypy_cache build dist *.egg-info
