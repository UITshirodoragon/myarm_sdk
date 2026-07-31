.PHONY: install install-dev test clean

install:
	./install.sh

install-dev:
	./install.sh --dev

test:
	.venv/bin/python -m pytest

install-editable:
	python3 -m pip install -e '.[pycore]'

clean:
	rm -rf .pytest_cache .ruff_cache .mypy_cache build dist *.egg-info
