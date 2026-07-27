.PHONY: install install-dev test clean

install:
	./install.sh

install-dev:
	./install.sh --dev

test:
	.venv/bin/python -m pytest

clean:
	rm -rf .pytest_cache .ruff_cache .mypy_cache build dist *.egg-info
