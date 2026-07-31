"""Small YAML configuration loader shared by SDK services."""

from pathlib import Path
from typing import Any, Mapping

import yaml


def sdk_root() -> Path:
    """Return the installed ``myarm_sdk`` package directory."""
    return Path(__file__).resolve().parents[1]


def resolve_sdk_path(relative_path: str) -> Path:
    """Resolve a package-relative config path without allowing directory escape."""
    root = sdk_root()
    candidate = (root / relative_path).resolve()
    if root != candidate and root not in candidate.parents:
        raise ValueError("SDK config path must stay inside myarm_sdk")
    return candidate


def load_yaml(path: Path) -> Mapping[str, Any]:
    """Load a YAML mapping and fail early on absent or malformed configuration."""
    if not path.is_file():
        raise FileNotFoundError(f"Configuration file does not exist: {path}")
    with path.open(encoding="utf-8") as config_file:
        document = yaml.safe_load(config_file)
    if not isinstance(document, dict):
        raise TypeError(f"Configuration root must be a mapping: {path}")
    return document


def load_sdk_yaml(relative_path: str) -> Mapping[str, Any]:
    """Load a YAML mapping stored as package data in ``myarm_sdk``."""
    return load_yaml(resolve_sdk_path(relative_path))
