"""Minimal validation helpers for configuration-backed services."""

from typing import Any, Mapping


def require_mapping(value: Any, name: str) -> Mapping[str, Any]:
    """Return a mapping or raise a clear configuration error."""
    if not isinstance(value, dict):
        raise TypeError(f"{name} must be a mapping")
    return value


def require_enabled(config: Mapping[str, Any], name: str) -> None:
    """Reject construction of a service disabled by the shared service config."""
    if not bool(config.get("enabled", False)):
        raise RuntimeError(f"{name} service is disabled in service/config/services.yaml")
