"""Shared manifest resolver for the camera launch and node boundaries."""

from __future__ import annotations

from typing import Any, Mapping, Tuple

from myarm_sdk.core import load_sdk_yaml


def load_camera_manifest(path: str) -> Mapping[str, Any]:
    document = load_sdk_yaml(path)
    services = _mapping(document.get("services"), "services")
    camera = _mapping(services.get("camera"), "services.camera")
    return camera


def resolve_camera_profile(path: str, profile_name: str) -> Tuple[str, ...]:
    camera = load_camera_manifest(path)
    profiles = _mapping(camera.get("profiles"), "services.camera.profiles")
    profile = _mapping(profiles.get(profile_name), "camera profile " + profile_name)
    raw_instances = profile.get("instances")
    if not isinstance(raw_instances, list) or not all(
        isinstance(item, str) and item for item in raw_instances
    ):
        raise TypeError("camera profile instances must be a list of non-empty strings")
    if raw_instances and not bool(camera.get("enabled", False)):
        raise RuntimeError("camera capability is disabled in services.yaml")
    instances = _mapping(camera.get("instances"), "services.camera.instances")
    for instance_id in raw_instances:
        instance = _mapping(instances.get(instance_id), "camera instance " + instance_id)
        if not bool(instance.get("enabled", False)):
            raise RuntimeError("camera instance " + instance_id + " is disabled")
    return tuple(raw_instances)


def camera_instance(path: str, instance_id: str) -> Mapping[str, Any]:
    camera = load_camera_manifest(path)
    instances = _mapping(camera.get("instances"), "services.camera.instances")
    instance = _mapping(instances.get(instance_id), "camera instance " + instance_id)
    if not bool(instance.get("enabled", False)):
        raise RuntimeError("camera instance " + instance_id + " is disabled")
    return instance


def camera_ros_config(instance: Mapping[str, Any]) -> Mapping[str, Any]:
    return _mapping(instance.get("ros"), "camera ros")


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise TypeError(name + " must be a mapping")
    return value
