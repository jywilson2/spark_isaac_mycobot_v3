"""Validated MyCobot 280 M5 model metadata and CPU FK reference.

Phase 1 uses this NumPy/standard-library implementation as an independent
reference for robot configuration checks. cuRobo remains the deployed GPU
kinematics/planning implementation; this module makes joint ordering, units,
limits, and the flange-to-TCP transform testable without importing CUDA.

The authoritative mesh URDF is obtained from Elephant Robotics by
``scripts/download_mycobot_ros2.sh``. See ``docs/phase1_robot_model.md`` for
the pinned source revision and assumptions.
"""

from __future__ import annotations

import copy
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml

from mycobot_curobo.errors import ConfigurationError

JOINT_NAMES: tuple[str, ...] = (
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
)
BASE_LINK = "g_base"
FLANGE_LINK = "joint6_flange"
TCP_LINK = "tcp_link"


@dataclass(frozen=True)
class JointLimits:
    """Joint limits in SI units and explicit cuRobo joint order."""

    names: tuple[str, ...]
    lower_rad: np.ndarray
    upper_rad: np.ndarray
    velocity_rad_s: np.ndarray
    acceleration_rad_s2: np.ndarray
    jerk_rad_s3: np.ndarray


@dataclass(frozen=True)
class Pose:
    """Pose in the robot base frame (meters and scalar-first ``wxyz``)."""

    position_m: np.ndarray
    quaternion_wxyz: np.ndarray


@dataclass(frozen=True)
class RobotModelSpec:
    """Phase 1 metadata resolved from cuRobo robot YAML and vendor URDF."""

    config_path: Path
    urdf_path: Path
    base_link: str
    flange_link: str
    tcp_link: str
    joint_names: tuple[str, ...]
    tool_frames: tuple[str, ...]
    default_joint_position_rad: np.ndarray
    tcp_fixed_transform_xyz_wxyz: np.ndarray
    collision_sphere_count_by_link: dict[str, int]
    limits: JointLimits
    min_detectable_obstacle_edge_m: float


@dataclass(frozen=True)
class _UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


def _as_finite_vector(value: object, length: int, label: str) -> np.ndarray:
    """Return a finite vector of the required length without clamping."""

    result = np.asarray(value, dtype=float)
    if result.shape != (length,):
        raise ConfigurationError(f"{label} must have shape ({length},), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ConfigurationError(f"{label} must contain only finite values")
    return result


def _parse_xyz(text: str | None) -> np.ndarray:
    if not text:
        return np.zeros(3, dtype=float)
    return _as_finite_vector(text.replace(",", " ").split(), 3, "URDF xyz/rpy")


def _rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _origin_matrix(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = _rpy_to_matrix(rpy)
    transform[:3, 3] = xyz
    return transform


def _axis_angle_matrix(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm <= 0.0:
        raise ConfigurationError("revolute URDF joint axis must be non-zero")
    x, y, z = axis / norm
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    one_minus_c = 1.0 - c
    return np.array(
        [
            [
                c + x * x * one_minus_c,
                x * y * one_minus_c - z * s,
                x * z * one_minus_c + y * s,
            ],
            [
                y * x * one_minus_c + z * s,
                c + y * y * one_minus_c,
                y * z * one_minus_c - x * s,
            ],
            [
                z * x * one_minus_c - y * s,
                z * y * one_minus_c + x * s,
                c + z * z * one_minus_c,
            ],
        ],
        dtype=float,
    )


def rotation_matrix_to_quaternion_wxyz(rotation: np.ndarray) -> np.ndarray:
    """Convert an orthonormal rotation matrix to a normalized quaternion."""

    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ConfigurationError("rotation must be a finite 3x3 matrix")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quaternion = np.array(
            [
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ]
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = 2.0 * math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            quaternion = np.array(
                [
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                ]
            )
        elif index == 1:
            scale = 2.0 * math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            quaternion = np.array(
                [
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                ]
            )
        else:
            scale = 2.0 * math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            quaternion = np.array(
                [
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                ]
            )
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0:
        raise ConfigurationError("rotation produced a zero quaternion")
    return quaternion / norm


def _fixed_transform_xyz_wxyz(values: object) -> np.ndarray:
    result = _as_finite_vector(values, 7, "TCP fixed_transform")
    quaternion_norm = float(np.linalg.norm(result[3:]))
    if not math.isclose(quaternion_norm, 1.0, abs_tol=1.0e-8):
        raise ConfigurationError("TCP fixed_transform quaternion must be unit length")
    return result


def _load_yaml_mapping(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ConfigurationError(f"robot config not found: {path}")
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ConfigurationError("robot config root must be a mapping")
    return payload


def _resolve_repo_path(config_path: Path, relative_path: str) -> Path:
    """Resolve project-relative paths from ``config/robots/*.yml``."""

    root = config_path.resolve().parents[2]
    return (root / relative_path).resolve()


def _resolve_overlay_path(config_path: Path, relative_path: str) -> Path:
    """Resolve a sphere overlay, preferring the real repo when tests use copies."""

    primary = _resolve_repo_path(config_path, relative_path)
    if primary.is_file():
        return primary
    for parent in config_path.resolve().parents:
        candidate = (parent / relative_path).resolve()
        package = parent / "src" / "mycobot_curobo"
        if candidate.is_file() and package.is_dir():
            return candidate
    return primary


def _parse_urdf(
    urdf_path: Path, base_link: str, flange_link: str
) -> tuple[tuple[_UrdfJoint, ...], JointLimits]:
    if not urdf_path.is_file():
        raise ConfigurationError(
            f"vendor URDF not found: {urdf_path}; run scripts/download_mycobot_ros2.sh"
        )
    root = ET.parse(urdf_path).getroot()
    joints_by_parent: dict[str, list[ET.Element]] = {}
    for element in root.findall("joint"):
        parent = element.find("parent")
        if parent is None or "link" not in parent.attrib:
            raise ConfigurationError("URDF joint is missing a parent link")
        joints_by_parent.setdefault(parent.attrib["link"], []).append(element)

    chain: list[_UrdfJoint] = []
    current = base_link
    visited: set[str] = set()
    while current != flange_link:
        if current in visited:
            raise ConfigurationError("URDF chain contains a cycle")
        visited.add(current)
        candidates = joints_by_parent.get(current, [])
        if len(candidates) != 1:
            raise ConfigurationError(
                f"expected one serial-chain child from {current!r}, found {len(candidates)}"
            )
        element = candidates[0]
        child = element.find("child")
        if child is None or "link" not in child.attrib:
            raise ConfigurationError(f"URDF joint {element.attrib.get('name')} has no child")
        origin = element.find("origin")
        axis = element.find("axis")
        chain.append(
            _UrdfJoint(
                name=element.attrib["name"],
                joint_type=element.attrib["type"],
                parent=current,
                child=child.attrib["link"],
                origin=_origin_matrix(
                    _parse_xyz(None if origin is None else origin.attrib.get("xyz")),
                    _parse_xyz(None if origin is None else origin.attrib.get("rpy")),
                ),
                axis=_parse_xyz(None if axis is None else axis.attrib.get("xyz")),
            )
        )
        current = child.attrib["link"]

    elements_by_name = {element.attrib["name"]: element for element in root.findall("joint")}
    lower: list[float] = []
    upper: list[float] = []
    velocity: list[float] = []
    for name in JOINT_NAMES:
        element = elements_by_name.get(name)
        if element is None or element.attrib.get("type") != "revolute":
            raise ConfigurationError(f"required revolute joint missing from URDF: {name}")
        limit = element.find("limit")
        if limit is None:
            raise ConfigurationError(f"joint limit missing from URDF: {name}")
        lower.append(float(limit.attrib["lower"]))
        upper.append(float(limit.attrib["upper"]))
        velocity.append(float(limit.attrib["velocity"]))

    return tuple(chain), JointLimits(
        names=JOINT_NAMES,
        lower_rad=np.asarray(lower),
        upper_rad=np.asarray(upper),
        velocity_rad_s=np.asarray(velocity),
        acceleration_rad_s2=np.empty(0),
        jerk_rad_s3=np.empty(0),
    )


def _expand_limit(value: object, label: str) -> np.ndarray:
    scalar = float(value)
    if not math.isfinite(scalar) or scalar <= 0.0:
        raise ConfigurationError(f"{label} must be a positive finite scalar")
    return np.full(len(JOINT_NAMES), scalar, dtype=float)


def apply_collision_sphere_overlay(kinematics: dict[str, Any], config_path: Path) -> float:
    """Merge Phase 1.1 sphere overlay into kinematics; return detectable edge ``E``."""

    edge_raw = kinematics.get("min_detectable_obstacle_edge_m")
    if edge_raw is None:
        raise ConfigurationError("min_detectable_obstacle_edge_m must be declared explicitly")
    edge = float(edge_raw)
    if not math.isfinite(edge) or edge <= 0.0:
        raise ConfigurationError("min_detectable_obstacle_edge_m must be positive finite")
    overlay_raw = kinematics.get("collision_sphere_overlay_path")
    if overlay_raw is None:
        return edge
    overlay_path = _resolve_overlay_path(config_path, str(overlay_raw))
    if not overlay_path.is_file():
        raise ConfigurationError(f"collision sphere overlay not found: {overlay_path}")
    overlay = _load_yaml_mapping(overlay_path)
    overlay_edge = float(overlay.get("min_detectable_obstacle_edge_m", edge))
    if not math.isfinite(overlay_edge) or overlay_edge <= 0.0:
        raise ConfigurationError("overlay min_detectable_obstacle_edge_m must be positive finite")
    if abs(overlay_edge - edge) > 1.0e-12:
        raise ConfigurationError(
            "robot min_detectable_obstacle_edge_m must match collision sphere overlay E "
            f"(robot={edge}, overlay={overlay_edge})"
        )
    spheres = overlay.get("collision_spheres")
    if not isinstance(spheres, dict) or not spheres:
        raise ConfigurationError("collision sphere overlay must define collision_spheres")
    kinematics["collision_spheres"] = spheres
    kinematics["min_detectable_obstacle_edge_m"] = overlay_edge
    return overlay_edge


def load_robot_model_spec(
    config_path: Path | str = Path("config/robots/mycobot_280_m5.yml"),
) -> RobotModelSpec:
    """Load and independently validate the Phase 1 robot configuration."""

    path = Path(config_path).resolve()
    payload = _load_yaml_mapping(path)
    try:
        kinematics = payload["robot_cfg"]["kinematics"]
        cspace = kinematics["cspace"]
    except (KeyError, TypeError) as exc:
        raise ConfigurationError("robot config must define robot_cfg.kinematics.cspace") from exc
    detectable_edge_m = apply_collision_sphere_overlay(kinematics, path)
    if float(kinematics.get("format_version", -1.0)) != 2.0:
        raise ConfigurationError("cuRobo robot config format_version must be 2.0")

    names = tuple(cspace.get("joint_names", ()))
    if names != JOINT_NAMES:
        raise ConfigurationError(f"joint_names must exactly equal {JOINT_NAMES!r}")
    if kinematics.get("grasp_contact_link_names") != []:
        raise ConfigurationError("grasp_contact_link_names must be empty in Phase 1")
    tool_frames = tuple(kinematics.get("tool_frames", ()))
    if TCP_LINK not in tool_frames:
        raise ConfigurationError(f"tool_frames must include {TCP_LINK!r}")

    extra_link = kinematics.get("extra_links", {}).get(TCP_LINK)
    if not isinstance(extra_link, dict):
        raise ConfigurationError("tcp_link must be an explicit extra fixed link")
    if extra_link.get("parent_link_name") != FLANGE_LINK:
        raise ConfigurationError(f"tcp_link parent must be {FLANGE_LINK!r}")
    if extra_link.get("joint_type") != "FIXED":
        raise ConfigurationError("tcp_link joint_type must be FIXED")
    tcp_transform = _fixed_transform_xyz_wxyz(extra_link.get("fixed_transform"))

    urdf_path = _resolve_repo_path(path, str(kinematics["urdf_path"]))
    _, urdf_limits = _parse_urdf(urdf_path, str(kinematics["base_link"]), FLANGE_LINK)
    acceleration = _expand_limit(cspace["max_acceleration"], "max_acceleration")
    jerk = _expand_limit(cspace["max_jerk"], "max_jerk")
    limits = JointLimits(
        names=urdf_limits.names,
        lower_rad=urdf_limits.lower_rad,
        upper_rad=urdf_limits.upper_rad,
        velocity_rad_s=urdf_limits.velocity_rad_s,
        acceleration_rad_s2=acceleration,
        jerk_rad_s3=jerk,
    )

    default = _as_finite_vector(
        cspace.get("default_joint_position"), len(names), "default_joint_position"
    )
    if np.any(default < limits.lower_rad) or np.any(default > limits.upper_rad):
        raise ConfigurationError("default_joint_position violates URDF position limits")

    collision_links = tuple(kinematics.get("collision_link_names", ()))
    spheres = kinematics.get("collision_spheres")
    if not isinstance(spheres, dict):
        raise ConfigurationError("collision_spheres must be an inline mapping")
    missing = [link for link in collision_links if not spheres.get(link)]
    if missing:
        raise ConfigurationError(f"collision links missing spheres: {missing}")
    counts = {str(link): len(link_spheres) for link, link_spheres in spheres.items()}

    return RobotModelSpec(
        config_path=path,
        urdf_path=urdf_path,
        base_link=str(kinematics["base_link"]),
        flange_link=FLANGE_LINK,
        tcp_link=TCP_LINK,
        joint_names=names,
        tool_frames=tool_frames,
        default_joint_position_rad=default,
        tcp_fixed_transform_xyz_wxyz=tcp_transform,
        collision_sphere_count_by_link=counts,
        limits=limits,
        min_detectable_obstacle_edge_m=detectable_edge_m,
    )


def load_curobo_robot_config(
    config_path: Path | str = Path("config/robots/mycobot_280_m5.yml"),
) -> dict[str, Any]:
    """Return cuRobo config data with external asset paths made absolute.

    cuRobo v0.8.0 resolves relative paths against its installed content
    directory, not against an external YAML file. Resolving these two fields
    before calling ``MotionPlannerCfg.create(robot=data)`` is therefore
    required for a portable project-owned robot configuration.
    """

    path = Path(config_path).resolve()
    payload = copy.deepcopy(_load_yaml_mapping(path))
    # Validate every Phase 1 invariant before handing data to CUDA code.
    load_robot_model_spec(path)
    kinematics = payload["robot_cfg"]["kinematics"]
    apply_collision_sphere_overlay(kinematics, path)
    kinematics["asset_root_path"] = str(
        _resolve_repo_path(path, str(kinematics["asset_root_path"]))
    )
    kinematics["urdf_path"] = str(_resolve_repo_path(path, str(kinematics["urdf_path"])))
    return payload


def reorder_joint_state(
    position_rad: Sequence[float],
    input_names: Sequence[str],
    expected_names: Sequence[str] = JOINT_NAMES,
) -> np.ndarray:
    """Explicitly reorder a named joint state into the expected order.

    Missing, duplicate, and unknown names are rejected. Callers that require
    exact ordering should compare names before calling this explicit adapter.
    """

    names = tuple(input_names)
    expected = tuple(expected_names)
    values = _as_finite_vector(position_rad, len(names), "joint positions")
    if len(set(names)) != len(names):
        raise ConfigurationError("joint names contain duplicates")
    missing = sorted(set(expected) - set(names))
    unknown = sorted(set(names) - set(expected))
    if missing or unknown:
        raise ConfigurationError(f"joint name mismatch: missing={missing}, unknown={unknown}")
    lookup = dict(zip(names, values, strict=True))
    return np.asarray([lookup[name] for name in expected], dtype=float)


def forward_kinematics(
    position_rad: Sequence[float],
    *,
    spec: RobotModelSpec | None = None,
) -> Pose:
    """Compute independent base-to-TCP FK for a six-joint state."""

    model_spec = load_robot_model_spec() if spec is None else spec
    q = _as_finite_vector(position_rad, len(model_spec.joint_names), "joint positions")
    if np.any(q < model_spec.limits.lower_rad) or np.any(q > model_spec.limits.upper_rad):
        raise ConfigurationError("joint positions violate configured limits")
    chain, _ = _parse_urdf(model_spec.urdf_path, model_spec.base_link, model_spec.flange_link)
    q_by_name = dict(zip(model_spec.joint_names, q, strict=True))
    transform = np.eye(4, dtype=float)
    for joint in chain:
        transform = transform @ joint.origin
        if joint.joint_type == "revolute":
            motion = np.eye(4, dtype=float)
            motion[:3, :3] = _axis_angle_matrix(joint.axis, q_by_name[joint.name])
            transform = transform @ motion
        elif joint.joint_type != "fixed":
            raise ConfigurationError(f"unsupported joint type: {joint.joint_type}")

    tcp = model_spec.tcp_fixed_transform_xyz_wxyz
    tcp_transform = np.eye(4, dtype=float)
    tcp_transform[:3, 3] = tcp[:3]
    w, x, y, z = tcp[3:]
    tcp_transform[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    transform = transform @ tcp_transform
    return Pose(
        position_m=transform[:3, 3].copy(),
        quaternion_wxyz=rotation_matrix_to_quaternion_wxyz(transform[:3, :3]),
    )
