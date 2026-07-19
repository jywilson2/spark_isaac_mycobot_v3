"""MyCobot 280 M5 constrained-approach planning and validation.

Phase 0 verifies the cuRobo/CUDA runtime. Phase 1 adds explicit robot metadata,
joint-state ordering, limits, collision geometry, and independent CPU FK.
Phase 2 adds validated surface targets and deterministic task-frame goal sets.
Planning, trajectory validation, and residual interfaces are added one phase
at a time after their acceptance criteria pass.
"""

from mycobot_curobo.errors import (
    ConfigurationError,
    EnvironmentVerificationError,
    MyCobotCuroboError,
)
from mycobot_curobo.frames import (
    TaskFrameCandidate,
    TaskFrameConfig,
    build_task_frame_candidates,
)
from mycobot_curobo.goal_set import SurfaceGoalSet, build_surface_goal_set
from mycobot_curobo.robot_model import (
    BASE_LINK,
    FLANGE_LINK,
    JOINT_NAMES,
    TCP_LINK,
    JointLimits,
    Pose,
    RobotModelSpec,
    forward_kinematics,
    load_curobo_robot_config,
    load_robot_model_spec,
    reorder_joint_state,
)
from mycobot_curobo.targets import SurfaceTarget
from mycobot_curobo.version_guard import (
    EnvironmentReport,
    RuntimeSnapshot,
    verify_environment,
)

__all__ = [
    "EnvironmentReport",
    "EnvironmentVerificationError",
    "ConfigurationError",
    "BASE_LINK",
    "FLANGE_LINK",
    "JOINT_NAMES",
    "JointLimits",
    "MyCobotCuroboError",
    "Pose",
    "RobotModelSpec",
    "RuntimeSnapshot",
    "TCP_LINK",
    "SurfaceGoalSet",
    "SurfaceTarget",
    "TaskFrameCandidate",
    "TaskFrameConfig",
    "build_surface_goal_set",
    "build_task_frame_candidates",
    "forward_kinematics",
    "load_curobo_robot_config",
    "load_robot_model_spec",
    "reorder_joint_state",
    "verify_environment",
]

__version__ = "0.1.0"
