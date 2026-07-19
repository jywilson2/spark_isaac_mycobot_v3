"""MyCobot 280 M5 constrained-approach planning and validation.

Phase 0 verifies the cuRobo/CUDA runtime. Phase 1 adds explicit robot metadata,
joint-state ordering, limits, collision geometry, and independent CPU FK.
Phase 2 adds validated surface targets and deterministic task-frame goal sets.
Phase 3 adds fail-closed nominal planning through a fresh cuRobo backend for
every ``plan_grasp`` call. Phase 4 independently validates terminal geometry,
limits, dynamics, and collision clearance before granting execution
eligibility. Residual interfaces are added only after these gates pass.
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
from mycobot_curobo.planner import (
    NamedJointState,
    NominalPlan,
    NominalPlanner,
    PlannerProfile,
    PlanningFailure,
    PlanningOutcome,
    PlanningRequest,
    create_curobo_planner,
    load_planner_profile,
)
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
from mycobot_curobo.validation import (
    CuroboTrajectoryEvaluator,
    KinematicCollisionBatch,
    ValidatedPlan,
    ValidationMetrics,
    ValidationProfile,
    ValidationReport,
    ValidationViolation,
    load_validation_profile,
    validate_nominal_plan,
)
from mycobot_curobo.version_guard import (
    EnvironmentReport,
    RuntimeSnapshot,
    verify_environment,
)

__all__ = [
    "EnvironmentReport",
    "EnvironmentVerificationError",
    "ConfigurationError",
    "CuroboTrajectoryEvaluator",
    "BASE_LINK",
    "FLANGE_LINK",
    "JOINT_NAMES",
    "JointLimits",
    "KinematicCollisionBatch",
    "MyCobotCuroboError",
    "NamedJointState",
    "NominalPlan",
    "NominalPlanner",
    "Pose",
    "PlannerProfile",
    "PlanningFailure",
    "PlanningOutcome",
    "PlanningRequest",
    "RobotModelSpec",
    "RuntimeSnapshot",
    "TCP_LINK",
    "SurfaceGoalSet",
    "SurfaceTarget",
    "TaskFrameCandidate",
    "TaskFrameConfig",
    "ValidatedPlan",
    "ValidationMetrics",
    "ValidationProfile",
    "ValidationReport",
    "ValidationViolation",
    "build_surface_goal_set",
    "build_task_frame_candidates",
    "create_curobo_planner",
    "forward_kinematics",
    "load_curobo_robot_config",
    "load_planner_profile",
    "load_robot_model_spec",
    "load_validation_profile",
    "reorder_joint_state",
    "verify_environment",
    "validate_nominal_plan",
]

__version__ = "0.1.0"
