"""MyCobot 280 M5 constrained-approach planning and validation.

Phase 0 verifies the cuRobo/CUDA runtime. Phase 1 adds explicit robot metadata,
joint-state ordering, limits, collision geometry, and independent CPU FK.
Phase 2 adds validated surface targets and deterministic task-frame goal sets.
Phase 3 adds fail-closed nominal planning through a fresh cuRobo backend for
every ``plan_grasp`` call. Phase 4 independently validates terminal geometry,
limits, dynamics, and collision clearance before granting execution
eligibility. Phase 5 adds a dry-run execution seam with zero residual output
and deterministic safety projection; it has no hardware-driver dependency.
Phase 6 adds reproducible randomized cases, replay records, taxonomy, and
JSON/Markdown reports. Phase 7 adds an Isaac-neutral validated playback-plan
contract while keeping Kit imports outside this package.
"""

from mycobot_curobo.benchmark import (
    BenchmarkCase,
    BenchmarkConfig,
    BenchmarkResult,
    BenchmarkRunner,
    BenchmarkSummary,
    FailureCategory,
    aggregate_results,
    deserialize_request,
    load_benchmark_config,
    sample_benchmark_cases,
    serialize_request,
    write_benchmark_reports,
)
from mycobot_curobo.errors import (
    ConfigurationError,
    EnvironmentVerificationError,
    MyCobotCuroboError,
)
from mycobot_curobo.execution import (
    CpuTcpPoseEvaluator,
    ExecutionResult,
    InMemoryCommandAdapter,
    JointCommand,
    ReplayRobotStateProvider,
    RobotStateSample,
    TrajectoryExecutor,
    TrajectorySample,
    TrajectorySource,
)
from mycobot_curobo.frames import (
    TaskFrameCandidate,
    TaskFrameConfig,
    build_task_frame_candidates,
)
from mycobot_curobo.goal_set import SurfaceGoalSet, build_surface_goal_set
from mycobot_curobo.plan_io import (
    PlaybackPlan,
    load_playback_plan,
    playback_plan_from_dict,
    require_executable_plan,
    validated_plan_to_playback_dict,
    write_playback_plan,
)
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
from mycobot_curobo.residual import (
    CartesianResidual,
    ResidualCorrector,
    ResidualObservation,
    ZeroResidualCorrector,
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
from mycobot_curobo.safety import (
    ResidualSafetyProfile,
    SafetyDecision,
    SafetyProjector,
    SafetyStatus,
    load_residual_safety_profile,
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
    "ExecutionResult",
    "ConfigurationError",
    "CuroboTrajectoryEvaluator",
    "CpuTcpPoseEvaluator",
    "CartesianResidual",
    "BASE_LINK",
    "BenchmarkCase",
    "BenchmarkConfig",
    "BenchmarkResult",
    "BenchmarkRunner",
    "BenchmarkSummary",
    "FailureCategory",
    "FLANGE_LINK",
    "JOINT_NAMES",
    "JointLimits",
    "JointCommand",
    "KinematicCollisionBatch",
    "MyCobotCuroboError",
    "NamedJointState",
    "NominalPlan",
    "NominalPlanner",
    "Pose",
    "PlannerProfile",
    "PlaybackPlan",
    "PlanningFailure",
    "PlanningOutcome",
    "PlanningRequest",
    "ReplayRobotStateProvider",
    "ResidualCorrector",
    "ResidualObservation",
    "ResidualSafetyProfile",
    "RobotModelSpec",
    "RobotStateSample",
    "RuntimeSnapshot",
    "TCP_LINK",
    "SurfaceGoalSet",
    "SurfaceTarget",
    "SafetyDecision",
    "SafetyProjector",
    "SafetyStatus",
    "TaskFrameCandidate",
    "TaskFrameConfig",
    "TrajectoryExecutor",
    "TrajectorySample",
    "TrajectorySource",
    "ValidatedPlan",
    "ValidationMetrics",
    "ValidationProfile",
    "ValidationReport",
    "ValidationViolation",
    "build_surface_goal_set",
    "build_task_frame_candidates",
    "aggregate_results",
    "create_curobo_planner",
    "deserialize_request",
    "forward_kinematics",
    "load_curobo_robot_config",
    "load_benchmark_config",
    "load_planner_profile",
    "load_playback_plan",
    "load_robot_model_spec",
    "load_residual_safety_profile",
    "load_validation_profile",
    "reorder_joint_state",
    "playback_plan_from_dict",
    "require_executable_plan",
    "sample_benchmark_cases",
    "serialize_request",
    "verify_environment",
    "validate_nominal_plan",
    "validated_plan_to_playback_dict",
    "write_benchmark_reports",
    "write_playback_plan",
    "ZeroResidualCorrector",
    "InMemoryCommandAdapter",
]

__version__ = "0.1.0"
