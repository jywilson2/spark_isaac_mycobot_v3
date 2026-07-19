"""MyCobot 280 M5 constrained-approach planning foundations.

Phase 0 exposes only environment verification. Planning, robot-model,
trajectory-validation, and residual interfaces are added one specification
phase at a time after their acceptance criteria are implemented and tested.
"""

from mycobot_curobo.errors import EnvironmentVerificationError, MyCobotCuroboError
from mycobot_curobo.version_guard import (
    EnvironmentReport,
    RuntimeSnapshot,
    verify_environment,
)

__all__ = [
    "EnvironmentReport",
    "EnvironmentVerificationError",
    "MyCobotCuroboError",
    "RuntimeSnapshot",
    "verify_environment",
]

__version__ = "0.1.0"
