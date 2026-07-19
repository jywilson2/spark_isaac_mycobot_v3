"""Structured errors for configuration and runtime-environment failures.

Expected planning infeasibility will use result objects in later phases.
Phase 0 raises these exceptions only when the process cannot safely initialize.
"""

from __future__ import annotations


class MyCobotCuroboError(Exception):
    """Base class for project-specific failures."""


class EnvironmentVerificationError(MyCobotCuroboError):
    """The CUDA, PyTorch, or cuRobo runtime does not meet Phase 0 requirements."""

