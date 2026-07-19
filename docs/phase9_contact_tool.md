# Phase 9 — Fabricated contact test tool

## Status

Requirements finalized; implementation deferred to `wip_phase9`.

Authoritative acceptance criteria are in [`spec.md`](../spec.md) §8.

## Purpose

Design, document, model, and fabricate an optional short, stiff contact tool
for `joint6_flange`. The tool provides a circular contact face coaxial with the
configured approach axis and a measurable TCP for later simulation and
physical evaluation.

The bare-flange identity TCP remains the default. Phase 9 performs no powered
arm motion and does not change Phase 7.1's `not_evaluated` Isaac tip metrics.

## Fabrication artifact

Phase 9 shall create and version:

- parameterized OpenSCAD source using millimetres;
- a matching generated, manifold/watertight 3D-printable STL;
- documented source parameters and critical dimensions;
- the exact OpenSCAD command and version needed to regenerate the STL;
- print orientation, supports, material assumptions, minimum wall thickness,
  mounting clearances, and fastener requirements; and
- dimensioned drawings or equivalent model documentation.

Both source and generated STL are committed so the design is reviewable and
the printable artifact is immediately available. Validation shall cover
manifoldness/watertightness, critical dimensions, minimum wall thickness, and
mounting fit. A fit-check coupon may precede the complete print.

## Robot-model integration

The optional tool profile shall define a measured
`joint6_flange`-to-`tcp_link` fixed transform in metres and scalar-first
`wxyz`. It shall include matching URDF/Isaac visual geometry and cuRobo
collision geometry. Calibration may not be hidden in target coordinates.

The physical flange diameter and mounting pattern must be measured. The
Phase 7.1 31 mm diameter assumption and 14 mm cube derivation are then either
confirmed or revised prospectively without rewriting historical reports.

## Acceptance boundary

A fabricated example must fit without forcing, uncontrolled play, or
interference; as-built dimensions are recorded. The optional profile must load
without changing bare-flange behavior, and visual/collision models must share
the documented dimensions.
