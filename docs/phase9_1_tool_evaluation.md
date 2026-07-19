# Phase 9.1 — Contact test tool evaluation

## Status

Requirements finalized; implementation deferred to `wip_phase9_1`.

Authoritative acceptance criteria are in [`spec.md`](../spec.md) §8.

## Purpose

Characterize whether the Phase 9 fabricated tool and its optional model provide
repeatable, measurable, collision-aware TCP behavior suitable for later
hardware testing. Physical checks in this phase are static and unpowered.

## Evaluation

Phase 9.1 shall:

1. inspect contact-face diameter, tool length, mounting features, offsets, and
   tolerances with measurement uncertainty;
2. perform a configured multi-trial remove/reinstall study and measure TCP
   positional and approach-axis angular repeatability;
3. calibrate and record the flange-to-TCP transform with method, date,
   equipment, uncertainty, SI position, and scalar-first `wxyz`;
4. compare independent FK with the calibrated transform;
5. verify Isaac/curobo visual placement and collision geometry;
6. run seeded normal-approach cube episodes using the optional profile and
   retain exact replay data; and
7. verify compatibility with the Phase 10 dry-run interface without issuing
   commands.

## Required metrics

- TCP positional remounting repeatability in metres and millimetres;
- approach-axis repeatability in radians and degrees;
- calibration residual and uncertainty;
- CAD-to-measured critical-dimensional error;
- simulated FK-to-expected-TCP position/orientation error;
- minimum tool/arm self-collision clearance; and
- normal-approach lateral and angular errors.

Initial hardware thresholds shall not be invented. The measured distributions
support a threshold proposal that requires explicit review before becoming a
hardware gate.

Tool-enabled Isaac tip metrics may be evaluated only from the calibrated
modeled frame. This does not alter Phase 7.1 reports, which remain null and
`not_evaluated`.
