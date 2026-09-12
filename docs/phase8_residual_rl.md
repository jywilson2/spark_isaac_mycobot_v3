# Phase 8 — Bounded residual RL (Isaac Lab / Isaac Sim)

**Branch:** `wip_phase8`  
**Status:** Implementation complete for acceptance criteria (2026-09-11),
including the actuator-noise amendment. Offline/sim-only training and
residual-on vs residual-off reporting are wired with joint actuator noise.
Live Isaac Lab Kit RL loops remain optional host wrappers around the same
`ResidualObservation` / `ResidualCorrector` contracts.

## Objective

Train and evaluate a residual policy that improves approach metrics under
model mismatch while preserving cuRobo as the exclusive motion planner.

## What landed

### Core (`mycobot_curobo`)

- `PolicyResidualCorrector` / `FixedResidualCorrector` behind the Phase 5
  `ResidualCorrector` protocol (non-zero correctors).
- `FiniteDifferenceResidualMapper`: bounded local Cartesian → Δq map via FK
  finite differences (not IK, not a replacement trajectory).
- `TrajectoryExecutor` applies projected residuals when a mapper is injected;
  on mapping / joint-envelope / corridor failure it falls back to the nominal
  waypoint or stops (`residual_fallback: nominal|stop`).
- `config/residual_safety.yml`: `max_joint_delta_rad` and `residual_fallback`
  on `simulation_zero_residual` and `simulation_bounded_residual`.
- Offline sim trainer (`residual_train.py`) refuses
  `ENABLE_MYCOBOT_HARDWARE_TESTS=1` and records the actuator-noise profile.
- Comparison reports (`residual_compare.py`) label `sim_only: true` with
  residual-off vs residual-on summaries.
- **Actuator noise** (`config/actuator_noise.yml`,
  `mycobot_curobo.actuator_noise`): joint-space motor/servo disturbance used
  in train and residual-on/off apply; distinct from tip bias and measurement
  noise.

### Scripts

- `scripts/train_residual_policy.py` / `scripts/host/train_residual_policy.sh`
- `scripts/compare_residual_benchmark.py` (GPU Phase 6 planner path when used
  with live cuRobo; unit tests cover report labeling without GPU)
- `scripts/host/residual_observation.py` — host bridge for Kit wrappers

### CLI

- `--execute-residual-checkpoint` on the Phase 6 benchmark entry
- `residual_train_main` / `residual_compare_main`

## Hard constraints preserved

- Deployed path: validated cuRobo plan → optional bounded local correction →
  independent projector / post-map corridor checks.
- Residual may not generate a replacement trajectory, invoke another planner,
  or map target pose → full 6-DOF joint solutions.
- Checkpoints are advisory; deterministic validation / safety projection has
  final authority.
- No physical hardware motion from the training loop.

## Acceptance evidence

| Criterion | Evidence |
|-----------|----------|
| Training/eval scripts run only in sim | Offline trainer + host script refuse hardware flag; reports `sim_only` |
| Residual bounds configuration-driven and tested | `residual_safety.yml` + `test_safety.py` / `test_execution.py` |
| Residual-on vs residual-off reports labeled sim-only | `write_residual_comparison_report` + `test_phase8_residual.py` |
| Physical hardware never moved by training | `assert_sim_only_training_environment` |
| Actuator noise model in train/eval | `config/actuator_noise.yml` + `test_actuator_noise.py`; smoke/apply `--actuator-noise-profile` |

`./scripts/run_verification.sh ci` — **318 passed**, Ruff clean (2026-09-11).
Offline residual retrain under actuator noise: tip error reduced
(`phase8_offline_policy.json`, `phase8_gui_demo_visible_policy.json`).

## Remaining / review

- Live Isaac Lab `rsl_rl` training inside Kit is not required for the current
  acceptance gate; the offline synthetic backend satisfies sim-only train/eval
  contracts. Pin `SPARK_ISAACLAB_BRANCH` before claiming Kit reproducibility.
- GPU residual-on/off comparison against Phase 6 scenes should be run on the
  host with cuRobo when operators want planner-backed metrics
  (`scripts/compare_residual_benchmark.py`).
- `isaac_lab/` directory was not writable in this environment; host bridge
  lives under `scripts/host/residual_observation.py` instead.

## Residual GUI smoke (noise + playback)

Host script: `scripts/host/smoke_phase8_residual_gui.sh`

**Default = demo-visible (Option 1 + 2):**

1. Uses residual safety profile `simulation_demo_visible` (25 mm / ~5° /
   0.15 rad joint Δ — demo-only, not acceptance).
2. Injects tip bias default **8 mm X** into a biased off-bundle
   (`--inject-tip-bias-only`) so residual-off is visibly misaligned.
3. Trains/forces a canceling demo checkpoint for the same tip bias under
   actuator noise (`SPARK_PHASE8_TIP_BIAS_*`,
   `SPARK_PHASE8_ACTUATOR_NOISE_PROFILE`, default `simulation_demo`).
4. **Pass A (residual-off):** plays the **biased** bundle with HUD
   `TEST Pass A of 2: Residual OFF (biased tip, no correction) […]`.
5. **Pass B (residual-on):** applies `PolicyResidualCorrector` on the
   biased bundle, then plays the corrected bundle with HUD
   `TEST Pass B of 2: Residual ON (tip bias corrected) […]`.

Use `--subtle` for production-like `simulation_bounded_residual` bounds
(smaller tip bias, train-if-missing only).

Override knobs: `SPARK_PHASE8_SOURCE_BUNDLE`, `SPARK_PHASE8_EPISODE_INDEX`,
`SPARK_PHASE8_NOISE_STD_RAD`, `SPARK_PHASE8_SAFETY_PROFILE`,
`SPARK_PHASE8_CHECKPOINT`, `SPARK_PHASE8_TIP_BIAS_{X,Y,Z}`,
`SPARK_PHASE8_OFF_BUNDLE`, `SPARK_PHASE8_ACTUATOR_NOISE_PROFILE`.

**Review:** An earlier inverted A/B (residual on nominal) made Pass B look
worse than Pass A; that path is retired.

**Demo A/B evidence (2026-09-11, actuator noise on standard 2×10):** Phase 7.2
standard 2×10 plan/play EXIT:0 (seed 4242, tip=20). Phase 8 residual demo A/B
on episode 0 EXIT:0 — Pass A/B tip=10, body=0, self=0, success_rate=1.0
(`phase8_residual_gui_standard_2x10_{off,on}.json`).

Helper modules: `mycobot_curobo/residual_playback.py`,
`mycobot_curobo/actuator_noise.py`,
`scripts/apply_residual_noise_to_bundle.py` (`--inject-tip-bias-only`,
`--tip-bias-m`, `--actuator-noise-profile`, `--force-retrain`).

