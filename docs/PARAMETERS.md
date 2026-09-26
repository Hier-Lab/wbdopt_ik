# Parameter reference

Every parameter the MoveIt plugin accepts in `kinematics.yaml`. The formulas
referenced here are defined in [ALGORITHM.md](ALGORITHM.md).

Parameters are declared in
`wbdopt_ik_kinematics_plugin/src/wbdopt_ik_kinematics_plugin_parameters.yaml`
via `generate_parameter_library`. The listed bounds are enforced at load time —
a violation is a startup error, not a silent clamp.

---

## Summary

| Parameter | Type | Default | Range | Backend | Purpose |
|---|---|---|---|---|---|
| `kinematics_solver` | string | — | — | both | Must be `wbdopt_ik_kinematics_plugin/WbdoptIkKinematicsPlugin` |
| `kinematics_solver_timeout` | double | `0.05` | > 0 | both | Wall-clock budget per IK call; always fully consumed |
| `solver_type` | string | `qp` | `qp`, `nlopt` | both | Optimization backend |
| `tip_frames` | string[] | `[]` | — | both | Ordered IK tips; **set explicitly on MoveIt Humble** |
| `epsilon` | double | `1e-5` | > 0 | both | Convergence tolerance, m and rad, combined across tips |
| `max_iterations` | int | `100` | > 0 | both | SQP iterations (`qp`) or objective evaluations (`nlopt`) |
| `position_weight` | double | `1.0` | ≥ 0 | both | Translational task weight $w_p$ |
| `orientation_weight` | double | `1.0` | ≥ 0 | both | Rotational task weight $w_R$; **`0` = position-only IK** |
| `regularization` | double | `1e-4` | ≥ 0 | both | $\lambda$ on QP diagonal, or $\lambda^2$ toward seed in `nlopt` |
| `max_joint_step` | double | `0.20` | > 0 | `qp` | Trust region per SQP iteration |
| `safety_distance` | double | `0.01` | ≥ 0 | both | Minimum distance $\epsilon_s$ between enabled pairs |
| `collision_activation_distance` | double | `0.05` | ≥ `safety_distance` | both | Screening band $\epsilon_a$ |
| `joint_centering_name` | string | `""` | active joint | both | Joint biased toward its midpoint |
| `joint_centering_weight` | double | `0.0` | ≥ 0 | both | Weight $w_c$; keep well below task weights |
| `nominal_joint_names` | string[] | `[]` | — | both | Labels for `nominal_joint_positions` |
| `nominal_joint_positions` | double[] | `[]` | — | both | Rest posture used to rank restart candidates |

`kinematics_solver_search_resolution` is read by MoveIt but unused here.

---

## Timing and backend

| Parameter | Notes |
|---|---|
| `kinematics_solver_timeout` | Becomes `max_time`. Bounds the **whole restart loop**, not one attempt. **Both backends consume it fully on every call** — each restarts until the deadline to collect candidates for nominal-pose ranking, then returns the best. Raising it improves posture quality and the odds of solving a hard request; lowering it trades both for latency. An unreachable target costs the full budget before reporting failure. |
| `solver_type` | `qp` steps through per-iteration linearizations of the task and collision distances. `nlopt` optimizes the absolute configuration under the true nonlinear collision constraints, which is more faithful near contact. Both rank candidates identically, so switching does not change posture selection. |
| `tip_frames` | **The order here is the order of desired poses** in multi-tip calls, and must match what MoveIt passes. On MoveIt Humble the stock loader otherwise passes only the group's last link, silently reducing a dual-arm group to single-arm IK. |

> Returning the first solution found would be far faster (sub-millisecond), but
> makes the output jump between IK branches on a redundant arm — nearly
> identical requests produce visibly different postures. Collecting and ranking
> is the deliberate trade.

---

## Convergence

| Parameter | Notes |
|---|---|
| `epsilon` | Mapped to `tolerance`. A configuration is accepted only when the **combined** pose error across all tips is within `epsilon` (§7), and it doubles as the feasibility tolerance on collision constraints. Metres and radians share one value, so it is implicitly a tighter angular requirement than a linear one. |
| `max_iterations` | For `qp`, max SQP iterations per restart; for `nlopt`, max objective evaluations per attempt. **These are not comparable** — one SQP iteration costs an FK, Jacobians and a QP solve, roughly an order of magnitude more than one objective evaluation. In `qp` the deadline usually fires first, making the value largely inert. |

> **The two shipped defaults disagree.** The parameter declaration says `1e-5`;
> the example `kinematics.yaml` says `1e-3`. Prefer `1e-4`–`1e-3` on real
> hardware: `1e-5` is often below the robot's repeatability, and on a two-tip
> group both tips must satisfy it jointly. Too tight rejects valid solutions and
> burns the timeout; too loose visibly misses the target.

---

## Task weights

| Parameter | Notes |
|---|---|
| `position_weight` | Weight $w_p$ on translational error; scales both the cost and its Hessian contribution (§3). |
| `orientation_weight` | Weight $w_R$ on rotational error. **Set to `0` for position-only IK** — a zero weight removes orientation from the convergence test entirely (§7), not merely de-emphasizing it, so solutions are accepted on position alone. |

What matters is the **ratio** $w_p / w_R$, which trades one metre of position
error against one radian of orientation error. Since a radian of wrist rotation
displaces a tool tip far less than a metre, the defaults `1.0 / 1.0` weight
orientation relatively heavily. If the solver tracks orientation well but
consistently misses position, lower `orientation_weight` to `0.1`.

---

## Regularization and step size

`regularization` is one key with two meanings:

| Backend | Meaning |
|---|---|
| `qp` | Added linearly to the Hessian diagonal, $H + \lambda I$ — Levenberg–Marquardt damping. Keeps the QP conditioned near singularities, where $H$ goes rank-deficient and the unregularized step explodes. Raise it if you see erratic large steps on a stretched-out arm. |
| `nlopt` | Enters squared, as $\lambda^2 \lVert q - q_{\text{seed}} \rVert^2$ added to the objective. Anchors the solution to the seed rather than conditioning anything. At `1e-4` the coefficient is `1e-8` — negligible by design; `0.01`+ produces a noticeable pull. |

> The example config uses `0.01`, the declared default `1e-4`. For `qp`, `0.01`
> is a reasonable more-conservative choice; for `nlopt` it meaningfully biases
> toward the seed.

| Parameter | Notes |
|---|---|
| `max_joint_step` | **`qp` only.** Trust region bounding each joint by $\lvert \Delta q_i \rvert \le \delta$. The QP's collision rows and task Jacobian are *linearizations* valid only near the current iterate: too large and the step leaves that region — the arm can tunnel through a thin obstacle, since only the endpoints are checked. Too small and convergence needs more iterations. Lower toward `0.05` in clutter; `0.20` suits open workspaces. |

---

## Collision avoidance

Both distances are **signed** distances between SRDF-enabled pairs. Pairs
disabled in the SRDF are removed at construction and never considered.

| Parameter | Notes |
|---|---|
| `safety_distance` | The margin $\epsilon_s$ kept between geometry pairs; the enforced constraint is $d_{ab}(q) \ge \epsilon_s$ (§4). Zero does **not** disable collision checking — the constraint becomes $d_{ab} \ge 0$, still rejecting interpenetration. |
| `collision_activation_distance` | The screening band $\epsilon_a$: a pair enters the problem only when $d_{ab}(q) \le \epsilon_a$ (§4). Larger gives earlier warning and smoother avoidance at the cost of more constraint rows, and SLSQP's inner QP scales poorly in that count. Too small and a fast joint can cross the band between evaluations. The ordering $\epsilon_a \ge \epsilon_s$ is validated at startup. |

> **The most common cause of a solver that never converges.** The convergence
> test rejects any configuration where *any* enabled pair violates the margin.
> If the model has link pairs permanently in contact — adjacent links whose
> meshes overlap at the joint — no configuration can pass, and every call
> returns the seed with `FAILED` after burning the full timeout. Fix it by
> disabling those pairs in the SRDF, not by zeroing `safety_distance`.

---

## Posture control

| Parameter | Notes |
|---|---|
| `joint_centering_name` | The single active joint biased toward the midpoint of its bounds. Must name a joint in the group's active set or startup fails. Typical use is a redundant DoF that should stay centred unless needed — a base slider or torso lift, so the robot prefers to reach with its arm. |
| `joint_centering_weight` | Weight $w_c$ (§3). Normalized by the joint's squared half-range, so it is dimensionless and comparable across joints with different travel. Requires `joint_centering_name` when non-zero. |

> Keep $w_c$ **well below** the task weights — this is a tiebreaker among
> otherwise equivalent solutions, not a task. The example uses `0.001` against
> task weights of `1.0`. At high values the solver sacrifices pose accuracy to
> centre the joint, and since it competes inside the same cost while the
> convergence test measures only pose error, it can prevent convergence.

---

## Nominal pose ranking

| Parameter | Notes |
|---|---|
| `nominal_joint_names` | Labels for `nominal_joint_positions`; the two arrays must be the same length. Names are matched to the group's active joints at initialization, so **YAML order need not match model order** — this is what makes it safe to paste values from an `initial_positions.yaml`. |
| `nominal_joint_positions` | The reference posture. Both backends collect every converged solution until the deadline and return the one minimizing squared distance to it, with continuous joints compared by shortest angular distance (§5). |

This is the mechanism that makes a redundant arm produce repeatable, sensible
configurations instead of an arbitrary valid one. If unset, the request seed is
used instead, biasing toward minimal motion from the current state — usually
right for servoing, but it means calls from different seeds may land in
different branches.

---

## Worked example

A 15-DoF dual-arm group on a prismatic base:

```yaml
whole_body:
  kinematics_solver: wbdopt_ik_kinematics_plugin/WbdoptIkKinematicsPlugin
  kinematics_solver_timeout: 0.05
  solver_type: qp
  tip_frames: [left_fr3_hand_tcp, right_fr3_hand_tcp]

  epsilon: 0.001                       # 1 mm / 1 mrad, combined across both tips
  max_iterations: 100

  position_weight: 1.0
  orientation_weight: 1.0

  regularization: 0.01                 # diagonal damping for the QP
  max_joint_step: 0.20                 # trust region per SQP iteration

  safety_distance: 0.005               # keep 5 mm between enabled pairs
  collision_activation_distance: 0.10  # constrain pairs within 10 cm

  joint_centering_name: base_slider_joint
  joint_centering_weight: 0.001        # gentle: prefer reaching with the arms

  nominal_joint_names: [base_slider_joint, left_fr3_joint1, ...]
  nominal_joint_positions: [0.0, 0.0, -0.785, ...]
```

To switch to the NLopt backend, set `solver_type: nlopt`. The nominal arrays
apply unchanged. Raise `regularization` to around `0.01` if you additionally
want each solve pulled toward its seed.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Always fails, returns the seed, uses the full timeout | Permanently-colliding link pairs not disabled in the SRDF — see `safety_distance` |
| Only one arm solves in a dual-arm group | `tip_frames` not set explicitly (MoveIt Humble) |
| Position tracks, orientation drifts | Raise `orientation_weight`, or check targets are in the URDF root frame |
| Solves always take the full timeout | Expected: both backends restart until the deadline to rank candidates. Lower `kinematics_solver_timeout` to trade posture quality for latency |
| Solutions jump between postures | Set `nominal_joint_positions`, and check the timeout is long enough to find more than one candidate |
| Erratic large joint motions near full extension | Raise `regularization` (singularity damping) |
| Passes through thin obstacles between iterations | Lower `max_joint_step`, raise `collision_activation_distance` |
| Startup error on collision distances | `collision_activation_distance` < `safety_distance` |
