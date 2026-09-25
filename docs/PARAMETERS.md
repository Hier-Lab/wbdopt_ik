# Parameter reference

Every parameter the MoveIt plugin accepts in `kinematics.yaml`, what it does in
the math, and how to tune it. The formulas referenced here are defined in
[ALGORITHM.md](ALGORITHM.md).

Parameters are declared in
`wbdopt_ik_kinematics_plugin/src/wbdopt_ik_kinematics_plugin_parameters.yaml`
via `generate_parameter_library`; the bounds listed below are enforced at load
time and a violation is a startup error, not a silent clamp.

---

## MoveIt framework parameters

These are read by MoveIt itself, not declared by this plugin.

### `kinematics_solver`
Must be `wbdopt_ik_kinematics_plugin/WbdoptIkKinematicsPlugin`.

### `kinematics_solver_timeout`
**Units:** seconds · **Typical:** `0.05`

The wall-clock budget for one IK call, becoming `max_time` in the library. It
bounds the **entire restart loop**, not one attempt.

**Both backends consume the full timeout on every call.** Each keeps restarting
until the deadline to collect candidates for nominal-pose ranking, then returns
the best one. Raising this value directly raises latency while improving posture
quality and the chance of solving a hard request; lowering it does the reverse.

This is a deliberate trade. Returning the first solution found is far faster
(sub-millisecond) but makes the output jump between IK branches on a redundant
arm — nearly identical requests produce visibly different postures.

For an unreachable target, both backends spend the whole budget before
reporting failure. If IK is called in a planning loop, that cost is paid on
every failure.

### `kinematics_solver_search_resolution`
Unused by this solver; retained for MoveIt compatibility.

---

## Backend selection

### `solver_type`
**Type:** string · **Default:** `qp` · **Values:** `qp`, `nlopt`

Selects the optimization backend (§5 and §6 of ALGORITHM.md).

Use `qp` when you care *which* valid solution you get — it ranks all restarts
against `nominal_joint_positions` and returns the most nominal-like posture.
Use `nlopt` when you want the answer fast and any valid solution is acceptable,
or when linearized collision constraints are too loose near contact.

### `tip_frames`
**Type:** string array · **Default:** `[]`

The ordered IK tip frames. **The order here is the order of desired poses** in
multi-tip calls, and must match the order MoveIt passes.

On MoveIt Humble, set this explicitly. The stock kinematics plugin loader
otherwise passes only the group's last link, which silently reduces a dual-arm
group to single-arm IK. Leaving it empty falls back to that inferred tip.

---

## Convergence

### `epsilon`
**Units:** metres and radians · **Default:** `1e-5` · **Must be** > 0

The convergence tolerance, mapped to the library's `tolerance`. A configuration
is accepted only when the **combined** pose error across all tips is within
`epsilon` (§7), and also serves as the feasibility tolerance on collision
constraints.

Because metres and radians share one value, it is implicitly a tighter angular
requirement than a linear one. Note the two shipped defaults differ: the
parameter declaration uses `1e-5`, the example `kinematics.yaml` uses `1e-3`.
Prefer `1e-4` to `1e-3` for real robots — `1e-5` is often below the repeatability
of the hardware and makes solves needlessly slow, or unreachable on a two-tip
group where both tips must satisfy it jointly.

Too tight and valid solutions get rejected and the timeout is burned; too loose
and the returned pose visibly misses the target.

### `max_iterations`
**Type:** int · **Default:** `100` · **Must be** > 0

For `qp`, the maximum SQP iterations per restart. For `nlopt`, the maximum
objective evaluations per attempt (`set_maxeval`).

100 is reasonable for both. The QP backend usually converges in well under 20
iterations from a good seed; if it regularly hits the cap, the problem is more
likely `max_joint_step` being too small or a conditioning issue than a genuine
need for more iterations.

---

## Task weights

### `position_weight`
**Default:** `1.0` · **Must be** ≥ 0

Weight $w_p$ on translational error. Scales both the cost and its Hessian
contribution (§3).

### `orientation_weight`
**Default:** `1.0` · **Must be** ≥ 0

Weight $w_R$ on rotational error.

**Set to `0` for position-only IK.** This is not merely a soft de-emphasis: a
zero weight removes orientation from the convergence test entirely (§7), so
solutions are accepted on position alone. This is the correct way to do
position-only IK here.

What matters is the **ratio** $w_p / w_R$, which trades one metre of position
error against one radian of orientation error. Since a radian of wrist rotation
displaces a tool tip far less than a metre, the defaults `1.0 / 1.0` weight
orientation relatively heavily. If the solver tracks orientation well but
consistently misses position, lower `orientation_weight` to `0.1`.

---

## Regularization and step size

### `regularization`
**Default:** `1e-4` · **Must be** ≥ 0

**Its meaning differs by backend** — the same number does two different things:

- **`qp`:** added linearly to the Hessian diagonal, $H + \lambda I$. This is
  Levenberg–Marquardt damping: it keeps the QP well-conditioned near
  singularities, where $H$ becomes rank-deficient and the unregularized step
  explodes. Raise it if you see erratic large steps near a stretched-out arm.

- **`nlopt`:** enters squared, added to the objective as the term
  $\lambda^2 \lVert q - q_{\text{seed}} \rVert^2$. This anchors the solution to the seed,
  biasing toward the nearest solution branch rather than conditioning anything.
  Because it is squared, `1e-4` contributes a coefficient of `1e-8` — negligible
  by design. Raising it to `0.01` or more produces a noticeable pull toward the
  seed.

The example config uses `0.01`, the declared default is `1e-4`. For `qp`, `0.01`
is a reasonable, more conservative choice; for `nlopt` it meaningfully biases
toward the seed.

### `max_joint_step`
**Units:** radians or metres · **Default:** `0.20` · **Must be** > 0

**`qp` only.** The trust region on one SQP iteration, bounding each joint by
$\lvert \Delta q_i \rvert \le \delta$.

This exists because the QP's collision rows and task Jacobian are *linearizations*
valid only near the current iterate. Too large and the step leaves the region
where they hold — the arm can tunnel through a thin obstacle between iterations,
since only the endpoints are checked. Too small and convergence needs many more
iterations, risking the `max_iterations` cap.

Lower it toward `0.05` in cluttered scenes; `0.20` suits open workspaces.

---

## Collision avoidance

Both distances refer to **signed** distances between SRDF-enabled collision
pairs. Pairs disabled in the SRDF are removed at construction and never
considered.

### `safety_distance`
**Units:** metres · **Default:** `0.01` · **Must be** ≥ 0

The margin $\epsilon_s$ the solver keeps between geometry pairs. The enforced
constraint is $d_{ab}(q) \ge \epsilon_s$ (§4).

> **This is the most common cause of a solver that never converges.** The
> convergence test rejects any configuration where *any* enabled pair violates
> the margin. If the model has link pairs that are permanently in contact —
> adjacent links whose collision meshes overlap at the joint — no configuration
> can ever pass, and the solver returns the seed with `FAILED` after burning the
> full timeout. Fix this in the SRDF by disabling those pairs, not by setting
> `safety_distance` to zero.

Note that zero is not a way to disable collision checking: the constraint
becomes $d_{ab} \ge 0$, which still rejects any interpenetration.

### `collision_activation_distance`
**Units:** metres · **Default:** `0.05` · **Must be** ≥ `safety_distance`

The screening band $\epsilon_a$. A pair is carried into the optimization only
when $d_{ab}(q) \le \epsilon_a$; pairs further apart are ignored for that
evaluation (§4).

This is a performance control with a correctness edge. Larger values give the
optimizer earlier warning of an approaching obstacle, producing smoother
avoidance, at the cost of more constraint rows — and SLSQP's inner QP scales
poorly in the constraint count. Too small and a fast-moving joint can cross the
band between evaluations without the constraint ever activating.

The ordering requirement $\epsilon_a \ge \epsilon_s$ is validated at startup;
violating it would mean a pair could breach the safety margin before ever
entering the problem.

---

## Posture control

### `joint_centering_name`
**Type:** string · **Default:** `""` (disabled)

The single active joint biased toward the midpoint of its bounds. Must name a
joint in the group's active set, or startup fails.

The typical use is a redundant DoF that should stay centred unless it is needed
— a base slider or a torso lift, so the robot prefers to reach with its arm and
uses the extra axis only when it must.

### `joint_centering_weight`
**Default:** `0.0` (disabled) · **Must be** ≥ 0

The weight $w_c$ on the centering term (§3). Because the term is normalized by
the joint's squared half-range, $w_c$ is dimensionless and comparable across
joints with different travel.

Keep it **well below** the task weights — this is a tiebreaker among otherwise
equivalent solutions, not a task. The example uses `0.001` against task weights
of `1.0`. At high values the solver will sacrifice pose accuracy to centre the
joint, and since it competes inside the same cost while the convergence test
only measures pose error, it can prevent convergence entirely.

Requires `joint_centering_name` to be set when non-zero.

---

## Nominal pose ranking

### `nominal_joint_names` / `nominal_joint_positions`
**Type:** string array / double array · **Default:** `[]`

A reference posture. `nominal_joint_names[i]` labels `nominal_joint_positions[i]`;
the two must be the same length. Names are matched to the group's active joints
during initialization, so **YAML order does not need to match model order** —
this is what makes it safe to paste values from an `initial_positions.yaml`.

**`qp` backend only.** The QP backend collects every converged solution until
the deadline and returns the one minimizing squared distance to this posture,
with continuous joints compared by shortest angular distance (§5). This is the
mechanism that makes a redundant arm produce repeatable, human-sensible
configurations instead of an arbitrary valid one.

If unset, the request seed is used as the reference, which biases toward
minimal motion from the current state — usually the right behaviour for
servoing, but it means consecutive calls from different seeds may land in
different branches.

Both backends use these identically, so switching `solver_type` does not change
which posture is selected.

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

To switch this group to the NLopt backend, set `solver_type: nlopt`. The
nominal arrays apply unchanged. Raise `regularization` to around `0.01` if you
additionally want each individual solve pulled toward its seed.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Always fails, returns the seed, uses the full timeout | Permanently-colliding link pairs not disabled in the SRDF — see `safety_distance` |
| Only one arm solves in a dual-arm group | `tip_frames` not set explicitly (MoveIt Humble) |
| Position tracks, orientation drifts | Raise `orientation_weight`, or check that targets are in the URDF root frame |
| Solves always take the full timeout | Expected: both backends restart until the deadline to rank candidates. Lower `kinematics_solver_timeout` to trade posture quality for latency |
| Solutions jump between postures | Set `nominal_joint_positions`, and check the timeout is long enough to find more than one candidate |
| Erratic large joint motions near full extension | Raise `regularization` (singularity damping) |
| Passes through thin obstacles between iterations | Lower `max_joint_step`, raise `collision_activation_distance` |
| Inconsistent postures across calls | Set `nominal_joint_positions` and use `solver_type: qp` |
| Startup error on collision distances | `collision_activation_distance` < `safety_distance` |
