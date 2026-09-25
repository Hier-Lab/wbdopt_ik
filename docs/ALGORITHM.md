# The wbdopt_ik IK algorithm

This document describes what the solver actually computes: the shared cost and
constraint model, the two optimization backends built on it, and how each
tunable parameter enters the math.

Notation: $q \in \mathbb{R}^{n}$ is the vector of **active joint positions**
(the configured one-DoF joints, not Pinocchio's full $n_q$ configuration),
$N_e$ is the number of end effectors, and $\mathcal{C}$ is the set of enabled
collision pairs.

---

## 1. Configuration mapping

The optimizer works in a reduced space. Only the joints listed in
`active_joint_names` are variables; everything else stays at the model's
neutral configuration. Each active joint contributes exactly one scalar.

Bounded revolute and prismatic joints map directly. A **continuous** revolute
joint occupies two entries in Pinocchio's configuration vector ($n_q = 2$), and
the solver stores the angle $\theta$, expanding it on each evaluation:

$$
q^{\text{full}}_{\text{idx}} = \cos\theta,
\qquad
q^{\text{full}}_{\text{idx}+1} = \sin\theta .
$$

Continuous joints are given the artificial bounds $[-\pi, \pi]$, since they have
no limits of their own.

---

## 2. Pose error

For end effector $i$ with current placement $\left(R_i(q),\, p_i(q)\right)$ and
target $\left(R_i^{d},\, p_i^{d}\right)$, the translational error is

$$
e_{p,i} = p_i^{d} - p_i(q).
$$

The orientation error is the logarithm of the relative rotation, **rotated into
the world frame**:

$$
e_{R,i} = R_i(q)\,\log\!\left(R_i(q)^{\mathsf{T}} R_i^{d}\right)^{\vee}.
$$

The world-frame rotation matters: it makes $e_{R,i}$ consistent with the
`LOCAL_WORLD_ALIGNED` frame Jacobians used below, so the two agree to first
order. Mixing conventions here is a common source of non-convergence.

---

## 3. The cost function

Let $J_{p,i}, J_{R,i} \in \mathbb{R}^{3 \times n}$ be the translational and
rotational blocks of the `LOCAL_WORLD_ALIGNED` frame Jacobian, restricted to
the active columns. With weights $w_p$ (`position_weight`) and $w_R$
(`orientation_weight`), the task error is

$$
E(q) \;=\; \sum_{i=1}^{N_e}
\Big( w_p \lVert e_{p,i} \rVert^2 + w_R \lVert e_{R,i} \rVert^2 \Big).
$$

This is the quantity reported as `CostData::squared_error`.

### Joint centering

An optional term biases one named joint $j$ toward the midpoint of its bounds.
With $q_{j,\text{mid}} = \tfrac{1}{2}(q_{j,\min} + q_{j,\max})$ and half-range
$h_j = \tfrac{1}{2}(q_{j,\max} - q_{j,\min})$,

$$
E_{\text{center}}(q) \;=\; \frac{w_c}{\max(h_j^2,\,10^{-12})}\,
\big(q_j - q_{j,\text{mid}}\big)^2 .
$$

Normalizing by $h_j^2$ makes $w_c$ (`joint_centering_weight`) dimensionless, so
the same value behaves comparably across joints with different travel. The
floor on the denominator guards a zero-range joint.

### Gradient and Gauss–Newton Hessian

`CostData` carries the **half-scaled** gradient and the Gauss–Newton Hessian:

$$
g(q) \;=\; -\sum_{i=1}^{N_e}
\Big( w_p J_{p,i}^{\mathsf{T}} e_{p,i} + w_R J_{R,i}^{\mathsf{T}} e_{R,i} \Big),
\qquad
H(q) \;=\; \sum_{i=1}^{N_e}
\Big( w_p J_{p,i}^{\mathsf{T}} J_{p,i} + w_R J_{R,i}^{\mathsf{T}} J_{R,i} \Big).
$$

> **Convention.** $g$ is the gradient of $\tfrac{1}{2}E$, not of $E$ — note
> $\nabla E = 2g$. The QP consumes $H$ and $g$ directly, in which form the
> factor is absorbed. A gradient-based caller must apply the factor of two
> itself, as the NLopt objective does. This is easy to get wrong; it is checked
> against finite differences in the tests.

$H$ is the Gauss–Newton approximation: it drops the second-order term
involving $\partial J / \partial q$, which is standard for least-squares
problems and keeps $H$ positive semidefinite.

---

## 4. Collision model

Both backends share one signed-distance constraint. For pair $(a,b)$ with
minimum distance $d_{ab}(q)$ and safety margin $\epsilon_s$ (`safety_distance`):

$$
h_{ab}(q) \;=\; \epsilon_s - d_{ab}(q) \;\le\; 0 .
$$

Distances are **signed**, so interpenetration gives $d_{ab} < 0$ and the
constraint grows without a discontinuity at contact.

Let $p_a, p_b$ be the witness points and $n_{ab}$ the contact normal
(from Coal; if degenerate, it falls back to $p_b - p_a$ normalized). With
$J_a, J_b$ the translational Jacobians **of the witness points**,

$$
\frac{\partial d_{ab}}{\partial q} = n_{ab}^{\mathsf{T}} (J_b - J_a),
\qquad
\frac{\partial h_{ab}}{\partial q} = -\,n_{ab}^{\mathsf{T}} (J_b - J_a).
$$

The witness-point Jacobian is built from the parent joint's Jacobian shifted to
the contact point $x$:

$$
J_{\text{point}} = J_{v} - \lfloor x - o_{\text{joint}} \rfloor_\times \, J_{\omega},
$$

where $o_{\text{joint}}$ is the joint origin and $\lfloor \cdot \rfloor_\times$
is the skew-symmetric matrix.

### Activation screening

Enforcing every pair is wasteful — most are far apart. A pair is carried into
the optimization only when it is within `collision_activation_distance`
$\epsilon_a$:

$$
h_{ab}(q) \;\ge\; \epsilon_s - \epsilon_a
\quad\Longleftrightarrow\quad
d_{ab}(q) \;\le\; \epsilon_a .
$$

This requires $\epsilon_a \ge \epsilon_s$, which the option validator enforces.
On a 7-DoF FR3 this typically reduces ~200 pairs to ~20.

---

## 5. Backend A — differential QP (`solver_type: qp`)

A sequential QP. At each iterate $q_k$ it solves for a joint **increment**
$\Delta q$ under a linearization of both the task and the collision distances.

$$
\begin{aligned}
\min_{\Delta q} \quad
& \tfrac{1}{2}\,\Delta q^{\mathsf{T}} \big(H(q_k) + \lambda I\big)\, \Delta q
   \;+\; g(q_k)^{\mathsf{T}} \Delta q \\[4pt]
\text{s.t.}\quad
& \max\!\big(q_{\min} - q_k,\; -\delta\big)
  \;\le\; \Delta q \;\le\;
  \min\!\big(q_{\max} - q_k,\; \delta\big), \\[4pt]
& n_{ab}^{\mathsf{T}}(J_b - J_a)\,\Delta q \;\ge\; \epsilon_s - d_{ab}(q_k),
  \qquad \forall (a,b) \in \mathcal{C}_{\text{active}} .
\end{aligned}
$$

Then $q_{k+1} = q_k + \Delta q^\star$.

- $\lambda$ is `regularization`, added to the diagonal. It damps the step near
  singularities, where $H$ becomes ill-conditioned. **It enters linearly here**
  (as $\lambda$, not $\lambda^2$) — this differs from the NLopt backend, below.
- $\delta$ is `max_joint_step`, a trust region. The constraint rows are only
  valid near $q_k$, so the step must stay small enough for the linearization to
  hold.
- The collision row is the linearization of the safety requirement
  $d_{ab}(q_k + \Delta q) \ge \epsilon_s$, using the first-order expansion

$$
d_{ab}(q_k + \Delta q) \;\approx\; d_{ab}(q_k)
  + \frac{\partial d_{ab}}{\partial q}\,\Delta q .
$$

Solved with qpOASES in MPC mode. An inner iteration stops when the pose
converges, when $\lVert \Delta q \rVert < 0.1\,\epsilon$, when the QP fails, or
at the deadline.

### Restarts and nominal-pose ranking

The QP backend **collects every converged solution** until the deadline, then
ranks them by squared distance to `nominal_joint_positions`:

$$
D(q) = \sum_{i} \Delta_i^2,
\qquad
\Delta_i = \begin{cases}
\operatorname{rem}(q_i - q_i^{\text{nom}},\, 2\pi) & \text{continuous joint} \\
q_i - q_i^{\text{nom}} & \text{otherwise,}
\end{cases}
$$

with task error breaking ties. The remainder gives the shortest angular
distance, so a continuous joint is not penalized for winding. Without a nominal
pose the seed is used instead. This is what produces repeatable, human-sensible
postures on a redundant arm — a 15-DoF dual-arm group has a continuum of exact
solutions, and this picks the one nearest a known-good configuration.

---

## 6. Backend B — nonlinear SLSQP (`solver_type: nlopt`)

Rather than stepping through linearizations, this optimizes over the absolute
configuration directly:

$$
\begin{aligned}
\min_{q} \quad & E(q) \;+\; \lambda^2 \lVert q - q_{\text{seed}} \rVert^2 \\[4pt]
\text{s.t.}\quad & q_{\min} \le q \le q_{\max}, \\[4pt]
& h_{ab}(q) \le \epsilon, \qquad \forall (a,b) \in \mathcal{C}_{\text{active}} .
\end{aligned}
$$

Note the differences from the QP backend: the variable is $q$, not $\Delta q$;
the collision constraints are the true nonlinear $h_{ab}$ rather than a
linearization; and **regularization enters as $\lambda^2$**, anchoring the
solution to the seed rather than damping a step.

The analytic gradient supplied to SLSQP is

$$
\nabla \big[ E + \lambda^2 \lVert q - q_{\text{seed}} \rVert^2 \big]
= 2g(q) + 2\lambda^2 (q - q_{\text{seed}}),
$$

where the factor on $g$ is the half-scaling convention from §3, and the
regularization term is differentiated at full scale.

### Early termination

SLSQP will keep polishing a solution long after it is accurate enough. The
objective therefore latches the best point seen and stops the moment the answer
is good enough:

$$
\sqrt{E(q)} \le \epsilon
\quad\text{and}\quad
h_{ab}(q) \le \epsilon \;\; \forall (a,b)
\qquad\Longrightarrow\qquad \texttt{nlopt::forced\_stop}.
$$

Because `forced_stop` unwinds without writing back to the caller's vector, the
converged point must be latched inside the objective — it cannot be recovered
afterward.

This bounds each *attempt*, not the call. The outer loop keeps restarting until
the deadline (§8), so the effect is that more restarts fit in the budget and
the ranking has more candidates to choose from, not that the call returns
sooner.

Constraint rows that cannot be evaluated are reported as large **negative**
(trivially satisfied), never positive: for an inequality $h \le \epsilon$, a
large positive value would tell SLSQP the region is infeasible.

---

## 7. Convergence test

Both backends accept a configuration only if it passes the same test:

$$
\sqrt{\;\sum_{i=1}^{N_e}
\Big( [w_p > 0]\,\lVert e_{p,i} \rVert^2
   + [w_R > 0]\,\lVert e_{R,i} \rVert^2 \Big)} \;\le\; \epsilon
\qquad\text{and}\qquad
h_{ab}(q) \le \epsilon \;\; \forall (a,b) \in \mathcal{C}.
$$

Two things to note. The norm is **combined across all tips**, so a two-tip
group must satisfy $\epsilon$ jointly, not per tip. And the weights act only as
on/off switches here — setting `orientation_weight: 0` performs position-only
IK by dropping orientation from both the cost and this test.

The collision half checks **all** enabled pairs, not just the screened active
set, so screening never lets a violation through.

> **Practical note.** `collisionSafe` returns false if *any* enabled pair has
> $d_{ab} < \epsilon_s$. If the model has link pairs that are always in contact
> — adjacent links whose meshes overlap at the joint — no configuration ever
> passes, and the solver returns the seed with `FAILED`. Disable those pairs in
> the SRDF. This is the most common cause of a solver that "never converges".

---

## 8. Restart loop

Both backends share the outer structure, which follows TRAC-IK:

```
deadline = now + max_time
attempt from the seed
while now < deadline:
    q0 = seed (first attempt) or uniform random within joint bounds
    run the backend from q0
    if converged: record it
```

The first attempt always uses the caller's seed, which is what makes the solver
fast and temporally coherent when tracking a moving target. Later attempts
sample uniformly within the joint bounds to escape local minima — an IK problem
on a redundant arm has many disconnected solution branches, and a gradient
method cannot cross between them.

Both backends then **collect every converged solution until the deadline and
return the one nearest the ranking reference** — `nominal_joint_positions` when
configured, otherwise the seed (§5). This is what makes the output repeatable:
returning whichever solution an attempt happened to reach first makes nearly
identical requests land on different branches, and on a redundant arm those
branches are entire postures apart.

The consequence is that both backends normally consume the full `max_time`.
Within an attempt the NLopt backend still exits as soon as it is inside
tolerance, but that only means more restarts fit in the budget.

---

## 9. Numerical caching

Every evaluation goes through `updateModel`, which caches by exact
floating-point equality of $q$. FK and frame placements are recomputed only
when $q$ changes; Jacobians and collision distances are each computed at most
once per configuration. Since the objective and the constraint callback are
invoked at the same $q$ within an SLSQP iteration, this roughly halves the
kinematics work.

---

## 10. Choosing a backend

| | `qp` | `nlopt` |
|---|---|---|
| Variable | increment $\Delta q$ | absolute $q$ |
| Collision | linearized per iteration | true nonlinear constraint |
| Regularization | $\lambda$ on the diagonal (step damping) | $\lambda^2$ toward the seed |
| Returns | best of all restarts, ranked by nominal pose | same |
| Strength | mature, well-tested path | constraint fidelity near contact |

Start with `qp` when you care about which of many valid solutions you get —
typically a redundant arm where posture matters. Prefer `nlopt` when you want
the answer as fast as possible and any valid solution will do, or when the
linearized collision constraint is too loose near contact.
