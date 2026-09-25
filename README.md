# wbdopt_ik

Whole-body inverse kinematics for ROS 2 robot groups with one or more end
effectors. Built on Pinocchio, with two interchangeable optimization backends:
a differential QP using qpOASES, and direct nonlinear SLSQP using NLopt.

Both backends solve for all active joints at once under joint limits and
self-collision constraints, so a dual-arm group on a mobile base is one
problem rather than two chained ones.

## Packages

| Package | Contents |
|---|---|
| `wbdopt_ik_lib` | The solver. Pinocchio + qpOASES + NLopt, no ROS dependency. |
| `wbdopt_ik_kinematics_plugin` | MoveIt `KinematicsBase` plugin, including the multi-pose overload dual-arm groups need. |
| `wbdopt_ik` | Metapackage. |

## Documentation

- **[docs/ALGORITHM.md](docs/ALGORITHM.md)** — what the solver computes: cost
  function, collision constraints, both backends, with the math.
- **[docs/PARAMETERS.md](docs/PARAMETERS.md)** — every YAML parameter, what it
  does, and how to tune it.

## Build

Requires Pinocchio, qpOASES, NLopt, MoveIt 2, and `generate_parameter_library`.
Both standard CMake installs and the `/opt/openrobots` robotpkg layout work.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-up-to wbdopt_ik
```

## MoveIt usage

Add the plugin to the planning group in `kinematics.yaml`:

```yaml
whole_body:
  kinematics_solver: wbdopt_ik_kinematics_plugin/WbdoptIkKinematicsPlugin
  kinematics_solver_timeout: 0.05
  solver_type: qp
  tip_frames: [left_fr3_hand_tcp, right_fr3_hand_tcp]
  epsilon: 0.001
  position_weight: 1.0
  orientation_weight: 1.0
  safety_distance: 0.005
  collision_activation_distance: 0.10
```

A complete annotated example is in
[`wbdopt_ik_kinematics_plugin/config/kinematics.yaml`](wbdopt_ik_kinematics_plugin/config/kinematics.yaml);
every parameter is documented in [docs/PARAMETERS.md](docs/PARAMETERS.md).

Three things to get right:

- **Set `tip_frames` explicitly on MoveIt Humble.** Its plugin loader otherwise
  passes only the group's last link, silently reducing a dual-arm group to
  single-arm IK. The order given here is the order of desired poses.
- **Disable permanently-colliding link pairs in the SRDF.** If any enabled pair
  is always in contact, no configuration passes the convergence test and every
  call fails after using the full timeout.
- **All active joints must be one-DoF**, revolute or prismatic. Continuous
  revolute joints are supported and handled as a single angle.

Collision pairs disabled in the SRDF are removed before optimization. Desired
poses are expressed in the URDF root frame.

## Choosing a backend

`qp` collects every solution found before the deadline and returns the one
closest to `nominal_joint_positions`, giving repeatable postures on redundant
arms — at the cost of always consuming the full timeout.

`nlopt` returns as soon as it finds a valid solution, typically in well under a
millisecond, and enforces collision distances as true nonlinear constraints
rather than per-iteration linearizations.

Start with `qp` when posture matters, `nlopt` when latency does. See
[docs/ALGORITHM.md §10](docs/ALGORITHM.md) for the full comparison.

## Library usage

```cpp
#include <wbdopt_ik/wbdopt_ik.hpp>

wbdopt_ik::SolverOptions options;
options.solver = wbdopt_ik::SolverTypes::NLOPT;   // or qpOASES
options.max_time = 0.05;
options.tolerance = 1e-4;

wbdopt_ik::WbdoptIk solver(
  urdf_path, srdf_path,
  {"left_fr3_hand_tcp", "right_fr3_hand_tcp"},    // tips
  active_joint_names,
  options);

Eigen::VectorXd solution;
const int result = solver.CartToJoint(seed, desired_poses, solution);
```

`CartToJoint` takes a seed vector and one `Eigen::Isometry3d` per configured
tip, in the same order. A convenience overload accepts a flat vector of 7
values per tip (`x, y, z, qx, qy, qz, qw`) or 3 for position-only IK. It
returns `SUCCESS`, `FAILED`, or `INVALID_INPUT`; on `FAILED` the solution holds
the closest configuration found.

Construct from URDF/SRDF file paths, or from XML strings with an explicit list
of disabled collision pairs — the form the MoveIt plugin uses.

## Design

`WbdoptIk` is a stable facade over an enum-selected backend. `SolverBase` owns
everything shared: model loading, the reduced configuration mapping, FK, cost,
constraints, and the convergence test. `QpSolver` and `NloptSolver` derive from
it and implement only `solveInternal`.
