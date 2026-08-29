# wbdopt_ik

`wbdopt_ik` is a ROS 2 whole-body inverse-kinematics solver for robot groups
with one or more end effectors. It supports a differential-QP backend using
qpOASES and a direct nonlinear SLSQP backend using NLopt.

The repository contains three ament packages:

- `wbdopt_ik_lib`: ROS-independent Pinocchio/qpOASES/NLopt solver library.
- `wbdopt_ik_kinematics_plugin`: MoveIt `KinematicsBase` plugin, including the
  multi-pose overload needed by dual-arm groups.
- `wbdopt_ik`: convenience metapackage.

## Build

Install Pinocchio, qpOASES, NLopt, MoveIt 2, and `generate_parameter_library`, then run
from the ROS workspace root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-up-to wbdopt_ik
```

The CMake configuration supports normal CMake package installations and the
`/opt/openrobots` layout used by robotpkg.

## MoveIt configuration

Use the plugin class in the planning group's `kinematics.yaml`:

```yaml
whole_body:
  kinematics_solver: wbdopt_ik_kinematics_plugin/WbdoptIkKinematicsPlugin
  kinematics_solver_timeout: 0.05
  solver_type: qp
  tip_frames: [left_fr3_hand_tcp, right_fr3_hand_tcp]
  epsilon: 0.001
  regularization: 0.0001
  position_weight: 1.0
  orientation_weight: 1.0
  joint_centering_name: base_slider_joint
  joint_centering_weight: 0.1
  safety_distance: 0.02
  collision_activation_distance: 0.10
  max_joint_step: 0.20
  max_iterations: 100
  nominal_joint_names: [base_slider_joint,
    left_fr3_joint1, left_fr3_joint2, left_fr3_joint3, left_fr3_joint4,
    left_fr3_joint5, left_fr3_joint6, left_fr3_joint7,
    right_fr3_joint1, right_fr3_joint2, right_fr3_joint3, right_fr3_joint4,
    right_fr3_joint5, right_fr3_joint6, right_fr3_joint7]
  nominal_joint_positions: [0.0,
    0.0, -0.7853981633974483, 0.0, -2.356194490192345,
    0.0, 1.5707963267948966, 0.7853981633974483,
    0.0, -0.7853981633974483, 0.0, -2.356194490192345,
    0.0, 1.5707963267948966, 0.7853981633974483]
```

All active joints must currently be one-DoF revolute or prismatic joints (both
bounded and continuous revolute joints are supported). The
tip-frame order configured by MoveIt is also the desired-pose order passed to
the multi-tip IK call. Collision pairs disabled in the SRDF are removed before
optimization.

`solver_type` selects the optimization backend. The default `qp` backend keeps
the differential IK loop and solves each local QP with qpOASES. The `nlopt`
backend uses NLopt SLSQP to optimize the nonlinear Cartesian objective directly
with joint bounds and enabled self-collision distance constraints. NLopt can
require a longer `kinematics_solver_timeout`; start with `0.2` seconds for a
15-joint, two-tip group and tune it for the robot model.

Internally, `WbdoptIk` is a stable facade over an enum-selected solver state.
`SolverBase` owns the shared model loading, configuration mapping, FK, cost,
constraint, and convergence procedures. `QpSolver` and `NloptSolver` derive
from it and implement only their backend solve procedure in separate files.

On MoveIt Humble, configure `tip_frames` explicitly because its standard
kinematics plugin loader otherwise passes only the last link of the group.

The optional joint-centering objective biases one active joint toward the
midpoint of its configured bounds. Set `joint_centering_weight` to zero (the
default) or leave `joint_centering_name` empty to disable it. The objective is
`weight * (q - q_middle)^2 / q_middle^2`; when `q_middle` is zero, as for the
`base_slider_joint` bounds `[-0.5, 0.5]`, the squared half-range is used as the
normalizer to avoid division by zero.

The QP backend continues random restarts until its deadline. Every converged
solution is retained and sorted by squared joint distance to the configured
nominal pose; continuous-joint differences use their shortest angular distance.
The nominal name/value arrays are mapped into the MoveIt group's active-joint
order during plugin initialization, so their YAML order does not need to match
the model order. If no nominal pose is configured, the request seed is used as
the ranking reference.

For direct library use, include `<wbdopt_ik/wbdopt_ik.hpp>`, construct
`wbdopt_ik::WbdoptIk` from URDF/SRDF files or XML, and call `CartToJoint` with a
seed vector and one `Eigen::Isometry3d` target per configured tip.

Select the direct-library backend before construction:

```cpp
wbdopt_ik::SolverOptions options;
options.solver = wbdopt_ik::SolverTypes::NLOPT;
```
