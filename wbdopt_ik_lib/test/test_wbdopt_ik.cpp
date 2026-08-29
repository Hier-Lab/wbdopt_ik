#include <gtest/gtest.h>
#include <wbdopt_ik/wbdopt_ik.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{

const char kPrismaticUrdf[] = R"(
<robot name="slider">
  <link name="base"/>
  <joint name="slide" type="prismatic">
    <parent link="base"/>
    <child link="tool"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1" effort="10" velocity="1"/>
  </joint>
  <link name="tool"/>
</robot>)";

const char kContinuousUrdf[] = R"(
<robot name="continuous">
  <link name="base"/>
  <joint name="turn" type="continuous">
    <parent link="base"/>
    <child link="tool"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="tool"/>
</robot>)";

const char kRedundantPrismaticUrdf[] = R"(
<robot name="redundant_slider">
  <link name="base"/>
  <joint name="slide_1" type="prismatic">
    <parent link="base"/><child link="middle"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="1" effort="10" velocity="1"/>
  </joint>
  <link name="middle"/>
  <joint name="slide_2" type="prismatic">
    <parent link="middle"/><child link="tool"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="1" effort="10" velocity="1"/>
  </joint>
  <link name="tool"/>
</robot>)";

const char kCollisionUrdf[] = R"(
<robot name="collision_rows">
  <link name="base"/>
  <joint name="slide_a" type="prismatic">
    <parent link="base"/><child link="a"/><axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
  <link name="a"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <joint name="slide_b" type="prismatic">
    <parent link="base"/><child link="b"/><origin xyz="0.45 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
  <link name="b"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
</robot>)";

const char kCylinderCollisionUrdf[] = R"(
<robot name="cylinder_replacement">
  <link name="base">
    <collision><geometry><cylinder radius="0.1" length="1.0"/></geometry></collision>
  </link>
  <joint name="slide" type="prismatic">
    <parent link="base"/><child link="probe"/><origin xyz="0.2 0 0.5"/><axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
  <link name="probe">
    <collision><geometry><sphere radius="0.02"/></geometry></collision>
  </link>
</robot>)";

TEST(WbdoptIk, SolvesSinglePrismaticJoint)
{
  wbdopt_ik::SolverOptions options;
  options.max_time = 0.1;
  options.tolerance = 1e-6;
  options.safety_distance = 0.0;
  options.collision_activation_distance = 0.0;
  options.max_joint_step = 0.25;

  wbdopt_ik::WbdoptIk solver(
    kPrismaticUrdf, std::vector<std::string>{"tool"},
    std::vector<std::string>{"slide"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{}, options);

  Eigen::VectorXd seed(1);
  seed << 0.0;
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation().x() = 0.4;
  Eigen::VectorXd solution;

  EXPECT_EQ(solver.CartToJoint(seed, std::vector<Eigen::Isometry3d>{target}, solution),
    wbdopt_ik::SUCCESS);
  ASSERT_EQ(solution.size(), 1);
  EXPECT_NEAR(solution[0], 0.4, 1e-5);
}

TEST(WbdoptIk, SolvesSinglePrismaticJointWithNlopt)
{
  wbdopt_ik::SolverOptions options;
  options.solver = wbdopt_ik::SolverTypes::NLOPT;
  options.max_time = 0.2;
  options.max_iterations = 100;
  options.tolerance = 1e-6;
  options.safety_distance = 0.0;
  options.collision_activation_distance = 0.0;

  wbdopt_ik::WbdoptIk solver(
    kPrismaticUrdf, std::vector<std::string>{"tool"},
    std::vector<std::string>{"slide"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{}, options);

  Eigen::VectorXd seed = Eigen::VectorXd::Zero(1);
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation().x() = 0.4;
  Eigen::VectorXd solution;

  EXPECT_EQ(solver.CartToJoint(seed, {target}, solution), wbdopt_ik::SUCCESS);
  ASSERT_EQ(solution.size(), 1);
  EXPECT_NEAR(solution[0], 0.4, 1e-5);
}

TEST(WbdoptIk, NloptHandlesInitiallyInactiveCollisionConstraint)
{
  wbdopt_ik::SolverOptions options;
  options.solver = wbdopt_ik::SolverTypes::NLOPT;
  options.max_time = 0.1;
  options.max_iterations = 100;
  options.tolerance = 1e-6;
  options.safety_distance = 0.02;
  options.collision_activation_distance = 0.10;

  wbdopt_ik::WbdoptIk solver(
    kCollisionUrdf, std::vector<std::string>{"a"},
    std::vector<std::string>{"slide_a", "slide_b"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{}, options);

  // The pair starts outside the QP activation range, but NLopt must still
  // provide one initialized value and gradient row for its fixed constraint.
  const Eigen::VectorXd seed = Eigen::VectorXd::Zero(2);
  const Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  Eigen::VectorXd solution;

  EXPECT_EQ(solver.CartToJoint(seed, {target}, solution), wbdopt_ik::SUCCESS);
  EXPECT_TRUE(solution.allFinite());
}

TEST(WbdoptIk, RejectsNonFiniteDesiredPose)
{
  wbdopt_ik::WbdoptIk solver(
    kPrismaticUrdf, std::vector<std::string>{"tool"},
    std::vector<std::string>{"slide"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{});

  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.linear()(0, 0) = std::numeric_limits<double>::quiet_NaN();
  Eigen::VectorXd solution;

  EXPECT_EQ(
    solver.CartToJoint(Eigen::VectorXd::Zero(1), {target}, solution),
    wbdopt_ik::INVALID_INPUT);
}

TEST(WbdoptIk, RanksRestartSolutionsByNominalPose)
{
  wbdopt_ik::SolverOptions options;
  options.max_time = 0.05;
  options.max_joint_step = 0.5;
  options.tolerance = 1e-6;
  options.safety_distance = 0.0;
  options.collision_activation_distance = 0.0;
  options.nominal_joint_positions = {0.0, 0.4};
  wbdopt_ik::WbdoptIk solver(
    kRedundantPrismaticUrdf, std::vector<std::string>{"tool"},
    std::vector<std::string>{"slide_1", "slide_2"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{}, options);

  Eigen::VectorXd seed(2);
  seed << 0.4, 0.0;
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation().x() = 0.4;
  Eigen::VectorXd solution;

  ASSERT_EQ(solver.CartToJoint(seed, {target}, solution), wbdopt_ik::SUCCESS);
  EXPECT_NEAR(solution.sum(), 0.4, 1e-5);
  const Eigen::Vector2d nominal(0.0, 0.4);
  EXPECT_LT((solution - nominal).squaredNorm(), (seed - nominal).squaredNorm());
}

TEST(WbdoptIk, RejectsWrongNominalPoseSize)
{
  wbdopt_ik::SolverOptions options;
  options.nominal_joint_positions = {0.0, 0.4};
  EXPECT_THROW(
    wbdopt_ik::WbdoptIk(
      kPrismaticUrdf, std::vector<std::string>{"tool"},
      std::vector<std::string>{"slide"},
      std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{}, options),
    std::invalid_argument);
}

TEST(WbdoptIk, ParsesSolverStateNames)
{
  EXPECT_EQ(wbdopt_ik::solverTypeFromString("qp"), wbdopt_ik::SolverTypes::qpOASES);
  EXPECT_EQ(wbdopt_ik::solverTypeFromString("nlopt"), wbdopt_ik::SolverTypes::NLOPT);
  EXPECT_STREQ(wbdopt_ik::solverTypeName(wbdopt_ik::SolverTypes::NLOPT), "nlopt");
  EXPECT_THROW(wbdopt_ik::solverTypeFromString("invalid"), std::invalid_argument);
}

TEST(WbdoptIk, RejectsBackendChangeAfterConstruction)
{
  wbdopt_ik::WbdoptIk solver(
    kPrismaticUrdf, std::vector<std::string>{"tool"},
    std::vector<std::string>{"slide"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{});
  wbdopt_ik::SolverOptions options = solver.options();
  options.solver = wbdopt_ik::SolverTypes::NLOPT;
  EXPECT_THROW(solver.setOptions(options), std::invalid_argument);
}

TEST(WbdoptIk, RejectsWrongSeedSize)
{
  wbdopt_ik::WbdoptIk solver(
    kPrismaticUrdf, std::vector<std::string>{"tool"},
    std::vector<std::string>{"slide"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{});
  Eigen::VectorXd solution;
  EXPECT_EQ(
    solver.CartToJoint(Eigen::VectorXd(), std::vector<Eigen::Isometry3d>{Eigen::Isometry3d::Identity()},
    solution),
    wbdopt_ik::INVALID_INPUT);
}

TEST(WbdoptIk, SupportsContinuousRevoluteJoint)
{
  wbdopt_ik::SolverOptions options;
  options.max_time = 0.05;
  options.tolerance = 1e-6;
  options.safety_distance = 0.0;
  options.collision_activation_distance = 0.0;
  wbdopt_ik::WbdoptIk solver(
    kContinuousUrdf, std::vector<std::string>{"tool"},
    std::vector<std::string>{"turn"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{}, options);

  Eigen::VectorXd seed = Eigen::VectorXd::Zero(1);
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::VectorXd solution;
  EXPECT_EQ(solver.CartToJoint(seed, {target}, solution), wbdopt_ik::SUCCESS);
  ASSERT_EQ(solution.size(), 1);
  EXPECT_NEAR(solution[0], 0.4, 1e-5);
}

TEST(WbdoptIk, FormsCollisionDistanceConstraint)
{
  wbdopt_ik::SolverOptions options;
  options.safety_distance = 0.30;
  options.collision_activation_distance = 1.0;
  wbdopt_ik::WbdoptIk solver(
    kCollisionUrdf, std::vector<std::string>{"a"},
    std::vector<std::string>{"slide_a", "slide_b"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{}, options);

  const auto constraints = solver.formConstraints(Eigen::VectorXd::Zero(2));
  ASSERT_EQ(constraints.update_lower.size(), 2);
  ASSERT_EQ(constraints.update_upper.size(), 2);
  EXPECT_NEAR(constraints.update_lower[0], -0.2, 1e-12);
  EXPECT_NEAR(constraints.update_upper[0], 0.2, 1e-12);
  ASSERT_EQ(constraints.collision_matrix.rows(), 1);
  ASSERT_EQ(constraints.collision_matrix.cols(), 2);
  EXPECT_NEAR(constraints.collision_matrix(0, 0), -1.0, 1e-6);
  EXPECT_NEAR(constraints.collision_matrix(0, 1), 1.0, 1e-6);
  EXPECT_NEAR(constraints.collision_lower[0], 0.05, 1e-6);
}

TEST(WbdoptIk, SeparatesUpdateBoundsAtJointLimit)
{
  wbdopt_ik::SolverOptions options;
  options.max_joint_step = 0.2;
  options.safety_distance = 0.0;
  options.collision_activation_distance = 0.0;
  wbdopt_ik::WbdoptIk solver(
    kPrismaticUrdf, std::vector<std::string>{"tool"},
    std::vector<std::string>{"slide"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{}, options);

  Eigen::VectorXd q(1);
  q << 1.0;
  const auto constraints = solver.formConstraints(q);
  EXPECT_NEAR(constraints.update_lower[0], -0.2, 1e-12);
  EXPECT_NEAR(constraints.update_upper[0], 0.0, 1e-12);
  EXPECT_EQ(constraints.collision_matrix.rows(), 0);
}

TEST(WbdoptIk, ReplacesCylinderWithExtentPreservingCapsule)
{
  wbdopt_ik::SolverOptions options;
  options.safety_distance = 0.0;
  options.collision_activation_distance = 1.0;
  wbdopt_ik::WbdoptIk solver(
    kCylinderCollisionUrdf, std::vector<std::string>{"probe"},
    std::vector<std::string>{"slide"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{}, options);

  const auto constraints = solver.formConstraints(Eigen::VectorXd::Zero(1));
  ASSERT_EQ(constraints.collision_lower.size(), 1);
  // Capsule centerline ends at z=0.4, so distance is hypot(0.2, 0.1)-0.1-0.02.
  const double expected_distance = std::hypot(0.2, 0.1) - 0.12;
  EXPECT_NEAR(constraints.collision_lower[0], -expected_distance, 1e-6);
}

TEST(WbdoptIk, AddsNormalizedJointCenteringCost)
{
  wbdopt_ik::SolverOptions options;
  options.regularization = 0.0;
  options.position_weight = 0.0;
  options.orientation_weight = 0.0;
  options.joint_centering_name = "slide";
  options.joint_centering_weight = 2.0;
  options.safety_distance = 0.0;
  options.collision_activation_distance = 0.0;
  wbdopt_ik::WbdoptIk solver(
    kPrismaticUrdf, std::vector<std::string>{"tool"},
    std::vector<std::string>{"slide"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{}, options);

  const Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
  const auto cost = solver.formCost(q, {Eigen::Isometry3d::Identity()});
  // Bounds [0, 1] give middle=0.5 and denominator=0.25.
  EXPECT_NEAR(cost.hessian(0, 0), 16.0, 1e-12);
  EXPECT_NEAR(cost.gradient[0], -8.0, 1e-12);
  EXPECT_NEAR(cost.squared_error, 2.0, 1e-12);
}

TEST(WbdoptIk, CentersJointWithZeroMidpointWithoutDivisionByZero)
{
  wbdopt_ik::SolverOptions options;
  options.regularization = 0.0;
  options.position_weight = 0.0;
  options.orientation_weight = 0.0;
  options.joint_centering_name = "slide_a";
  options.joint_centering_weight = 2.0;
  options.safety_distance = 0.0;
  options.collision_activation_distance = 0.0;
  wbdopt_ik::WbdoptIk solver(
    kCollisionUrdf, std::vector<std::string>{"a"},
    std::vector<std::string>{"slide_a", "slide_b"},
    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair>{}, options);

  Eigen::VectorXd q(2);
  q << 0.25, 0.0;
  const auto cost = solver.formCost(q, {Eigen::Isometry3d::Identity()});
  // Symmetric bounds [-1, 1] give middle=0 and half-range normalization=1.
  EXPECT_NEAR(cost.hessian(0, 0), 4.0, 1e-12);
  EXPECT_NEAR(cost.gradient[0], 1.0, 1e-12);
  EXPECT_NEAR(cost.squared_error, 0.125, 1e-12);
}

}  // namespace
