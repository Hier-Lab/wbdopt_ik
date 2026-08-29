#ifndef WBDOPT_IK__WBDOPT_IK_HPP_
#define WBDOPT_IK__WBDOPT_IK_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace wbdopt_ik
{

enum Status
{
  SUCCESS = 0,
  FAILED = 1,
  INVALID_INPUT = 2,
};

enum class ModelInput
{
  FILES,
  XML,
};

enum class SolverTypes
{
  qpOASES,
  NLOPT,
};

struct SolverOptions
{
  SolverTypes solver{SolverTypes::qpOASES};
  double max_time{0.05};
  double tolerance{1e-4};
  double regularization{1e-4};
  double position_weight{1.0};
  double orientation_weight{1.0};
  std::string joint_centering_name;
  double joint_centering_weight{0.0};
  double safety_distance{0.02};
  double collision_activation_distance{0.10};
  double max_joint_step{0.20};
  int max_iterations{100};
  std::vector<double> nominal_joint_positions;
};

struct CostData
{
  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient;
  double squared_error{0.0};
};

struct ConstraintData
{
  // Bounds on the QP variable delta_q, including joint limits and max step.
  Eigen::VectorXd update_lower;
  Eigen::VectorXd update_upper;

  // Linearized collision bounds: collision_lower <= collision_matrix * delta_q.
  Eigen::MatrixXd collision_matrix;
  Eigen::VectorXd collision_lower;
  Eigen::VectorXd collision_upper;
};

/**
 * Whole-body sequential quadratic-programming inverse kinematics solver.
 *
 * The optimization variables are the one-DoF joints listed in active_joint_names.
 * If that list is empty, all one-DoF non-fixed Pinocchio joints are used. Bounded
 * and continuous revolute joints are both represented by one scalar angle. Desired
 * poses are expressed in the Pinocchio model (URDF root) frame.
 */
class SolverBase;

SolverTypes solverTypeFromString(const std::string & value);
const char * solverTypeName(SolverTypes solver);

class WbdoptIk
{
public:
  using DisabledCollisionPair = std::pair<std::string, std::string>;

  WbdoptIk(
    const std::string & urdf, const std::string & srdf,
    const std::vector<std::string> & end_effector_frames = {},
    const std::vector<std::string> & active_joint_names = {},
    const SolverOptions & options = SolverOptions(), ModelInput input = ModelInput::FILES,
    const std::vector<std::string> & package_paths = {});

  WbdoptIk(
    const std::string & urdf_xml, const std::vector<std::string> & end_effector_frames,
    const std::vector<std::string> & active_joint_names,
    const std::vector<DisabledCollisionPair> & disabled_collision_pairs,
    const SolverOptions & options = SolverOptions(),
    const std::vector<std::string> & package_paths = {});

  ~WbdoptIk();
  WbdoptIk(WbdoptIk &&) noexcept;
  WbdoptIk & operator=(WbdoptIk &&) noexcept;
  WbdoptIk(const WbdoptIk &) = delete;
  WbdoptIk & operator=(const WbdoptIk &) = delete;

  bool setBounds(const std::vector<double> & lower, const std::vector<double> & upper);
  bool setEndEffectorFrames(const std::vector<std::string> & frame_names);
  void setOptions(const SolverOptions & options);

  const SolverOptions & options() const;
  const std::vector<std::string> & jointNames() const;
  const std::vector<std::string> & endEffectorFrames() const;
  std::size_t numberOfJoints() const;
  double minCost() const;

  CostData formCost(const Eigen::VectorXd & q, const std::vector<Eigen::Isometry3d> & desired_poses);
  ConstraintData formConstraints(const Eigen::VectorXd & q);

  int CartToJoint(
    const Eigen::VectorXd & q_init, 
    const std::vector<Eigen::Isometry3d> & desired_poses,
    Eigen::VectorXd & q_solution);

  /**
   * Convenience overload. p_desired is either 7 values per tip
   * [x,y,z,qx,qy,qz,qw], or 3 values per tip for position-only IK.
   */
  int CartToJoint(
    const Eigen::VectorXd & q_init, 
    const Eigen::VectorXd & p_desired,
    Eigen::VectorXd & q_solution);

private:
  std::unique_ptr<SolverBase> solver_;
};

}  // namespace wbdopt_ik

#endif  // WBDOPT_IK__WBDOPT_IK_HPP_
