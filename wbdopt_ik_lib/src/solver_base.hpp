#ifndef WBDOPT_IK__SOLVER_BASE_HPP_
#define WBDOPT_IK__SOLVER_BASE_HPP_

#include <wbdopt_ik/wbdopt_ik.hpp>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <chrono>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace wbdopt_ik
{

class SolverBase
{
public:
  using DisabledCollisionPair = WbdoptIk::DisabledCollisionPair;
  using Deadline = std::chrono::steady_clock::time_point;

  SolverBase(
    const std::string & urdf, const std::string & srdf,
    const std::vector<std::string> & end_effector_frames,
    const std::vector<std::string> & active_joint_names, const SolverOptions & options,
    ModelInput input, const std::vector<std::string> & package_paths);

  SolverBase(
    const std::string & urdf_xml, const std::vector<std::string> & end_effector_frames,
    const std::vector<std::string> & active_joint_names,
    const std::vector<DisabledCollisionPair> & disabled_collision_pairs,
    const SolverOptions & options, const std::vector<std::string> & package_paths);

  virtual ~SolverBase() = default;

  virtual SolverTypes type() const noexcept = 0;

  bool setBounds(const std::vector<double> & lower, const std::vector<double> & upper);
  bool setEndEffectors(const std::vector<std::string> & frame_names);
  void setOptions(const SolverOptions & options);

  const SolverOptions & options() const noexcept {return options_;}
  const std::vector<std::string> & jointNames() const noexcept {return joint_names_;}
  const std::vector<std::string> & endEffectorFrames() const noexcept
  {
    return end_effector_names_;
  }
  std::size_t numberOfJoints() const noexcept {return joint_names_.size();}
  double minCost() const noexcept {return min_cost_;}

  CostData formCost(
    const Eigen::VectorXd & q, const std::vector<Eigen::Isometry3d> & desired);
  ConstraintData formConstraints(const Eigen::VectorXd & q);
  int solve(
    const Eigen::VectorXd & seed, const std::vector<Eigen::Isometry3d> & desired,
    Eigen::VectorXd & solution);
  int solve(
    const Eigen::VectorXd & seed, const Eigen::VectorXd & desired,
    Eigen::VectorXd & solution);

protected:
  static constexpr double kInfinity = 1.0e20;

  struct CollisionEvaluation
  {
    Eigen::VectorXd values;
    Eigen::MatrixXd gradients;
  };

  // A converged solution together with the two keys used to rank it against the
  // other solutions found from different restarts.
  struct Candidate
  {
    Eigen::VectorXd configuration;
    double task_error;
    double nominal_distance;
  };

  void finishInitialization(
    const std::vector<std::string> & end_effector_frames,
    const std::vector<std::string> & requested_joint_names);
  void validateOptions(const SolverOptions & options) const;

  void updateModel(const Eigen::VectorXd & active_q);
  Eigen::VectorXd toFullConfiguration(const Eigen::VectorXd & active_q) const;
  Eigen::MatrixXd activeColumns(const Eigen::MatrixXd & full_jacobian) const;
  Eigen::MatrixXd pointJacobian(pinocchio::JointIndex joint_id, const Eigen::Vector3d & world_point) const;
  CostData cost(const Eigen::VectorXd & active_q, const std::vector<Eigen::Isometry3d> & desired);
  ConstraintData constraints(const Eigen::VectorXd & active_q);
  const std::vector<std::size_t> & enabledCollisionPairIds() const noexcept;
  CollisionEvaluation evaluateCollisionConstraints(
    const Eigen::VectorXd & active_q, const std::vector<std::size_t> & pair_ids,
    bool compute_gradients);
  bool collisionSafe(const Eigen::VectorXd & active_q);
  bool converged(const Eigen::VectorXd & active_q, const std::vector<Eigen::Isometry3d> & desired);
  Eigen::VectorXd randomConfiguration(std::mt19937 & generator) const;

  // Ranking reference for restart candidates: the configured nominal pose when
  // one is set, otherwise the request seed.
  Eigen::VectorXd rankingReference(const Eigen::VectorXd & seed) const;
  double nominalDistanceSquared(
    const Eigen::VectorXd & configuration, const Eigen::VectorXd & nominal) const;
  // Orders candidates by distance to the ranking reference, task error breaking ties.
  static bool moreNominal(const Candidate & left, const Candidate & right);

  virtual int solveInternal(
    const Eigen::VectorXd & seed, const std::vector<Eigen::Isometry3d> & desired,
    const Deadline & deadline, Eigen::VectorXd & solution) = 0;

  pinocchio::Model model_;
  pinocchio::GeometryModel geometry_model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::unique_ptr<pinocchio::GeometryData> geometry_data_;
  Eigen::VectorXd q_reference_;
  std::vector<std::string> joint_names_;
  std::vector<pinocchio::JointIndex> q_indices_;
  std::vector<int> q_dimensions_;
  std::vector<pinocchio::JointIndex> v_indices_;
  std::vector<double> lower_;
  std::vector<double> upper_;
  std::vector<std::string> end_effector_names_;
  std::vector<pinocchio::FrameIndex> end_effector_ids_;
  std::vector<std::size_t> enabled_collision_pair_ids_;
  SolverOptions options_;
  double min_cost_{std::numeric_limits<double>::infinity()};

  Eigen::VectorXd cached_active_q_;
  bool model_cache_valid_{false};
  bool jacobian_cache_valid_{false};
  bool distance_cache_valid_{false};
};

}  // namespace wbdopt_ik

#endif  // WBDOPT_IK__SOLVER_BASE_HPP_
