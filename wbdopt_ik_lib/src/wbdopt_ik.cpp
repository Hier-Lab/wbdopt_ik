#include <wbdopt_ik/wbdopt_ik.hpp>

#include "solver_base.hpp"
#include "solver_factory.hpp"

#include <stdexcept>

namespace wbdopt_ik
{

SolverTypes solverTypeFromString(const std::string & value)
{
  if (value == "qp" || value == "qpoases")
  {
    return SolverTypes::qpOASES;
  }
  if (value == "nlopt")
  {
    return SolverTypes::NLOPT;
  }
  throw std::invalid_argument("Unknown WbdoptIk solver type: " + value);
}

const char * solverTypeName(SolverTypes solver)
{
  switch (solver)
  {
    case SolverTypes::qpOASES:
      return "qp";
    case SolverTypes::NLOPT:
      return "nlopt";
  }
  return "unknown";
}

WbdoptIk::WbdoptIk(
  const std::string & urdf, const std::string & srdf,
  const std::vector<std::string> & end_effector_frames,
  const std::vector<std::string> & active_joint_names, const SolverOptions & options,
  ModelInput input, const std::vector<std::string> & package_paths)
: solver_(makeSolver(
      urdf, srdf, end_effector_frames, active_joint_names, options, input, package_paths))
{
}

WbdoptIk::WbdoptIk(
  const std::string & urdf_xml, const std::vector<std::string> & end_effector_frames,
  const std::vector<std::string> & active_joint_names,
  const std::vector<DisabledCollisionPair> & disabled_collision_pairs,
  const SolverOptions & options, const std::vector<std::string> & package_paths)
: solver_(makeSolver(
      urdf_xml, end_effector_frames, active_joint_names, disabled_collision_pairs, options,
      package_paths))
{
}

WbdoptIk::~WbdoptIk() = default;
WbdoptIk::WbdoptIk(WbdoptIk &&) noexcept = default;
WbdoptIk & WbdoptIk::operator=(WbdoptIk &&) noexcept = default;

bool WbdoptIk::setBounds(
  const std::vector<double> & lower, const std::vector<double> & upper)
{
  return solver_->setBounds(lower, upper);
}

bool WbdoptIk::setEndEffectorFrames(const std::vector<std::string> & frame_names)
{
  return solver_->setEndEffectors(frame_names);
}

void WbdoptIk::setOptions(const SolverOptions & options)
{
  solver_->setOptions(options);
}

const SolverOptions & WbdoptIk::options() const {return solver_->options();}
const std::vector<std::string> & WbdoptIk::jointNames() const {return solver_->jointNames();}
const std::vector<std::string> & WbdoptIk::endEffectorFrames() const
{
  return solver_->endEffectorFrames();
}
std::size_t WbdoptIk::numberOfJoints() const {return solver_->numberOfJoints();}
double WbdoptIk::minCost() const {return solver_->minCost();}

CostData WbdoptIk::formCost(
  const Eigen::VectorXd & q, const std::vector<Eigen::Isometry3d> & desired_poses)
{
  return solver_->formCost(q, desired_poses);
}

ConstraintData WbdoptIk::formConstraints(const Eigen::VectorXd & q)
{
  return solver_->formConstraints(q);
}

int WbdoptIk::CartToJoint(
  const Eigen::VectorXd & q_init, const std::vector<Eigen::Isometry3d> & desired_poses,
  Eigen::VectorXd & q_solution)
{
  return solver_->solve(q_init, desired_poses, q_solution);
}

int WbdoptIk::CartToJoint(
  const Eigen::VectorXd & q_init, const Eigen::VectorXd & p_desired,
  Eigen::VectorXd & q_solution)
{
  return solver_->solve(q_init, p_desired, q_solution);
}

}  // namespace wbdopt_ik
