#include "solver_factory.hpp"

#include "nlopt_solver.hpp"
#include "qp_solver.hpp"

#include <stdexcept>

namespace wbdopt_ik
{
namespace
{

template<typename... Args>
std::unique_ptr<SolverBase> makeSelectedSolver(SolverTypes type, Args && ... args)
{
  // The enum is the state selector; each state owns only its backend solve behavior.
  switch (type)
  {
    case SolverTypes::qpOASES:
      return std::make_unique<QpSolver>(std::forward<Args>(args)...);
    case SolverTypes::NLOPT:
      return std::make_unique<NloptSolver>(std::forward<Args>(args)...);
  }
  throw std::invalid_argument("Unknown WbdoptIk solver type");
}

}  // namespace

std::unique_ptr<SolverBase> makeSolver(
  const std::string & urdf, const std::string & srdf,
  const std::vector<std::string> & end_effector_frames,
  const std::vector<std::string> & active_joint_names, const SolverOptions & options,
  ModelInput input, const std::vector<std::string> & package_paths)
{
  return makeSelectedSolver(
    options.solver, urdf, srdf, end_effector_frames, active_joint_names, options, input,
    package_paths);
}

std::unique_ptr<SolverBase> makeSolver(
  const std::string & urdf_xml, const std::vector<std::string> & end_effector_frames,
  const std::vector<std::string> & active_joint_names,
  const std::vector<WbdoptIk::DisabledCollisionPair> & disabled_collision_pairs,
  const SolverOptions & options, const std::vector<std::string> & package_paths)
{
  return makeSelectedSolver(
    options.solver, urdf_xml, end_effector_frames, active_joint_names,
    disabled_collision_pairs, options, package_paths);
}

}  // namespace wbdopt_ik
