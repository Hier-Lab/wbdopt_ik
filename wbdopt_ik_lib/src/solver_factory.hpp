#ifndef WBDOPT_IK__SOLVER_FACTORY_HPP_
#define WBDOPT_IK__SOLVER_FACTORY_HPP_

#include "solver_base.hpp"

namespace wbdopt_ik
{

std::unique_ptr<SolverBase> makeSolver(
  const std::string & urdf, const std::string & srdf,
  const std::vector<std::string> & end_effector_frames,
  const std::vector<std::string> & active_joint_names, const SolverOptions & options,
  ModelInput input, const std::vector<std::string> & package_paths);

std::unique_ptr<SolverBase> makeSolver(
  const std::string & urdf_xml, const std::vector<std::string> & end_effector_frames,
  const std::vector<std::string> & active_joint_names,
  const std::vector<WbdoptIk::DisabledCollisionPair> & disabled_collision_pairs,
  const SolverOptions & options, const std::vector<std::string> & package_paths);

}  // namespace wbdopt_ik

#endif  // WBDOPT_IK__SOLVER_FACTORY_HPP_
