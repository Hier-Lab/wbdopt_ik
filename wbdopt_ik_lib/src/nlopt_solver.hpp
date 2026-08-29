#ifndef WBDOPT_IK__NLOPT_SOLVER_HPP_
#define WBDOPT_IK__NLOPT_SOLVER_HPP_

#include "solver_base.hpp"

namespace wbdopt_ik
{

class NloptSolver final : public SolverBase
{
public:
  using SolverBase::SolverBase;

  SolverTypes type() const noexcept override {return SolverTypes::NLOPT;}

private:
  struct Context
  {
    NloptSolver * solver;
    const std::vector<Eigen::Isometry3d> * desired;
    const Eigen::VectorXd * seed;
    std::vector<std::size_t> collision_pair_ids;
  };

  static double objective(
    unsigned int variables, const double * values, double * gradient, void * user_data);
  static void collisionConstraints(
    unsigned int constraint_count, double * result, unsigned int variables,
    const double * values, double * gradient, void * user_data);

  bool solveAttempt(
    const Eigen::VectorXd & initial, const Eigen::VectorXd & seed,
    const std::vector<Eigen::Isometry3d> & desired, double remaining_time,
    Eigen::VectorXd & candidate, double & objective_value);
  int solveInternal(
    const Eigen::VectorXd & seed, const std::vector<Eigen::Isometry3d> & desired,
    const Deadline & deadline, Eigen::VectorXd & solution) override;
};

}  // namespace wbdopt_ik

#endif  // WBDOPT_IK__NLOPT_SOLVER_HPP_
