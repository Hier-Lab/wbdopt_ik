#ifndef WBDOPT_IK__QP_SOLVER_HPP_
#define WBDOPT_IK__QP_SOLVER_HPP_

#include "solver_base.hpp"

namespace wbdopt_ik
{

class QpSolver final : public SolverBase
{
public:
  using SolverBase::SolverBase;

  SolverTypes type() const noexcept override {return SolverTypes::qpOASES;}

private:
  struct Candidate
  {
    Eigen::VectorXd configuration;
    double task_error;
    double nominal_distance;
  };
  double nominalDistanceSquared(
    const Eigen::VectorXd & configuration, const Eigen::VectorXd & nominal) const;
  bool solveQp(
    const CostData & cost_data, const ConstraintData & constraint_data,
    Eigen::VectorXd & delta) const;
  int solveInternal(
    const Eigen::VectorXd & seed, const std::vector<Eigen::Isometry3d> & desired,
    const Deadline & deadline, Eigen::VectorXd & solution) override;
};

}  // namespace wbdopt_ik

#endif  // WBDOPT_IK__QP_SOLVER_HPP_
