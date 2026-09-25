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

    // Best point seen by the objective during this attempt, latched the moment the
    // pose error drops under tolerance. nlopt::forced_stop unwinds without writing
    // back to the caller's x, so the early exit must keep its own copy.
    Eigen::VectorXd best_q;
    double best_pose_error{std::numeric_limits<double>::infinity()};
    bool converged{false};
  };

  static double objective(
    unsigned int variables, const double * values, double * gradient, void * user_data);
  static void collisionConstraints(
    unsigned int constraint_count, double * result, unsigned int variables,
    const double * values, double * gradient, void * user_data);

  std::vector<std::size_t> screenCollisionPairs(const Eigen::VectorXd & active_q);

  bool solveAttempt(
    const Eigen::VectorXd & initial, const Eigen::VectorXd & seed,
    const std::vector<Eigen::Isometry3d> & desired, double remaining_time,
    Eigen::VectorXd & candidate, double & pose_error);
  int solveInternal(
    const Eigen::VectorXd & seed, const std::vector<Eigen::Isometry3d> & desired,
    const Deadline & deadline, Eigen::VectorXd & solution) override;
};

}  // namespace wbdopt_ik

#endif  // WBDOPT_IK__NLOPT_SOLVER_HPP_
