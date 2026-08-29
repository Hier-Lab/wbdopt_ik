#include "nlopt_solver.hpp"

#include <nlopt.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <random>
#include <vector>

#ifdef WBDOPT_IK_DEBUG
#include <iostream>
#endif

namespace wbdopt_ik
{
namespace
{

constexpr double kInvalidEvaluationPenalty = 1.0e30;

}  // namespace

double NloptSolver::objective(
  unsigned int variables, const double * values, double * gradient, void * user_data)
{
  auto & context = *static_cast<Context *>(user_data);
  Eigen::Map<const Eigen::VectorXd> q(values, static_cast<Eigen::Index>(variables));
  if (gradient)
  {
    std::fill(gradient, gradient + variables, 0.0);
  }
  if (!q.allFinite())
  {
    return kInvalidEvaluationPenalty;
  }

  try
  {
    const CostData cost_data = context.solver->cost(q, *context.desired);
    const Eigen::VectorXd seed_error = q - *context.seed;
    const double regularization = context.solver->options_.regularization;
    const double regularization_squared = regularization * regularization;
    const double value = cost_data.squared_error +
      regularization_squared * seed_error.squaredNorm();
    if (!std::isfinite(value) || !cost_data.gradient.allFinite())
    {
      return kInvalidEvaluationPenalty;
    }

    if (gradient)
    {
      Eigen::Map<Eigen::VectorXd> output_gradient(
        gradient, static_cast<Eigen::Index>(variables));
      // CostData stores the gradient of one-half the reported squared error.
      output_gradient = 2.0 * cost_data.gradient +
        2.0 * regularization_squared * seed_error;
    }
    return value;
  }
  catch (const std::exception &)
  {
    return kInvalidEvaluationPenalty;
  }
}

void NloptSolver::collisionConstraints(
  unsigned int constraint_count, double * result, unsigned int variables,
  const double * values, double * gradient, void * user_data)
{
  auto & context = *static_cast<Context *>(user_data);
  NloptSolver & solver = *context.solver;
  std::fill(result, result + constraint_count, kInvalidEvaluationPenalty);
  if (gradient)
  {
    std::fill(gradient, gradient + constraint_count * variables, 0.0);
  }
  if (constraint_count != context.collision_pair_ids.size() ||
    variables != solver.joint_names_.size())
  {
    return;
  }

  Eigen::Map<const Eigen::VectorXd> q(values, static_cast<Eigen::Index>(variables));
  if (!q.allFinite())
  {
    return;
  }

  try
  {
    const CollisionEvaluation collision = solver.evaluateCollisionConstraints(
      q, context.collision_pair_ids, gradient != nullptr);
    if (collision.values.size() != static_cast<Eigen::Index>(constraint_count) ||
      !collision.values.allFinite() ||
      (gradient && !collision.gradients.allFinite()))
    {
      return;
    }
    Eigen::Map<Eigen::VectorXd>(result, static_cast<Eigen::Index>(constraint_count)) =
      collision.values;
    if (gradient)
    {
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        output_gradient(
        gradient, static_cast<Eigen::Index>(constraint_count),
        static_cast<Eigen::Index>(variables));
      output_gradient = collision.gradients;
    }
  }
  catch (const std::exception &)
  {
    return;
  }
}

bool NloptSolver::solveAttempt(
  const Eigen::VectorXd & initial, const Eigen::VectorXd & seed,
  const std::vector<Eigen::Isometry3d> & desired, double remaining_time,
  Eigen::VectorXd & candidate, double & objective_value)
{
  const unsigned int variables = static_cast<unsigned int>(joint_names_.size());
  Context context{this, &desired, &seed, {}};
  context.collision_pair_ids = enabledCollisionPairIds();

  // Configure SLSQP with hard joint bounds and nonlinear collision inequalities.
  nlopt::opt optimizer(nlopt::LD_SLSQP, variables);
  optimizer.set_lower_bounds(lower_);
  optimizer.set_upper_bounds(upper_);
  optimizer.set_min_objective(&NloptSolver::objective, &context);
  if (!context.collision_pair_ids.empty())
  {
    optimizer.add_inequality_mconstraint(
      &NloptSolver::collisionConstraints, &context,
      std::vector<double>(context.collision_pair_ids.size(), options_.tolerance));
  }
  optimizer.set_xtol_abs(options_.tolerance * 0.1);
  optimizer.set_ftol_abs(options_.tolerance * options_.tolerance);
  optimizer.set_maxeval(options_.max_iterations);
  optimizer.set_maxtime(remaining_time);

  std::vector<double> values(initial.data(), initial.data() + initial.size());
  try
  {
    const nlopt::result status = optimizer.optimize(values, objective_value);
    candidate = Eigen::Map<Eigen::VectorXd>(
      values.data(), static_cast<Eigen::Index>(values.size()));
#ifdef WBDOPT_IK_DEBUG
    std::cerr << "WbdoptIk NLopt: status=" << static_cast<int>(status) <<
      " evaluations=" << optimizer.get_numevals() <<
      " objective=" << objective_value << std::endl;
#endif
    return status > 0 && candidate.allFinite();
  }
  catch (const std::exception & exception)
  {
#ifdef WBDOPT_IK_DEBUG
    std::cerr << "WbdoptIk NLopt: " << exception.what() << std::endl;
#endif
    return false;
  }
}

int NloptSolver::solveInternal(
  const Eigen::VectorXd & seed, const std::vector<Eigen::Isometry3d> & desired,
  const Deadline & deadline, Eigen::VectorXd & solution)
{
  std::random_device random_device;
  std::mt19937 generator(random_device());
  Eigen::VectorXd best = seed;
  min_cost_ = std::numeric_limits<double>::infinity();
  bool first_attempt = true;
  bool found_solution = false;

  // Retry nonlinear optimization from random configurations until the shared deadline.
  do
  {
    const Eigen::VectorXd initial = first_attempt ? seed : randomConfiguration(generator);
    first_attempt = false;
    const double remaining_time = std::chrono::duration<double>(
      deadline - std::chrono::steady_clock::now()).count();
    if (remaining_time <= 0.0)
    {
      break;
    }

    Eigen::VectorXd candidate;
    double objective_value = std::numeric_limits<double>::infinity();
    if (solveAttempt(initial, seed, desired, remaining_time, candidate, objective_value) &&
      converged(candidate, desired) && objective_value < min_cost_)
    {
      best = candidate;
      min_cost_ = objective_value;
      found_solution = true;
    }
  } while (std::chrono::steady_clock::now() < deadline);

  solution = best;
  return found_solution ? SUCCESS : FAILED;
}

}  // namespace wbdopt_ik
