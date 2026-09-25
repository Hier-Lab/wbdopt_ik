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

// A constraint row that could not be evaluated must not look like a violation: for
// h(q) <= tol, a large positive value tells SLSQP the whole region is infeasible.
constexpr double kInactiveConstraintValue = -1.0e20;

// Skip a restart whose slice of the deadline is too short to run a useful number of
// SLSQP iterations; such an attempt only returns noise.
constexpr double kMinimumAttemptTime = 1.0e-3;

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
      // CostData stores the gradient of one-half the pose and centering terms, so
      // those need the factor of two. The regularization term is accumulated here at
      // full scale, and d/dq of reg^2 * ||q - seed||^2 is already 2 * reg^2 * error.
      output_gradient = 2.0 * cost_data.gradient +
        2.0 * regularization_squared * seed_error;
    }

    // Track the best point by pose error alone. The regularization term measures
    // distance from the seed, which would otherwise penalize a restart for being far
    // from the seed even when it reaches the target more accurately.
    const double pose_error = cost_data.squared_error;
    if (pose_error < context.best_pose_error)
    {
      context.best_pose_error = pose_error;
      context.best_q = q;
    }

    // Stop as soon as the pose is within tolerance and the configuration is clear of
    // collisions; SLSQP would otherwise keep polishing a solution that is already
    // good enough, consuming the whole time budget.
    const double tolerance = context.solver->options_.tolerance;
    if (!context.converged && std::sqrt(pose_error) <= tolerance &&
      context.solver->collisionSafe(q))
    {
      context.best_pose_error = pose_error;
      context.best_q = q;
      context.converged = true;
      throw nlopt::forced_stop();
    }
    return value;
  }
  catch (const nlopt::forced_stop &)
  {
    throw;
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
  // A row that cannot be evaluated is reported as trivially satisfied rather than
  // violated, matching how evaluateCollisionConstraints handles a failed pair.
  std::fill(result, result + constraint_count, kInactiveConstraintValue);
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

std::vector<std::size_t> NloptSolver::screenCollisionPairs(const Eigen::VectorXd & active_q)
{
  // SLSQP's internal QP scales poorly in the number of constraints, so carry only the
  // pairs near enough to matter. The set is screened per attempt at its start rather
  // than once per solve, because the relevant pairs change as q moves.
  const std::vector<std::size_t> & all_pairs = enabledCollisionPairIds();
  if (all_pairs.empty())
  {
    return {};
  }

  std::vector<std::size_t> screened;
  try
  {
    const CollisionEvaluation collision =
      evaluateCollisionConstraints(active_q, all_pairs, false);
    // h = safety_distance - distance, so a pair is within the activation band when
    // h >= safety_distance - collision_activation_distance.
    const double activation_value =
      options_.safety_distance - options_.collision_activation_distance;
    screened.reserve(all_pairs.size());
    for (std::size_t row = 0; row < all_pairs.size(); ++row)
    {
      const double value = collision.values[static_cast<Eigen::Index>(row)];
      if (std::isfinite(value) && value >= activation_value)
      {
        screened.push_back(all_pairs[row]);
      }
    }
  }
  catch (const std::exception &)
  {
    // Screening is an optimization; fall back to the full set if it fails.
    return all_pairs;
  }
  return screened;
}

bool NloptSolver::solveAttempt(
  const Eigen::VectorXd & initial, const Eigen::VectorXd & seed,
  const std::vector<Eigen::Isometry3d> & desired, double remaining_time,
  Eigen::VectorXd & candidate, double & pose_error)
{
  const unsigned int variables = static_cast<unsigned int>(joint_names_.size());
  Context context{this, &desired, &seed, {}, initial,
    std::numeric_limits<double>::infinity(), false};
  context.collision_pair_ids = screenCollisionPairs(initial);

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
  // The objective is a squared error, so a pose tolerance of t corresponds to an
  // objective near t^2. Both stopping criteria sit an order of magnitude below the
  // converged scale; the early exit in the objective is the intended way to stop.
  optimizer.set_xtol_abs(options_.tolerance * 0.1);
  optimizer.set_ftol_abs(options_.tolerance * options_.tolerance * 0.1);
  optimizer.set_maxeval(options_.max_iterations);
  optimizer.set_maxtime(remaining_time);

  std::vector<double> values(initial.data(), initial.data() + initial.size());
  double objective_value = std::numeric_limits<double>::infinity();
  bool optimizer_succeeded = false;
  try
  {
    const nlopt::result status = optimizer.optimize(values, objective_value);
    optimizer_succeeded = status > 0;
#ifdef WBDOPT_IK_DEBUG
    std::cerr << "WbdoptIk NLopt: status=" << static_cast<int>(status) <<
      " evaluations=" << optimizer.get_numevals() <<
      " constraints=" << context.collision_pair_ids.size() <<
      " objective=" << objective_value << std::endl;
#endif
  }
  catch (const nlopt::forced_stop &)
  {
    // The early exit fired. nlopt unwinds without writing back to values, so the
    // latched best_q below is the only record of the converged point.
    optimizer_succeeded = true;
#ifdef WBDOPT_IK_DEBUG
    std::cerr << "WbdoptIk NLopt: early stop at tolerance, evaluations=" <<
      optimizer.get_numevals() <<
      " constraints=" << context.collision_pair_ids.size() << std::endl;
#endif
  }
  catch (const std::exception & exception)
  {
#ifdef WBDOPT_IK_DEBUG
    std::cerr << "WbdoptIk NLopt: " << exception.what() << std::endl;
#endif
    optimizer_succeeded = false;
  }

  if (!context.converged && optimizer_succeeded)
  {
    // A normal return leaves the optimum in values, which the objective may never
    // have been asked to evaluate.
    const Eigen::VectorXd final_q = Eigen::Map<Eigen::VectorXd>(
      values.data(), static_cast<Eigen::Index>(values.size()));
    if (final_q.allFinite())
    {
      try
      {
        const double final_error = cost(final_q, desired).squared_error;
        if (std::isfinite(final_error) && final_error < context.best_pose_error)
        {
          context.best_pose_error = final_error;
          context.best_q = final_q;
        }
      }
      catch (const std::exception &)
      {
        // Keep whatever the objective already latched.
      }
    }
  }

  if (!optimizer_succeeded && !context.converged)
  {
    return false;
  }
  if (!context.best_q.allFinite() ||
    !std::isfinite(context.best_pose_error))
  {
    return false;
  }
  candidate = context.best_q;
  pose_error = context.best_pose_error;
  return true;
}

int NloptSolver::solveInternal(
  const Eigen::VectorXd & seed, const std::vector<Eigen::Isometry3d> & desired,
  const Deadline & deadline, Eigen::VectorXd & solution)
{
  std::random_device random_device;
  std::mt19937 generator(random_device());
  const Eigen::VectorXd nominal = rankingReference(seed);
  Eigen::VectorXd best_approximation = seed;
  double best_approximation_error = std::numeric_limits<double>::infinity();
  std::vector<Candidate> candidates;
  bool first_attempt = true;

  // Retry nonlinear optimization from random configurations until the shared
  // deadline, keeping every converged solution. A redundant arm has a continuum
  // of exact solutions, so returning whichever one an attempt happens to reach
  // makes successive calls jump between branches; ranking them against the
  // nominal pose is what makes the output repeatable.
  while (true)
  {
    const double remaining_time = std::chrono::duration<double>(
      deadline - std::chrono::steady_clock::now()).count();
    // Checked before the attempt, so a slice too short to be useful ends the loop
    // instead of producing a noisy result that could displace a good one.
    if (remaining_time <= kMinimumAttemptTime)
    {
      break;
    }

    const Eigen::VectorXd initial = first_attempt ? seed : randomConfiguration(generator);
    first_attempt = false;

    Eigen::VectorXd candidate;
    // Reported as pose error alone, so a distant restart is not penalized for its
    // distance from the seed the way the regularized objective would penalize it.
    double pose_error = std::numeric_limits<double>::infinity();
    if (!solveAttempt(initial, seed, desired, remaining_time, candidate, pose_error))
    {
      continue;
    }
    if (pose_error < best_approximation_error)
    {
      best_approximation_error = pose_error;
      best_approximation = candidate;
    }
    if (converged(candidate, desired))
    {
      candidates.push_back(
        Candidate{candidate, pose_error, nominalDistanceSquared(candidate, nominal)});
    }
  }

  if (candidates.empty())
  {
    solution = best_approximation;
    min_cost_ = best_approximation_error;
    return FAILED;
  }

  // Prefer the valid solution closest to the ranking reference.
  const auto selected = std::min_element(
    candidates.begin(), candidates.end(), &SolverBase::moreNominal);
  solution = selected->configuration;
  min_cost_ = selected->task_error;
#ifdef WBDOPT_IK_DEBUG
  std::cerr << "WbdoptIk NLopt: candidates=" << candidates.size() <<
    " selected nominal_distance=" << selected->nominal_distance <<
    " cost=" << min_cost_ << std::endl;
#endif
  return SUCCESS;
}

}  // namespace wbdopt_ik
