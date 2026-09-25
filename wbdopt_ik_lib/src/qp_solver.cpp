#include "qp_solver.hpp"

#include <qpOASES.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

namespace wbdopt_ik
{

bool QpSolver::solveQp(
  const CostData & cost_data, const ConstraintData & constraint_data,
  Eigen::VectorXd & delta) const
{
  using qpOASES::real_t;
  using RowMajorMatrix = Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using Vector = Eigen::Matrix<real_t, Eigen::Dynamic, 1>;
  const int variables = static_cast<int>(cost_data.gradient.size());
  const int constraint_count = static_cast<int>(constraint_data.collision_matrix.rows());
  RowMajorMatrix hessian = cost_data.hessian.cast<real_t>();
  RowMajorMatrix collision_matrix = constraint_data.collision_matrix.cast<real_t>();
  Vector gradient = cost_data.gradient.cast<real_t>();
  Vector update_lower = constraint_data.update_lower.cast<real_t>();
  Vector update_upper = constraint_data.update_upper.cast<real_t>();
  Vector collision_lower = constraint_data.collision_lower.cast<real_t>();
  Vector collision_upper = constraint_data.collision_upper.cast<real_t>();

  // std::vector<real_t> hessian(static_cast<std::size_t>(variables * variables));
  // std::vector<real_t> gradient(static_cast<std::size_t>(variables));
  // std::vector<real_t> matrix(static_cast<std::size_t>(constraint_count * variables));
  // std::vector<real_t> update_lower(static_cast<std::size_t>(variables));
  // std::vector<real_t> update_upper(static_cast<std::size_t>(variables));
  // std::vector<real_t> collision_lower(static_cast<std::size_t>(constraint_count));
  // std::vector<real_t> collision_upper(static_cast<std::size_t>(constraint_count));

  // // Convert Eigen's matrices into the row-major buffers expected by qpOASES.
  // for (int row = 0; row < variables; ++row)
  // {
  //   gradient[static_cast<std::size_t>(row)] = cost_data.gradient[row];
  //   update_lower[static_cast<std::size_t>(row)] = constraint_data.update_lower[row];
  //   update_upper[static_cast<std::size_t>(row)] = constraint_data.update_upper[row];
  //   for (int column = 0; column < variables; ++column)
  //   {
  //     hessian[static_cast<std::size_t>(row * variables + column)] =
  //       cost_data.hessian(row, column);
  //   }
  //   // Penalize the magnitude of every joint update.
  //   hessian[static_cast<std::size_t>(row * variables + row)] += options_.regularization;
  // }
  // for (int row = 0; row < constraint_count; ++row)
  // {
  //   collision_lower[static_cast<std::size_t>(row)] = constraint_data.collision_lower[row];
  //   collision_upper[static_cast<std::size_t>(row)] = constraint_data.collision_upper[row];
  //   for (int column = 0; column < variables; ++column)
  //   {
  //     matrix[static_cast<std::size_t>(row * variables + column)] =
  //       constraint_data.collision_matrix(row, column);
  //   }
  // }

  hessian.diagonal().array() += options_.regularization; // Regulation
  const real_t * matrix_data = constraint_count > 0 ? collision_matrix.data() : nullptr;
  const real_t * collision_lower_data = constraint_count > 0 ? collision_lower.data() : nullptr;
  const real_t * collision_upper_data = constraint_count > 0 ? collision_upper.data() : nullptr;

  qpOASES::QProblem problem(variables, constraint_count);
  qpOASES::Options qp_options;
  qp_options.setToMPC();
  qp_options.printLevel = qpOASES::PL_NONE;
  qp_options.enableRegularisation = qpOASES::BT_TRUE;
  problem.setOptions(qp_options);
  int working_set_recalculations = std::max(100, 10 * (variables + constraint_count));
  // const real_t * matrix_data = constraint_count > 0 ? matrix.data() : nullptr;
  // const real_t * collision_lower_data = constraint_count > 0 ? collision_lower.data() : nullptr;
  // const real_t * collision_upper_data = constraint_count > 0 ? collision_upper.data() : nullptr;
  
  const qpOASES::returnValue status = problem.init(
    hessian.data(), gradient.data(), matrix_data, update_lower.data(), update_upper.data(),
    collision_lower_data, collision_upper_data, working_set_recalculations);
  
  // const qpOASES::returnValue status = problem.init(
  //   cost_data.hessian.data(), 
  //   cost_data.gradient.data(), 
  //   constraint_data.collision_matrix.data(), 
  //   constraint_data.update_lower.data(), 
  //   constraint_data.update_upper.data(),
  //   constraint_data.collision_lower.data(), 
  //   constraint_data.collision_upper.data(), 
  //   working_set_recalculations
  // );

  if (status != qpOASES::SUCCESSFUL_RETURN)
  {
    return false;
  }

  std::vector<real_t> qp_solution(static_cast<std::size_t>(variables));
  if (problem.getPrimalSolution(qp_solution.data()) != qpOASES::SUCCESSFUL_RETURN)
  {
    return false;
  }
  delta.resize(variables);
  for (int i = 0; i < variables; ++i)
  {
    delta[i] = qp_solution[static_cast<std::size_t>(i)];
  }
  return delta.allFinite();
}

int QpSolver::solveInternal(
  const Eigen::VectorXd & seed, const std::vector<Eigen::Isometry3d> & desired,
  const Deadline & deadline, Eigen::VectorXd & solution)
{
  std::random_device random_device;
  std::mt19937 generator(random_device());
  Eigen::VectorXd best_approximation = seed;
  double best_approximation_error = std::numeric_limits<double>::infinity();
  const Eigen::VectorXd nominal = rankingReference(seed);
  std::vector<Candidate> candidates;
  bool first_attempt = true;
  std::size_t attempt = 0;

  // Run differential-QP iterations, restarting randomly while time remains.
  do
  {
    const bool using_seed = first_attempt;
    Eigen::VectorXd q = using_seed ? seed : randomConfiguration(generator);
    first_attempt = false;
    ++attempt;
#ifdef WBDOPT_IK_DEBUG
    std::cerr << "WbdoptIk QP: attempt=" << attempt <<
      " initial=" << (using_seed ? "seed" : "random") << std::endl;
#endif


    for (int iteration = 0; iteration < options_.max_iterations; ++iteration)
    {
      // Form the local objective and accept configurations meeting exact IK tolerances.
#ifdef WBDOPT_IK_DEBUG
      const auto cost_start = std::chrono::steady_clock::now();
#endif
      const CostData cost_data = cost(q, desired);
#ifdef WBDOPT_IK_DEBUG
      const auto cost_end = std::chrono::steady_clock::now();
#endif
      if (cost_data.squared_error < best_approximation_error)
      {
        best_approximation_error = cost_data.squared_error;
        best_approximation = q;
      }
      if (converged(q, desired))
      {
        candidates.push_back(
          Candidate{q, cost_data.squared_error, nominalDistanceSquared(q, nominal)});
        break;
      }

      // Form constraints and solve one differential QP update.
#ifdef WBDOPT_IK_DEBUG
      const auto constraints_start = std::chrono::steady_clock::now();
#endif
      const ConstraintData constraint_data = constraints(q);
#ifdef WBDOPT_IK_DEBUG
      const auto constraints_end = std::chrono::steady_clock::now();
      const auto qp_start = std::chrono::steady_clock::now();
#endif
      Eigen::VectorXd delta;
      const bool qp_solved = solveQp(cost_data, constraint_data, delta);
#ifdef WBDOPT_IK_DEBUG
      const auto qp_end = std::chrono::steady_clock::now();
      const double cost_ms =
        std::chrono::duration<double, std::milli>(cost_end - cost_start).count();
      const double constraints_ms =
        std::chrono::duration<double, std::milli>(constraints_end - constraints_start).count();
      const double qp_ms =
        std::chrono::duration<double, std::milli>(qp_end - qp_start).count();
      std::cerr << "WbdoptIk QP timing: attempt=" << attempt <<
        " iteration=" << iteration + 1 <<
        " cost_ms=" << cost_ms <<
        " constraints_ms=" << constraints_ms <<
        " form_qp_ms=" << cost_ms + constraints_ms <<
        " solve_qp_ms=" << qp_ms << std::endl;
        // " success=" << (qp_solved ? "true" : "false") << std::endl;
#endif
      if (!qp_solved)
      {
        // if (std::chrono::steady_clock::now() >= deadline) {
        //   break;
        // } else {
        //   continue; // Try the next iteration with the same configuration.
        // }
        break;
      }
      q += delta;
      // for (Eigen::Index i = 0; i < q.size(); ++i)
      // {
      //   const std::size_t index = static_cast<std::size_t>(i);
      //   q[i] = std::clamp(q[i], lower_[index], upper_[index]);
      // }
      // Check for joint convergence.
      if (delta.norm() < options_.tolerance * 0.1)
      {
        break;
      }
      // Check for time limit exceeded.
      if (std::chrono::steady_clock::now() >= deadline)
      {
#ifdef WBDOPT_IK_DEBUG
        std::cerr << "WbdoptIk: Time limit exceeded after " << iteration + 1 << " iterations." << std::endl;
#endif
        break;
      }
    }
  } while (std::chrono::steady_clock::now() < deadline);

#ifdef WBDOPT_IK_DEBUG
  std::cerr << "WbdoptIk QP: candidates=" << candidates.size() <<
    " attempts=" << attempt << std::endl;
#endif
  if (candidates.empty())
  {
    solution = best_approximation;
    min_cost_ = best_approximation_error;
    return FAILED;
  }

  // Prefer the valid IK solution closest to the configured nominal joint pose.
  std::sort(candidates.begin(), candidates.end(), &SolverBase::moreNominal);
  solution = candidates.front().configuration;
  min_cost_ = candidates.front().task_error;
#ifdef WBDOPT_IK_DEBUG
  std::cerr << "WbdoptIk QP: selected nominal_distance=" <<
    candidates.front().nominal_distance << " cost=" << min_cost_ << std::endl;
#endif
  return SUCCESS;
}

}  // namespace wbdopt_ik
