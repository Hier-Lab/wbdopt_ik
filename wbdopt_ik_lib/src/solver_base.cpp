#include "solver_base.hpp"

#include <coal/shape/geometric_shapes.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#ifdef WBDOPT_IK_DEBUG
#include <iostream>
#endif

namespace wbdopt_ik
{
namespace
{

Eigen::Matrix3d skew(const Eigen::Vector3d & value)
{
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(), -value.y(), value.x(), 0.0;
  return result;
}

std::string canonicalPair(const std::string & first, const std::string & second)
{
  return first < second ? first + "\n" + second : second + "\n" + first;
}

std::size_t replaceCylindersWithCapsules(pinocchio::GeometryModel & geometry_model)
{
  std::size_t replacement_count = 0;
  for (auto & geometry_object : geometry_model.geometryObjects)
  {
    const auto cylinder = std::dynamic_pointer_cast<coal::Cylinder>(geometry_object.geometry);
    if (!cylinder)
    {
      continue;
    }

    // Preserve the cylinder's radius and total axial extent after rounding its ends.
    const double capsule_length = std::max(
      0.0, 2.0 * (cylinder->halfLength - cylinder->radius));
    auto capsule = std::make_shared<coal::Capsule>(cylinder->radius, capsule_length);
    capsule->setSweptSphereRadius(cylinder->getSweptSphereRadius());
    capsule->computeLocalAABB();
    geometry_object.geometry = std::move(capsule);
    ++replacement_count;
  }
  return replacement_count;
}

bool isValidDesiredPose(const Eigen::Isometry3d & pose)
{
  if (!pose.matrix().allFinite())
  {
    return false;
  }
  const Eigen::Matrix3d rotation = pose.linear();
  return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
    std::abs(rotation.determinant() - 1.0) < 1e-6;
}

}  // namespace

SolverBase::SolverBase(
  const std::string & urdf, const std::string & srdf,
  const std::vector<std::string> & end_effector_frames,
  const std::vector<std::string> & active_joint_names, const SolverOptions & options,
  ModelInput input, const std::vector<std::string> & package_paths)
: options_(options)
{
  // Load both kinematic and collision models from files or serialized XML.
  if (input == ModelInput::FILES)
  {
    pinocchio::urdf::buildModel(urdf, model_);
    pinocchio::urdf::buildGeom(
      model_, urdf, pinocchio::COLLISION, geometry_model_, package_paths);
    geometry_model_.addAllCollisionPairs();
    if (!srdf.empty())
    {
      pinocchio::srdf::removeCollisionPairs(model_, geometry_model_, srdf, false);
    }
  }
  else
  {
    pinocchio::urdf::buildModelFromXML(urdf, model_);
    std::istringstream urdf_stream(urdf);
    pinocchio::urdf::buildGeom(
      model_, urdf_stream, pinocchio::COLLISION, geometry_model_, package_paths);
    geometry_model_.addAllCollisionPairs();
    if (!srdf.empty())
    {
      pinocchio::srdf::removeCollisionPairsFromXML(model_, geometry_model_, srdf, false);
    }
  }
  finishInitialization(end_effector_frames, active_joint_names);
}

SolverBase::SolverBase(
  const std::string & urdf_xml, const std::vector<std::string> & end_effector_frames,
  const std::vector<std::string> & active_joint_names,
  const std::vector<DisabledCollisionPair> & disabled_collision_pairs,
  const SolverOptions & options, const std::vector<std::string> & package_paths)
: options_(options)
{
  // MoveIt supplies URDF XML plus its already-resolved disabled collision pairs.
  pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
  std::istringstream urdf_stream(urdf_xml);
  pinocchio::urdf::buildGeom(
    model_, urdf_stream, pinocchio::COLLISION, geometry_model_, package_paths);
  geometry_model_.addAllCollisionPairs();

  std::unordered_set<std::string> disabled;
  for (const auto & pair : disabled_collision_pairs)
  {
    disabled.insert(canonicalPair(pair.first, pair.second));
  }
  std::vector<pinocchio::CollisionPair> pairs_to_remove;
  for (const auto & pair : geometry_model_.collisionPairs)
  {
    const auto & object_a = geometry_model_.geometryObjects[pair.first];
    const auto & object_b = geometry_model_.geometryObjects[pair.second];
    const std::string & link_a = model_.frames[object_a.parentFrame].name;
    const std::string & link_b = model_.frames[object_b.parentFrame].name;
    if (disabled.count(canonicalPair(link_a, link_b)) != 0U)
    {
      pairs_to_remove.push_back(pair);
    }
  }
  for (const auto & pair : pairs_to_remove)
  {
    geometry_model_.removeCollisionPair(pair);
  }
  finishInitialization(end_effector_frames, active_joint_names);
}

void SolverBase::finishInitialization(
  const std::vector<std::string> & end_effector_frames,
  const std::vector<std::string> & requested_joint_names)
{
  // Replace sharp-ended cylinders before GeometryData caches their shape information.
  [[maybe_unused]] const std::size_t replaced_cylinders =
    replaceCylindersWithCapsules(geometry_model_);

  // Allocate reusable Pinocchio data and enable signed collision distances.
  data_ = std::make_unique<pinocchio::Data>(model_);
  geometry_data_ = std::make_unique<pinocchio::GeometryData>(geometry_model_);
  for (auto & request : geometry_data_->distanceRequests)
  {
    request.enable_signed_distance = true;
  }
  enabled_collision_pair_ids_.reserve(geometry_model_.collisionPairs.size());
  for (std::size_t pair_id = 0; pair_id < geometry_model_.collisionPairs.size(); ++pair_id)
  {
    if (geometry_data_->activeCollisionPairs[pair_id])
    {
      enabled_collision_pair_ids_.push_back(pair_id);
    }
  }

  // Resolve the active one-DoF joints and their Pinocchio indices.
  std::vector<std::string> active_names = requested_joint_names;
  if (active_names.empty())
  {
    for (pinocchio::JointIndex joint_id = 1; joint_id < model_.joints.size(); ++joint_id)
    {
      if (model_.joints[joint_id].nv() == 1 &&
        (model_.joints[joint_id].nq() == 1 || model_.joints[joint_id].nq() == 2))
      {
        active_names.push_back(model_.names[joint_id]);
      }
    }
  }
  for (const std::string & name : active_names)
  {
    const pinocchio::JointIndex joint_id = model_.getJointId(name);
    if (joint_id == model_.joints.size())
    {
      throw std::invalid_argument("Unknown active joint: " + name);
    }
    const auto & joint = model_.joints[joint_id];
    if (joint.nv() != 1 || (joint.nq() != 1 && joint.nq() != 2))
    {
      throw std::invalid_argument("Only one-DoF active joints are supported: " + name);
    }
    joint_names_.push_back(name);
    q_indices_.push_back(joint.idx_q());
    q_dimensions_.push_back(joint.nq());
    v_indices_.push_back(joint.idx_v());
    lower_.push_back(joint.nq() == 1 ? model_.lowerPositionLimit[joint.idx_q()] : -M_PI);
    upper_.push_back(joint.nq() == 1 ? model_.upperPositionLimit[joint.idx_q()] : M_PI);
  }
  if (joint_names_.empty())
  {
    throw std::invalid_argument("The robot model has no supported active joints");
  }

  q_reference_ = pinocchio::neutral(model_);
  if (!setEndEffectors(end_effector_frames))
  {
    throw std::invalid_argument("One or more end-effector frame names are invalid");
  }
  validateOptions(options_);

#ifdef WBDOPT_IK_DEBUG
  std::cerr << "WbdoptIk: model DoF=" << model_.nq <<
    " active_joints=" << joint_names_.size() <<
    " q_indices=" << q_indices_.size() <<
    " v_indices_" << v_indices_.size() <<
    " collision_pairs=" << geometry_model_.collisionPairs.size() <<
    " cylinders_replaced=" << replaced_cylinders << std::endl;
#endif
}

bool SolverBase::setBounds(
  const std::vector<double> & lower, const std::vector<double> & upper)
{
  if (lower.size() != joint_names_.size() || upper.size() != lower.size())
  {
    return false;
  }
  for (std::size_t i = 0; i < lower.size(); ++i)
  {
    if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] > upper[i])
    {
      return false;
    }
  }
  lower_ = lower;
  upper_ = upper;
  return true;
}

bool SolverBase::setEndEffectors(const std::vector<std::string> & frame_names)
{
  std::vector<pinocchio::FrameIndex> ids;
  ids.reserve(frame_names.size());
  for (const std::string & name : frame_names)
  {
    const pinocchio::FrameIndex frame_id = model_.getFrameId(name);
    if (frame_id == model_.frames.size())
    {
      return false;
    }
    ids.push_back(frame_id);
  }
  end_effector_names_ = frame_names;
  end_effector_ids_ = std::move(ids);
  return true;
}

void SolverBase::validateOptions(const SolverOptions & options) const
{
  if (options.max_time <= 0.0 || options.tolerance <= 0.0 ||
    options.regularization < 0.0 || options.position_weight < 0.0 ||
    options.orientation_weight < 0.0 || options.joint_centering_weight < 0.0 ||
    options.safety_distance < 0.0 ||
    options.collision_activation_distance < options.safety_distance ||
    options.max_joint_step <= 0.0 || options.max_iterations <= 0)
  {
    throw std::invalid_argument("Invalid WbdoptIk solver option");
  }
  if (!options.nominal_joint_positions.empty())
  {
    if (options.nominal_joint_positions.size() != joint_names_.size() ||
      !std::all_of(
        options.nominal_joint_positions.begin(), options.nominal_joint_positions.end(),
        [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "nominal_joint_positions must contain one finite value per active joint");
    }
  }
  if (options.joint_centering_weight > 0.0)
  {
    if (options.joint_centering_name.empty())
    {
      throw std::invalid_argument(
              "joint_centering_name is required when joint_centering_weight is positive");
    }
    if (std::find(
        joint_names_.begin(), joint_names_.end(), options.joint_centering_name) ==
      joint_names_.end())
    {
      throw std::invalid_argument(
              "Unknown joint_centering_name: " + options.joint_centering_name);
    }
  }
}

void SolverBase::setOptions(const SolverOptions & options)
{
  if (options.solver != type())
  {
    throw std::invalid_argument(
            "Cannot change solver backend with setOptions; construct a new WbdoptIk instance");
  }
  validateOptions(options);
  options_ = options;
}

Eigen::VectorXd SolverBase::toFullConfiguration(const Eigen::VectorXd & active_q) const
{
  Eigen::VectorXd full_q = q_reference_;
  for (Eigen::Index i = 0; i < active_q.size(); ++i)
  {
    const std::size_t index = static_cast<std::size_t>(i);
    if (q_dimensions_[index] == 1)
    {
      full_q[q_indices_[index]] = active_q[i];
    }
    else
    {
      full_q[q_indices_[index]] = std::cos(active_q[i]);
      full_q[q_indices_[index] + 1] = std::sin(active_q[i]);
    }
  }
  return full_q;
}

void SolverBase::updateModel(const Eigen::VectorXd & active_q)
{
  if (active_q.size() != static_cast<Eigen::Index>(joint_names_.size()) ||
    !active_q.allFinite())
  {
    throw std::invalid_argument("Configuration must contain one finite value per active joint");
  }
  const bool configuration_changed = !model_cache_valid_ ||
    cached_active_q_.size() != active_q.size() ||
    !(cached_active_q_.array() == active_q.array()).all();
  if (configuration_changed)
  {
    // FK and frame placement are the common prerequisite for every solver operation.
    const Eigen::VectorXd full_q = toFullConfiguration(active_q);
    pinocchio::forwardKinematics(model_, *data_, full_q);
    pinocchio::updateFramePlacements(model_, *data_);
    cached_active_q_ = active_q;
    model_cache_valid_ = true;
    jacobian_cache_valid_ = false;
    distance_cache_valid_ = false;
  }

  // Compute each expensive derivative or geometry stage at most once for this q.
  if (!jacobian_cache_valid_)
  {
    pinocchio::computeJointJacobians(model_, *data_);
    jacobian_cache_valid_ = true;
  }
}

Eigen::MatrixXd SolverBase::activeColumns(const Eigen::MatrixXd & full_jacobian) const
{
  Eigen::MatrixXd result(full_jacobian.rows(), static_cast<Eigen::Index>(v_indices_.size()));
  for (std::size_t i = 0; i < v_indices_.size(); ++i)
  {
    result.col(static_cast<Eigen::Index>(i)) = full_jacobian.col(v_indices_[i]);
  }
  return result;
}

CostData SolverBase::cost(
  const Eigen::VectorXd & active_q, const std::vector<Eigen::Isometry3d> & desired)
{
  CostData result;
  const Eigen::Index n = static_cast<Eigen::Index>(joint_names_.size());
  // result.hessian = options_.regularization * options_.regularization *
  //   Eigen::MatrixXd::Identity(n, n);
  result.hessian = Eigen::MatrixXd::Zero(n, n);
  result.gradient = Eigen::VectorXd::Zero(n);

  // Reuse FK/Jacobians if another objective or constraint already evaluated this q.
  updateModel(active_q);
  for (std::size_t i = 0; i < end_effector_ids_.size(); ++i)
  {
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, model_.nv);
    jacobian.setZero();
    pinocchio::getFrameJacobian(
      model_, *data_, end_effector_ids_[i], pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    const Eigen::MatrixXd active_jacobian = activeColumns(jacobian);
    const pinocchio::SE3 & current = data_->oMf[end_effector_ids_[i]];
    const Eigen::Vector3d position_error = desired[i].translation() - current.translation();
    const Eigen::Vector3d orientation_error =
      current.rotation() * pinocchio::log3(current.rotation().transpose() * desired[i].linear()); // Rotation error in world frame

    const auto linear = active_jacobian.topRows(3);
    const auto angular = active_jacobian.bottomRows(3);
    result.hessian.noalias() += options_.position_weight * linear.transpose() * linear;
    result.hessian.noalias() +=
      options_.orientation_weight * angular.transpose() * angular;
    result.gradient.noalias() -=
      options_.position_weight * linear.transpose() * position_error;
    result.gradient.noalias() -=
      options_.orientation_weight * angular.transpose() * orientation_error;
    result.squared_error += options_.position_weight * position_error.squaredNorm() +
      options_.orientation_weight * orientation_error.squaredNorm();
  }

  // Optionally bias one joint toward the midpoint of its configured bounds.
  if (options_.joint_centering_weight > 0.0)
  {
    const auto iterator = std::find(
      joint_names_.begin(), joint_names_.end(), options_.joint_centering_name);
    const std::size_t index = static_cast<std::size_t>(
      std::distance(joint_names_.begin(), iterator));
    const double middle = 0.5 * (lower_[index] + upper_[index]);
    const double half_range = 0.5 * (upper_[index] - lower_[index]);
    const double denominator = std::max(half_range * half_range, 1e-12);
    const double error = active_q[static_cast<Eigen::Index>(index)] - middle;
    const double normalized_weight = options_.joint_centering_weight / denominator;
    result.hessian(static_cast<Eigen::Index>(index), static_cast<Eigen::Index>(index)) +=
      normalized_weight;
    result.gradient[static_cast<Eigen::Index>(index)] += normalized_weight * error;
    result.squared_error += normalized_weight * error * error;
  }
  return result;
}

Eigen::MatrixXd SolverBase::pointJacobian(
  pinocchio::JointIndex joint_id, const Eigen::Vector3d & world_point) const
{
  Eigen::Matrix<double, 6, Eigen::Dynamic> joint_jacobian(6, model_.nv);
  joint_jacobian.setZero();
  pinocchio::getJointJacobian(
    model_, *data_, joint_id, pinocchio::LOCAL_WORLD_ALIGNED, joint_jacobian);
  const Eigen::Vector3d offset = world_point - data_->oMi[joint_id].translation();
  const Eigen::MatrixXd translational = joint_jacobian.topRows(3) -
    skew(offset) * joint_jacobian.bottomRows(3);
  return activeColumns(translational);
}

const std::vector<std::size_t> & SolverBase::enabledCollisionPairIds() const noexcept
{
  return enabled_collision_pair_ids_;
}

SolverBase::CollisionEvaluation SolverBase::evaluateCollisionConstraints(
  const Eigen::VectorXd & active_q, const std::vector<std::size_t> & pair_ids,
  bool compute_gradients)
{
  // Both QP and NLopt use h(q) = safety_distance - distance(q) <= 0.
  updateModel(active_q);
  if (!distance_cache_valid_)
  {
    pinocchio::updateGeometryPlacements(model_, *data_, geometry_model_, *geometry_data_);
    pinocchio::computeDistances(geometry_model_, *geometry_data_);
    distance_cache_valid_ = true;
  }

  const Eigen::Index rows = static_cast<Eigen::Index>(pair_ids.size());
  CollisionEvaluation evaluation;
  evaluation.values = Eigen::VectorXd::Constant(rows, -kInfinity);
  evaluation.gradients = Eigen::MatrixXd::Zero(rows, active_q.size());

  for (std::size_t row = 0; row < pair_ids.size(); ++row)
  {
    const std::size_t pair_id = pair_ids[row];
    const auto & distance = geometry_data_->distanceResults[pair_id];
    if (!std::isfinite(distance.min_distance))
    {
      continue;
    }
    const Eigen::Index output_row = static_cast<Eigen::Index>(row);
    evaluation.values[output_row] = options_.safety_distance - distance.min_distance;
    if (!compute_gradients)
    {
      continue;
    }

    // Coal's normal and nearest points define the derivative of signed distance.
    Eigen::Vector3d normal = distance.normal;
    if (!normal.allFinite() || normal.norm() < 1e-12)
    {
      normal = distance.nearest_points[1] - distance.nearest_points[0];
    }
    if (!normal.allFinite() || normal.norm() < 1e-12)
    {
      continue;
    }
    normal.normalize();
    if (!distance.nearest_points[0].allFinite() || !distance.nearest_points[1].allFinite())
    {
      continue;
    }

    const pinocchio::CollisionPair & pair = geometry_model_.collisionPairs[pair_id];
    const auto & geometry_a = geometry_model_.geometryObjects[pair.first];
    const auto & geometry_b = geometry_model_.geometryObjects[pair.second];
    const Eigen::MatrixXd jacobian_a =
      pointJacobian(geometry_a.parentJoint, distance.nearest_points[0]);
    const Eigen::MatrixXd jacobian_b =
      pointJacobian(geometry_b.parentJoint, distance.nearest_points[1]);
    const Eigen::RowVectorXd distance_gradient =
      normal.transpose() * (jacobian_b - jacobian_a);

    // h(q) = safety_distance - distance(q), so dh/dq = -ddistance/dq.
    evaluation.gradients.row(output_row) = -distance_gradient;
  }
  return evaluation;
}

ConstraintData SolverBase::constraints(const Eigen::VectorXd & active_q)
{
  ConstraintData result;
  const Eigen::Index n = active_q.size();

  // Keep delta-q bounds separate from the linearized collision constraint matrix.
  result.update_lower = Eigen::VectorXd::Zero(n);
  result.update_upper = Eigen::VectorXd::Zero(n);
  for (Eigen::Index i = 0; i < n; ++i)
  {
    result.update_lower[i] = std::max(
      lower_[static_cast<std::size_t>(i)] - active_q[i], -options_.max_joint_step);
    result.update_upper[i] = std::min(
      upper_[static_cast<std::size_t>(i)] - active_q[i], options_.max_joint_step);
  }

  // Collision constraints
  const auto & pair_ids = enabledCollisionPairIds();
  const CollisionEvaluation collision =
    evaluateCollisionConstraints(active_q, pair_ids, true);
  std::vector<Eigen::RowVectorXd> collision_jacobian_rows;
  std::vector<double> collision_lower;
  
  // QP form of collision avoidance: distance_gradient * delta >= safety_distance - distance.
  for (std::size_t row = 0; row < pair_ids.size(); ++row)
  {
    const double value = collision.values[static_cast<Eigen::Index>(row)];
    const double activation_value =
      options_.safety_distance - options_.collision_activation_distance;
    if (!std::isfinite(value) || value < activation_value)
    {
      continue;
    }

    // Lower bound of collision constraints
    collision_lower.push_back(value);

    // QP uses distance_gradient * delta >= safety_distance - distance.
    collision_jacobian_rows.push_back(
      -collision.gradients.row(static_cast<Eigen::Index>(row)));
  }

  const Eigen::Index rows = static_cast<Eigen::Index>(collision_jacobian_rows.size());
  result.collision_matrix = Eigen::MatrixXd::Zero(rows, n);
  result.collision_lower = Eigen::VectorXd::Zero(rows);
  result.collision_upper = Eigen::VectorXd::Constant(rows, kInfinity);
  for (std::size_t i = 0; i < collision_jacobian_rows.size(); ++i)
  {
    const Eigen::Index row = static_cast<Eigen::Index>(i);
    result.collision_matrix.row(row) = collision_jacobian_rows[i];
    result.collision_lower[row] = collision_lower[i];
  }
  return result;
}

bool SolverBase::collisionSafe(const Eigen::VectorXd & active_q)
{
  const auto & pair_ids = enabledCollisionPairIds();
  if (pair_ids.empty())
  {
    return true;
  }
  const CollisionEvaluation collision =
    evaluateCollisionConstraints(active_q, pair_ids, false);
  for (Eigen::Index i = 0; i < collision.values.size(); ++i)
  {
    if (collision.values[i] > options_.tolerance)
    {
      return false;
    }
  }
  return true;
}

bool SolverBase::converged(
  const Eigen::VectorXd & active_q, const std::vector<Eigen::Isometry3d> & desired)
{
  updateModel(active_q);
  double square_error = 0.0;
  for (std::size_t i = 0; i < desired.size(); ++i)
  {
    const pinocchio::SE3 & current = data_->oMf[end_effector_ids_[i]];
    // if (options_.position_weight > 0.0 &&
    //   (desired[i].translation() - current.translation()).norm() > options_.tolerance)
    // {
    //   return false;
    // }
    // if (options_.orientation_weight > 0.0 &&
    //   pinocchio::log3(current.rotation().transpose() * desired[i].linear()).norm() >
    //   options_.tolerance)
    // {
    //   return false;
    // }
    const Eigen::Vector3d position_error = desired[i].translation() - current.translation();
    const Eigen::Vector3d orientation_error = 
      current.rotation() * pinocchio::log3(current.rotation().transpose() * desired[i].linear());

    if (options_.position_weight > 0.0)
    {
      square_error += position_error.squaredNorm();
    }
    if (options_.orientation_weight > 0.0)
    {
      square_error += orientation_error.squaredNorm();
    }
  }
  if (std::sqrt(square_error) > options_.tolerance)
  {
    return false;
  }
  return collisionSafe(active_q);
}

Eigen::VectorXd SolverBase::rankingReference(const Eigen::VectorXd & seed) const
{
  if (options_.nominal_joint_positions.empty())
  {
    // Without a configured rest posture, prefer the solution closest to the
    // request seed, which keeps successive calls temporally coherent.
    return seed;
  }
  return Eigen::Map<const Eigen::VectorXd>(
    options_.nominal_joint_positions.data(),
    static_cast<Eigen::Index>(options_.nominal_joint_positions.size()));
}

double SolverBase::nominalDistanceSquared(
  const Eigen::VectorXd & configuration, const Eigen::VectorXd & nominal) const
{
  double distance = 0.0;
  for (Eigen::Index i = 0; i < configuration.size(); ++i)
  {
    double difference = configuration[i] - nominal[i];
    if (q_dimensions_[static_cast<std::size_t>(i)] == 2)
    {
      // Continuous joints compare by shortest angular distance, so a solution is
      // not penalized for winding.
      difference = std::remainder(difference, 2.0 * M_PI);
    }
    distance += difference * difference;
  }
  return distance;
}

bool SolverBase::moreNominal(const Candidate & left, const Candidate & right)
{
  if (left.nominal_distance == right.nominal_distance)
  {
    return left.task_error < right.task_error;
  }
  return left.nominal_distance < right.nominal_distance;
}

Eigen::VectorXd SolverBase::randomConfiguration(std::mt19937 & generator) const
{
  Eigen::VectorXd result(static_cast<Eigen::Index>(joint_names_.size()));
  for (Eigen::Index i = 0; i < result.size(); ++i)
  {
    double lower = lower_[static_cast<std::size_t>(i)];
    double upper = upper_[static_cast<std::size_t>(i)];
    if (!std::isfinite(lower) || lower < -1e10)
    {
      lower = -M_PI;
    }
    if (!std::isfinite(upper) || upper > 1e10)
    {
      upper = M_PI;
    }
    std::uniform_real_distribution<double> distribution(lower, upper);
    result[i] = distribution(generator);
  }
  return result;
}

CostData SolverBase::formCost(
  const Eigen::VectorXd & q, const std::vector<Eigen::Isometry3d> & desired)
{
  if (q.size() != static_cast<Eigen::Index>(joint_names_.size()) ||
    desired.size() != end_effector_ids_.size() || !q.allFinite() ||
    !std::all_of(desired.begin(), desired.end(), isValidDesiredPose))
  {
    throw std::invalid_argument("Invalid configuration or desired pose");
  }
  return cost(q, desired);
}

ConstraintData SolverBase::formConstraints(const Eigen::VectorXd & q)
{
  if (q.size() != static_cast<Eigen::Index>(joint_names_.size()))
  {
    throw std::invalid_argument("Invalid configuration size");
  }
  return constraints(q);
}

int SolverBase::solve(
  const Eigen::VectorXd & seed, const std::vector<Eigen::Isometry3d> & desired,
  Eigen::VectorXd & solution)
{
  if (seed.size() != static_cast<Eigen::Index>(joint_names_.size()) || desired.empty() ||
    desired.size() != end_effector_ids_.size() || !seed.allFinite() ||
    !std::all_of(desired.begin(), desired.end(), isValidDesiredPose))
  {
    return INVALID_INPUT;
  }

  // Accept tiny state-monitor overshoots, then clamp the backend seed to hard bounds.
  constexpr double limit_tolerance = 1e-6;
  Eigen::VectorXd bounded_seed = seed;
  for (Eigen::Index i = 0; i < seed.size(); ++i)
  {
    const std::size_t index = static_cast<std::size_t>(i);
    if (seed[i] < lower_[index] - limit_tolerance || seed[i] > upper_[index] + limit_tolerance)
    {
      return INVALID_INPUT;
    }
    bounded_seed[i] = std::clamp(seed[i], lower_[index], upper_[index]);
  }

  const Deadline deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(options_.max_time));
  return solveInternal(bounded_seed, desired, deadline, solution);
}

int SolverBase::solve(
  const Eigen::VectorXd & seed, const Eigen::VectorXd & desired_vector,
  Eigen::VectorXd & solution)
{
  const std::size_t tips = end_effector_ids_.size();
  if (tips == 0U ||
    (desired_vector.size() != static_cast<Eigen::Index>(3U * tips) &&
    desired_vector.size() != static_cast<Eigen::Index>(7U * tips)) ||
    !desired_vector.allFinite())
  {
    return INVALID_INPUT;
  }

  // Convert the compact position-only or pose representation into target transforms.
  std::vector<Eigen::Isometry3d> desired(tips, Eigen::Isometry3d::Identity());
  if (desired_vector.size() == static_cast<Eigen::Index>(3U * tips))
  {
    if (seed.size() != static_cast<Eigen::Index>(joint_names_.size()))
    {
      return INVALID_INPUT;
    }
    const Eigen::VectorXd full_q = toFullConfiguration(seed);
    pinocchio::forwardKinematics(model_, *data_, full_q);
    pinocchio::updateFramePlacements(model_, *data_);
    for (std::size_t i = 0; i < tips; ++i)
    {
      desired[i].linear() = data_->oMf[end_effector_ids_[i]].rotation();
      desired[i].translation() =
        desired_vector.segment<3>(static_cast<Eigen::Index>(3U * i));
    }
  }
  else
  {
    for (std::size_t i = 0; i < tips; ++i)
    {
      const Eigen::Index offset = static_cast<Eigen::Index>(7U * i);
      desired[i].translation() = desired_vector.segment<3>(offset);
      Eigen::Quaterniond quaternion(
        desired_vector[offset + 6], desired_vector[offset + 3],
        desired_vector[offset + 4], desired_vector[offset + 5]);
      if (quaternion.norm() < 1e-12)
      {
        return INVALID_INPUT;
      }
      quaternion.normalize();
      desired[i].linear() = quaternion.toRotationMatrix();
    }
  }
  return solve(seed, desired, solution);
}

}  // namespace wbdopt_ik
