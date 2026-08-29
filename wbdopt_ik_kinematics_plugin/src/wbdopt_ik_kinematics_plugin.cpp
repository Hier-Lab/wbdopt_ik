#include <wbdopt_ik_kinematics_plugin/wbdopt_ik_kinematics_plugin.hpp>

#include <ament_index_cpp/get_resources.hpp>
#include <class_loader/class_loader.hpp>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_list_macros.hpp>
#include <srdfdom/model.h>
#include <tinyxml.h>
#include <urdf_parser/urdf_parser.h>
#include <wbdopt_ik_kinematics_plugin/wbdopt_ik_kinematics_plugin_parameters.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace wbdopt_ik_kinematics_plugin
{
namespace
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("wbdopt_ik_kinematics_plugin");

Eigen::Isometry3d toEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);
  if (!quaternion.coeffs().allFinite() || !translation.allFinite() ||
    quaternion.norm() < 1e-12)
  {
    throw std::invalid_argument("Desired pose contains non-finite values or a zero quaternion");
  }
  quaternion.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = translation;
  return transform;
}

geometry_msgs::msg::Pose toMessage(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.rotation());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

std::string exportUrdf(const moveit::core::RobotModel & robot_model)
{
  std::unique_ptr<TiXmlDocument> document(urdf::exportURDF(*robot_model.getURDF()));
  if (!document)
  {
    throw std::runtime_error("Could not serialize MoveIt's URDF model");
  }
  TiXmlPrinter printer;
  document->Accept(&printer);
  return printer.CStr();
}

std::vector<std::string> amentPackagePaths()
{
  const auto resources = ament_index_cpp::get_resources("packages");
  std::set<std::string> unique_paths;
  for (const auto & resource : resources)
  {
    unique_paths.insert(resource.second + "/share");
  }
  return {unique_paths.begin(), unique_paths.end()};
}

std::vector<double> orderedNominalPositions(
  const std::vector<std::string> & active_joint_names,
  const std::vector<std::string> & nominal_joint_names,
  const std::vector<double> & nominal_joint_positions)
{
  if (nominal_joint_names.empty() && nominal_joint_positions.empty())
  {
    return {};
  }
  if (nominal_joint_names.size() != nominal_joint_positions.size())
  {
    throw std::runtime_error(
            "nominal_joint_names and nominal_joint_positions must have equal lengths");
  }

  std::unordered_map<std::string, double> positions_by_name;
  for (std::size_t i = 0; i < nominal_joint_names.size(); ++i)
  {
    if (!std::isfinite(nominal_joint_positions[i]) ||
      !positions_by_name.emplace(nominal_joint_names[i], nominal_joint_positions[i]).second)
    {
      throw std::runtime_error("Nominal joint entries must be finite and uniquely named");
    }
  }

  std::vector<double> ordered;
  ordered.reserve(active_joint_names.size());
  for (const std::string & joint_name : active_joint_names)
  {
    const auto position = positions_by_name.find(joint_name);
    if (position == positions_by_name.end())
    {
      throw std::runtime_error("Missing nominal position for active joint: " + joint_name);
    }
    ordered.push_back(position->second);
  }
  return ordered;
}

}  // namespace

bool WbdoptIkKinematicsPlugin::initialize(
  const rclcpp::Node::SharedPtr & node, const moveit::core::RobotModel & robot_model,
  const std::string & group_name, const std::string & base_frame,
  const std::vector<std::string> & tip_frames, double search_discretization)
{
  active_ = false;
  joint_model_group_ = robot_model.getJointModelGroup(group_name);
  if (!joint_model_group_)
  {
    RCLCPP_ERROR(LOGGER, "Unknown MoveIt joint group '%s'", group_name.c_str());
    return false;
  }
  if (!supportsGroup(joint_model_group_))
  {
    return false;
  }

  joint_names_ = joint_model_group_->getActiveJointModelNames();
  link_names_ = joint_model_group_->getLinkModelNames();
  try
  {
    const std::string parameter_prefix = "robot_description_kinematics." + group_name;
    wbdopt_ik_kinematics::ParamListener listener(node, parameter_prefix);
    const auto parameters = listener.get_params();

    std::vector<std::string> effective_tip_frames = parameters.tip_frames;
    if (effective_tip_frames.empty())
    {
      // MoveIt Humble's loader selects only the last group link. Prefer the
      // semantic end-effector tips when the JointModelGroup can provide them.
      joint_model_group_->getEndEffectorTips(effective_tip_frames);
    }
    if (effective_tip_frames.empty())
    {
      effective_tip_frames = tip_frames;
    }
    if (effective_tip_frames.empty())
    {
      throw std::runtime_error("At least one IK tip frame is required");
    }
    for (const std::string & tip : effective_tip_frames)
    {
      if (!robot_model.hasLinkModel(tip))
      {
        throw std::runtime_error("Unknown IK tip frame: " + tip);
      }
    }
    storeValues(robot_model, group_name, base_frame, effective_tip_frames, search_discretization);

    solver_options_.solver = wbdopt_ik::solverTypeFromString(parameters.solver_type);
    solver_options_.tolerance = parameters.epsilon;
    solver_options_.regularization = parameters.regularization;
    solver_options_.position_weight = parameters.position_weight;
    solver_options_.orientation_weight = parameters.orientation_weight;
    solver_options_.joint_centering_name = parameters.joint_centering_name;
    solver_options_.joint_centering_weight = parameters.joint_centering_weight;
    solver_options_.safety_distance = parameters.safety_distance;
    solver_options_.collision_activation_distance = parameters.collision_activation_distance;
    solver_options_.max_joint_step = parameters.max_joint_step;
    solver_options_.max_iterations = parameters.max_iterations;
    solver_options_.nominal_joint_positions = orderedNominalPositions(
      joint_names_, parameters.nominal_joint_names, parameters.nominal_joint_positions);
    solver_options_.max_time = getDefaultTimeout();

    std::vector<wbdopt_ik::WbdoptIk::DisabledCollisionPair> disabled_pairs;
    int dis_pairs = 0;
    if (robot_model.getSRDF())
    {
      
      for (const auto & pair : robot_model.getSRDF()->getDisabledCollisionPairs())
      {
        disabled_pairs.emplace_back(pair.link1_, pair.link2_);
        dis_pairs++;
      }
    }
    std::cout << "Disabled collision pairs: " << dis_pairs << std::endl;
    solver_ = std::make_unique<wbdopt_ik::WbdoptIk>(
      exportUrdf(robot_model), effective_tip_frames, joint_names_, disabled_pairs, solver_options_,
      amentPackagePaths());

    std::vector<double> lower;
    std::vector<double> upper;
    lower.reserve(joint_names_.size());
    upper.reserve(joint_names_.size());
    for (const std::string & joint_name : joint_names_)
    {
      const moveit::core::JointModel * joint = robot_model.getJointModel(joint_name);
      const auto & bounds = joint->getVariableBounds().front();
      if (bounds.position_bounded_)
      {
        lower.push_back(bounds.min_position_);
        upper.push_back(bounds.max_position_);
      }
      else
      {
        lower.push_back(-M_PI);
        upper.push_back(M_PI);
      }
    }
    if (!solver_->setBounds(lower, upper))
    {
      throw std::runtime_error("Failed to transfer MoveIt joint bounds to WbdoptIk");
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(LOGGER, "Could not initialize WbdoptIk: %s", exception.what());
    solver_.reset();
    return false;
  }

  active_ = true;
  RCLCPP_INFO(
    LOGGER, "Initialized WbdoptIk for group '%s' with %zu joints and %zu tips",
    group_name.c_str(), joint_names_.size(), tip_frames_.size());
  for (std::size_t i = 0; i < tip_frames_.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "  IK tip[%zu]: %s", i, tip_frames_[i].c_str());
  }
  return true;
}

bool WbdoptIkKinematicsPlugin::supportsGroup(
  const moveit::core::JointModelGroup * group, std::string * error_text) const
{
  if (!group)
  {
    if (error_text) {*error_text = "The joint model group is null";}
    return false;
  }
  if (group->getVariableCount() != group->getActiveJointModels().size())
  {
    if (error_text)
    {
      *error_text = "WbdoptIk does not support multi-variable or mimic joints in the IK group";
    }
    return false;
  }
  for (const moveit::core::JointModel * joint : group->getActiveJointModels())
  {
    if (joint->getVariableCount() != 1U)
    {
      if (error_text)
      {
        *error_text = "WbdoptIk currently supports only one-DoF active joints; joint '" +
          joint->getName() + "' has multiple variables";
      }
      return false;
    }
  }
  return !group->getActiveJointModels().empty();
}

bool WbdoptIkKinematicsPlugin::getPositionFK(
  const std::vector<std::string> & requested_links, const std::vector<double> & joint_angles,
  std::vector<geometry_msgs::msg::Pose> & poses) const
{
  if (!active_ || joint_angles.size() != joint_names_.size())
  {
    return false;
  }
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(joint_model_group_, joint_angles);
  state.update();
  const moveit::core::LinkModel * base_link = robot_model_->getLinkModel(base_frame_);
  if (!base_link)
  {
    return false;
  }
  const Eigen::Isometry3d world_to_base = state.getGlobalLinkTransform(base_link).inverse();
  poses.clear();
  poses.reserve(requested_links.size());
  for (const std::string & link_name : requested_links)
  {
    const moveit::core::LinkModel * link = robot_model_->getLinkModel(link_name);
    if (!link)
    {
      return false;
    }
    poses.push_back(toMessage(world_to_base * state.getGlobalLinkTransform(link)));
  }
  return true;
}

bool WbdoptIkKinematicsPlugin::getPositionIK(
  const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
  std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
  const kinematics::KinematicsQueryOptions & options) const
{
  if (tip_frames_.size() != 1U)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }
  return searchPositionIK(
    ik_pose, ik_seed_state, default_timeout_, solution, error_code, options);
}

bool WbdoptIkKinematicsPlugin::getPositionIK(
  const std::vector<geometry_msgs::msg::Pose> & ik_poses,
  const std::vector<double> & ik_seed_state, std::vector<std::vector<double>> & solutions,
  kinematics::KinematicsResult & result,
  const kinematics::KinematicsQueryOptions & options) const
{
  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  const bool success = solve(
    ik_poses, ik_seed_state, default_timeout_, {}, solution, {}, error_code, nullptr);
  solutions.clear();
  if (success)
  {
    solutions.push_back(std::move(solution));
    result.kinematic_error = kinematics::KinematicErrors::OK;
    result.solution_percentage = 1.0;
  }
  else
  {
    result.kinematic_error = kinematics::KinematicErrors::NO_SOLUTION;
    result.solution_percentage = 0.0;
  }
  (void)options;
  return success;
}

bool WbdoptIkKinematicsPlugin::searchPositionIK(
  const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
  double timeout, std::vector<double> & solution,
  moveit_msgs::msg::MoveItErrorCodes & error_code,
  const kinematics::KinematicsQueryOptions & options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, {}, solution, {}, error_code, options);
}

bool WbdoptIkKinematicsPlugin::searchPositionIK(
  const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
  double timeout, const std::vector<double> & consistency_limits,
  std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
  const kinematics::KinematicsQueryOptions & options) const
{
  return searchPositionIK(
    ik_pose, ik_seed_state, timeout, consistency_limits, solution, {}, error_code, options);
}

bool WbdoptIkKinematicsPlugin::searchPositionIK(
  const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
  double timeout, std::vector<double> & solution, const IKCallbackFn & solution_callback,
  moveit_msgs::msg::MoveItErrorCodes & error_code,
  const kinematics::KinematicsQueryOptions & options) const
{
  return searchPositionIK(
    ik_pose, ik_seed_state, timeout, {}, solution, solution_callback, error_code, options);
}

bool WbdoptIkKinematicsPlugin::searchPositionIK(
  const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
  double timeout, const std::vector<double> & consistency_limits,
  std::vector<double> & solution, const IKCallbackFn & solution_callback,
  moveit_msgs::msg::MoveItErrorCodes & error_code,
  const kinematics::KinematicsQueryOptions & options) const
{
  if (tip_frames_.size() != 1U)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }
  (void)options;
  return solve(
    {ik_pose}, ik_seed_state, timeout, consistency_limits, solution, solution_callback,
    error_code, nullptr);
}

bool WbdoptIkKinematicsPlugin::searchPositionIK(
  const std::vector<geometry_msgs::msg::Pose> & ik_poses,
  const std::vector<double> & ik_seed_state, double timeout,
  const std::vector<double> & consistency_limits, std::vector<double> & solution,
  const IKCallbackFn & solution_callback, moveit_msgs::msg::MoveItErrorCodes & error_code,
  const kinematics::KinematicsQueryOptions & options,
  const moveit::core::RobotState * context_state) const
{
  (void)options;
  return solve(
    ik_poses, ik_seed_state, timeout, consistency_limits, solution, solution_callback,
    error_code, context_state);
}

bool WbdoptIkKinematicsPlugin::solve(
  const std::vector<geometry_msgs::msg::Pose> & ik_poses,
  const std::vector<double> & ik_seed_state, double timeout,
  const std::vector<double> & consistency_limits, std::vector<double> & solution,
  const IKCallbackFn & solution_callback, moveit_msgs::msg::MoveItErrorCodes & error_code,
  const moveit::core::RobotState * context_state) const
{
  if (!active_ || !solver_)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }
  if (ik_poses.size() != tip_frames_.size() || ik_seed_state.size() != joint_names_.size() ||
    timeout <= 0.0 ||
    (!consistency_limits.empty() && consistency_limits.size() != joint_names_.size()))
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  moveit::core::RobotState state = context_state ? *context_state : moveit::core::RobotState(robot_model_);
  if (!context_state)
  {
    state.setToDefaultValues();
  }
  state.setJointGroupPositions(joint_model_group_, ik_seed_state);
  state.update();
  const moveit::core::LinkModel * base_link = robot_model_->getLinkModel(base_frame_);
  if (!base_link)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
    return false;
  }
  const Eigen::Isometry3d world_from_base = state.getGlobalLinkTransform(base_link);

  std::vector<Eigen::Isometry3d> desired;
  desired.reserve(ik_poses.size());
  try
  {
    for (const auto & pose : ik_poses)
    {
      desired.push_back(world_from_base * toEigen(pose));
    }
  }
  catch (const std::exception &)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  Eigen::Map<const Eigen::VectorXd> seed(ik_seed_state.data(), ik_seed_state.size());
  Eigen::VectorXd q_solution;
  int status;
  {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    auto request_options = solver_options_;
    request_options.max_time = timeout;
    solver_->setOptions(request_options);
    status = solver_->CartToJoint(seed, desired, q_solution);
  }
  if (status != wbdopt_ik::SUCCESS)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }
  solution.assign(q_solution.data(), q_solution.data() + q_solution.size());

  for (std::size_t i = 0; i < consistency_limits.size(); ++i)
  {
    if (consistency_limits[i] < 0.0 ||
      std::abs(solution[i] - ik_seed_state[i]) > consistency_limits[i])
    {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }
  }
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  if (solution_callback)
  {
    solution_callback(ik_poses.front(), solution, error_code);
  }
  return error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

}  // namespace wbdopt_ik_kinematics_plugin

PLUGINLIB_EXPORT_CLASS(
  wbdopt_ik_kinematics_plugin::WbdoptIkKinematicsPlugin, kinematics::KinematicsBase)
