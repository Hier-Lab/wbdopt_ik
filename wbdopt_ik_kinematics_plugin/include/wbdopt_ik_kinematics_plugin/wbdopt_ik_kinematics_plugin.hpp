#ifndef WBDOPT_IK_KINEMATICS_PLUGIN__WBDOPT_IK_KINEMATICS_PLUGIN_HPP_
#define WBDOPT_IK_KINEMATICS_PLUGIN__WBDOPT_IK_KINEMATICS_PLUGIN_HPP_

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <wbdopt_ik/wbdopt_ik.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace wbdopt_ik_kinematics_plugin
{

class WbdoptIkKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  WbdoptIkKinematicsPlugin() = default;
  ~WbdoptIkKinematicsPlugin() override = default;

  bool initialize(
    const rclcpp::Node::SharedPtr & node, const moveit::core::RobotModel & robot_model,
    const std::string & group_name, const std::string & base_frame,
    const std::vector<std::string> & tip_frames, double search_discretization) override;

  bool supportsGroup(
    const moveit::core::JointModelGroup * joint_model_group,
    std::string * error_text = nullptr) const override;

  const std::vector<std::string> & getJointNames() const override {return joint_names_;}
  const std::vector<std::string> & getLinkNames() const override {return link_names_;}

  bool getPositionFK(
    const std::vector<std::string> & link_names, const std::vector<double> & joint_angles,
    std::vector<geometry_msgs::msg::Pose> & poses) const override;

  bool getPositionIK(
    const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
    std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options = {}) const override;

  bool getPositionIK(
    const std::vector<geometry_msgs::msg::Pose> & ik_poses,
    const std::vector<double> & ik_seed_state, std::vector<std::vector<double>> & solutions,
    kinematics::KinematicsResult & result,
    const kinematics::KinematicsQueryOptions & options) const override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
    double timeout, std::vector<double> & solution,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options = {}) const override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
    double timeout, const std::vector<double> & consistency_limits,
    std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options = {}) const override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
    double timeout, std::vector<double> & solution, const IKCallbackFn & solution_callback,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options = {}) const override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
    double timeout, const std::vector<double> & consistency_limits,
    std::vector<double> & solution, const IKCallbackFn & solution_callback,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options = {}) const override;

  bool searchPositionIK(
    const std::vector<geometry_msgs::msg::Pose> & ik_poses,
    const std::vector<double> & ik_seed_state, double timeout,
    const std::vector<double> & consistency_limits, std::vector<double> & solution,
    const IKCallbackFn & solution_callback, moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options = {},
    const moveit::core::RobotState * context_state = nullptr) const override;

private:
  bool solve(
    const std::vector<geometry_msgs::msg::Pose> & ik_poses,
    const std::vector<double> & ik_seed_state, double timeout,
    const std::vector<double> & consistency_limits, std::vector<double> & solution,
    const IKCallbackFn & solution_callback, moveit_msgs::msg::MoveItErrorCodes & error_code,
    const moveit::core::RobotState * context_state) const;

  bool active_{false};
  const moveit::core::JointModelGroup * joint_model_group_{nullptr};
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  wbdopt_ik::SolverOptions solver_options_;
  std::unique_ptr<wbdopt_ik::WbdoptIk> solver_;
  mutable std::mutex solver_mutex_;
};

}  // namespace wbdopt_ik_kinematics_plugin

#endif  // WBDOPT_IK_KINEMATICS_PLUGIN__WBDOPT_IK_KINEMATICS_PLUGIN_HPP_
