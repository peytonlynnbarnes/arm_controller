#include "ros2_control_demo_example_7/r6bot_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace ros2_control_demo_example_7
{

RobotController::RobotController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn RobotController::on_init()
{
  joint_names_ =
    auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  point_interp_.positions.assign(joint_names_.size(), 0.0);
  point_interp_.velocities.assign(joint_names_.size(), 0.0);

  kp_ = auto_declare<double>("kp", 50.0);
  kd_ = auto_declare<double>("kd", 5.0);

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  for (const auto & j : joint_names_) {
    for (const auto & iface : command_interface_types_) {
      conf.names.push_back(j + "/" + iface);   // ONLY "effort"
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  for (const auto & j : joint_names_) {
    for (const auto & iface : state_interface_types_) {
      conf.names.push_back(j + "/" + iface);  // position, velocity, effort
    }
  }
  return conf;
}

controller_interface::CallbackReturn RobotController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto callback = [this](const trajectory_msgs::msg::JointTrajectory msg)
  {
    traj_msg_external_.set(msg);
    new_msg_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Received trajectory");
  };

  joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(
  const rclcpp_lifecycle::State &)
{
  joint_effort_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  joint_effort_state_interface_.clear();

  // Command interfaces (ONLY effort)
  for (auto & interface : command_interfaces_) {
    if (interface.get_interface_name() == "effort") {
      joint_effort_command_interface_.push_back(interface);
    }
  }

  // State interfaces (ALL)
  for (auto & interface : state_interfaces_) {
    const auto & name = interface.get_interface_name();
    if (name == "position") {
      joint_position_state_interface_.push_back(interface);
    } else if (name == "velocity") {
      joint_velocity_state_interface_.push_back(interface);
    } else if (name == "effort") {
      joint_effort_state_interface_.push_back(interface);
    }
  }

  return CallbackReturn::SUCCESS;
}

void interpolate_point(
  const trajectory_msgs::msg::JointTrajectoryPoint & p1,
  const trajectory_msgs::msg::JointTrajectoryPoint & p2,
  trajectory_msgs::msg::JointTrajectoryPoint & out,
  double delta)
{
  for (size_t i = 0; i < p1.positions.size(); i++) {
    out.positions[i] = delta * p2.positions[i] + (1.0 - delta) * p1.positions[i];
    out.velocities[i] = delta * p2.velocities[i] + (1.0 - delta) * p1.velocities[i];
  }
}

void interpolate_trajectory_point(
  const trajectory_msgs::msg::JointTrajectory & traj,
  const rclcpp::Duration & cur_time,
  trajectory_msgs::msg::JointTrajectoryPoint & point_interp,
  bool & reached_end)
{
  double total_time =
    traj.points.back().time_from_start.sec +
    traj.points.back().time_from_start.nanosec * 1e-9;

  double t = cur_time.seconds();
  reached_end = (t >= total_time);

  if (reached_end) {
    point_interp.positions = traj.points.back().positions;
    std::fill(point_interp.velocities.begin(), point_interp.velocities.end(), 0.0);
    return;
  }

  // explicit casts fix warnings
  const double num_points = static_cast<double>(traj.points.size());
  const double segment_count = num_points - 1.0;

  size_t idx = static_cast<size_t>((t / total_time) * segment_count);
  idx = std::min(idx, traj.points.size() - 2);

  double segment_time = total_time / num_points;
  double delta = (t - static_cast<double>(idx) * segment_time) / segment_time;

  delta = std::clamp(delta, 0.0, 1.0);

  interpolate_point(traj.points[idx], traj.points[idx + 1], point_interp, delta);
}


controller_interface::return_type RobotController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (new_msg_) {
    auto op = traj_msg_external_.try_get();
    if (op.has_value()) {
      trajectory_msg_ = op.value();
      start_time_ = time;
      new_msg_ = false;
    }
  }

  if (trajectory_msg_.points.empty()) {
    return controller_interface::return_type::OK;
  }

  bool done = false;
  interpolate_trajectory_point(
    trajectory_msg_, time - start_time_, point_interp_, done);

  if (done) {
    trajectory_msg_.points.clear();
    RCLCPP_INFO(get_node()->get_logger(), "Trajectory complete.");
  }

  // Effort-only PD control
  for (size_t i = 0; i < joint_effort_command_interface_.size(); i++)
  {
      auto q_opt  = joint_position_state_interface_[i].get().get_optional();
      auto dq_opt = joint_velocity_state_interface_[i].get().get_optional();

      double q  = q_opt.value_or(0.0);
      double dq = dq_opt.value_or(0.0);

      double qd  = point_interp_.positions[i];
      double dqd = point_interp_.velocities[i];

      double e  = qd - q;
      double de = dqd - dq;

      double tau = kp_ * e + kd_ * de;

      bool ok = joint_effort_command_interface_[i].get().set_value(tau);
      if (!ok) {
          RCLCPP_ERROR(
              get_node()->get_logger(),
              "Failed to set effort command value for joint %zu", i);
      }
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

}  // namespace ros2_control_demo_example_7

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_7::RobotController,
  controller_interface::ControllerInterface)
