// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "xlerobot_control/diff_drive_controller.hpp"

namespace xlerobot_control
{
controller_interface::CallbackReturn DiffDriveController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  command_timed_out_ = false;
  return diff_drive_controller::DiffDriveController::on_activate(previous_state);
}

controller_interface::return_type DiffDriveController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Adapted from ros2_controllers 4.42.1's subscriber update. The only change
  // is the timeout warning latch; braking still executes on EVERY stale update.
  auto logger = get_node()->get_logger();
  const auto current = received_velocity_msg_.try_get();
  if (current.has_value()) {
    command_msg_ = current.value();
  }

  const bool timed_out = cmd_vel_timeout_ != rclcpp::Duration::from_seconds(0.0) &&
    time - command_msg_.header.stamp > cmd_vel_timeout_;
  if (timed_out) {
    (void)ordered_exported_reference_interfaces_[0]->set_value(0.0);
    (void)ordered_exported_reference_interfaces_[1]->set_value(0.0);
    if (!command_timed_out_) {
      RCLCPP_WARN(logger, "Velocity command timed out. Braking.");
    }
    command_timed_out_ = true;
  } else if (
    std::isfinite(command_msg_.twist.linear.x) && std::isfinite(command_msg_.twist.angular.z))
  {
    (void)ordered_exported_reference_interfaces_[0]->set_value(command_msg_.twist.linear.x);
    (void)ordered_exported_reference_interfaces_[1]->set_value(command_msg_.twist.angular.z);
    command_timed_out_ = false;
  } else {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger, *get_node()->get_clock(), 1000,
      "Command message contains NaNs. Not updating reference interfaces.");
  }
  previous_update_timestamp_ = time;
  return controller_interface::return_type::OK;
}
}  // namespace xlerobot_control

PLUGINLIB_EXPORT_CLASS(
  xlerobot_control::DiffDriveController, controller_interface::ChainableControllerInterface)
