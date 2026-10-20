#pragma once

#include "diff_drive_controller/diff_drive_controller.hpp"

namespace xlerobot_control
{
// Keep upstream wheel control, odometry, limits, subscriptions and lifecycle;
// only report entry into a command timeout, rather than every second of it.
class DiffDriveController : public diff_drive_controller::DiffDriveController
{
public:
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool command_timed_out_{false};
};
}  // namespace xlerobot_control
