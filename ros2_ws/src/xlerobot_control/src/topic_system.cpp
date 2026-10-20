#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace xlerobot_control
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using SteadyClock = std::chrono::steady_clock;

class TopicSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    const auto & info = get_hardware_info();
    try {
      timeout_ = std::stod(info.hardware_parameters.at("feedback_timeout"));
      startup_timeout_ = std::stod(info.hardware_parameters.at("startup_timeout"));
      if (!std::isfinite(timeout_) || timeout_ <= 0 ||
        !std::isfinite(startup_timeout_) || startup_timeout_ <= 0) {
        return CallbackReturn::ERROR;
      }
      for (const auto & joint : info.joints) {
        if (joint.command_interfaces.size() != 1) {return CallbackReturn::ERROR;}
        const auto & command = joint.command_interfaces.front();
        if (command.name != "position" && command.name != "velocity") {
          return CallbackReturn::ERROR;
        }
        names_.push_back(joint.name);
        modes_.push_back(command.name);
        lower_.push_back(std::stod(command.min));
        upper_.push_back(std::stod(command.max));
        if (!std::isfinite(lower_.back()) || !std::isfinite(upper_.back()) ||
          lower_.back() >= upper_.back()) {return CallbackReturn::ERROR;}
      }
      namespace_ = info.hardware_parameters.at("namespace");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Invalid hardware configuration: %s", error.what());
      return CallbackReturn::ERROR;
    }
    positions_.assign(names_.size(), std::numeric_limits<double>::quiet_NaN());
    velocities_ = positions_;
    commands_ = positions_;
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
    for (size_t i = 0; i < names_.size(); ++i) {
      interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(names_[i], "position", &positions_[i]));
      interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(names_[i], "velocity", &velocities_[i]));
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
    for (size_t i = 0; i < names_.size(); ++i) {
      interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(names_[i], modes_[i], &commands_[i]));
    }
    return interfaces;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    // Own executor: callbacks run synchronously in read()/on_activate(), so no
    // middleware thread can modify interface storage during a controller update.
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    node_ = std::make_shared<rclcpp::Node>("joint_transport", "/" + namespace_, options);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    position_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("backend/position_command", 1);
    velocity_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("backend/velocity_command", 1);
    subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "backend/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->name.size() != msg->position.size() || msg->name.size() != msg->velocity.size()) {return;}
        std::map<std::string, size_t> index;
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (!index.emplace(msg->name[i], i).second) {return;}
        }
        for (const auto & name : names_) {
          const auto found = index.find(name);
          if (found == index.end() || !std::isfinite(msg->position[found->second]) ||
            !std::isfinite(msg->velocity[found->second])) {return;}
        }
        for (size_t i = 0; i < names_.size(); ++i) {
          positions_[i] = msg->position[index.at(names_[i])];
          velocities_[i] = msg->velocity[index.at(names_[i])];
        }
        last_feedback_ = SteadyClock::now();
        received_ = true;
      });
    active_ = received_ = false;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    const auto start = SteadyClock::now();
    while (rclcpp::ok() && elapsed(start) < startup_timeout_) {
      executor_->spin_some();
      if (fresh()) {
        for (size_t i = 0; i < names_.size(); ++i) {
          commands_[i] = modes_[i] == "position" ? positions_[i] : 0.0;
        }
        active_ = true;
        RCLCPP_INFO(node_->get_logger(), "Transport activated from measured joint positions");
        return CallbackReturn::SUCCESS;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_ERROR(node_->get_logger(), "No complete backend feedback before startup timeout");
    return CallbackReturn::ERROR;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    executor_->spin_some();
    if (active_ && !fresh()) {
      halt();
      RCLCPP_ERROR(node_->get_logger(), "Backend feedback expired; stopping controllers");
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    if (!active_) {return return_type::OK;}
    if (!fresh()) {halt(); return return_type::ERROR;}
    sensor_msgs::msg::JointState position, velocity;
    position.header.stamp = velocity.header.stamp = time;
    for (size_t i = 0; i < names_.size(); ++i) {
      if (!std::isfinite(commands_[i])) {halt(); return return_type::ERROR;}
      const double value = std::clamp(commands_[i], lower_[i], upper_[i]);
      if (modes_[i] == "position") {
        position.name.push_back(names_[i]); position.position.push_back(value);
      } else {
        velocity.name.push_back(names_[i]); velocity.velocity.push_back(value);
      }
    }
    position_pub_->publish(position);
    velocity_pub_->publish(velocity);
    return return_type::OK;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {halt(); return CallbackReturn::SUCCESS;}
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override {halt(); return CallbackReturn::SUCCESS;}
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {halt(); return CallbackReturn::SUCCESS;}
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    halt(); subscription_.reset(); position_pub_.reset(); velocity_pub_.reset();
    executor_.reset(); node_.reset(); received_ = false;
    return CallbackReturn::SUCCESS;
  }

private:
  static double elapsed(SteadyClock::time_point since) {return std::chrono::duration<double>(SteadyClock::now() - since).count();}
  bool fresh() const {return received_ && elapsed(last_feedback_) <= timeout_;}
  void halt()
  {
    if (!active_ || !node_) {return;}
    sensor_msgs::msg::JointState position, velocity;
    position.header.stamp = velocity.header.stamp = node_->now();
    for (size_t i = 0; i < names_.size(); ++i) {
      if (modes_[i] == "velocity") {velocity.name.push_back(names_[i]); velocity.velocity.push_back(0.0);}
      else if (std::isfinite(positions_[i])) {position.name.push_back(names_[i]); position.position.push_back(positions_[i]);}
    }
    velocity_pub_->publish(velocity); position_pub_->publish(position);
    active_ = false;
  }
  std::string namespace_;
  std::vector<std::string> names_, modes_;
  std::vector<double> positions_, velocities_, commands_, lower_, upper_;
  double timeout_{1.0}, startup_timeout_{120.0};
  bool received_{false}, active_{false};
  SteadyClock::time_point last_feedback_{};
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr position_pub_, velocity_pub_;
};
}  // namespace xlerobot_control
PLUGINLIB_EXPORT_CLASS(xlerobot_control::TopicSystem, hardware_interface::SystemInterface)
