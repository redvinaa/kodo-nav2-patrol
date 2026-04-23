#ifndef PATROL_BT_PLUGINS__CONTROL__PAUSE_RESUME_CONTROLLER_HPP_
#define PATROL_BT_PLUGINS__CONTROL__PAUSE_RESUME_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp/control_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace patrol_bt_plugins
{

using Trigger = std_srvs::srv::Trigger;

enum state_t {UNPAUSED, PAUSED, PAUSE_REQUESTED, ON_PAUSE, RESUME_REQUESTED, ON_RESUME};

/**
 * @brief Control node that exposes ROS 2 services to pause and resume BT execution.
 *
 * Children (all optional except the first):
 *   [0] UNPAUSED branch  — ticked while running normally
 *   [1] PAUSED branch    — ticked while paused
 *   [2] ON_PAUSE branch  — ticked once when transitioning into paused
 *   [3] ON_RESUME branch — ticked once when transitioning out of paused
 *
 * Input ports:
 *   pause_service_name  — service name to trigger a pause
 *   resume_service_name — service name to trigger a resume
 */
class PauseResumeController : public BT::ControlNode
{
public:
  PauseResumeController(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~PauseResumeController();

  void halt() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("pause_service_name", "Name of the service to pause"),
      BT::InputPort<std::string>("resume_service_name", "Name of the service to resume"),
    };
  }

private:
  void pause_service_callback(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  void resume_service_callback(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<std::thread> spinner_thread_;
  rclcpp::Service<Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<Trigger>::SharedPtr resume_srv_;
  state_t state_;
  std::mutex state_mutex_;
};

}  // namespace patrol_bt_plugins

#endif  // PATROL_BT_PLUGINS__CONTROL__PAUSE_RESUME_CONTROLLER_HPP_
