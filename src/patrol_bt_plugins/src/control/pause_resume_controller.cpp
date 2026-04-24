// Based on navigation2/nav2_behavior_tree

#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "patrol_bt_plugins/control/pause_resume_controller.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace patrol_bt_plugins
{

using namespace std::placeholders;

PauseResumeController::PauseResumeController(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode(xml_tag_name, conf)
{
  node_ = this->config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  state_ = UNPAUSED;

  cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(cb_group_, node_->get_node_base_interface());

  std::string pause_service_name;
  getInput("pause_service_name", pause_service_name);
  pause_srv_ = node_->create_service<Trigger>(
    pause_service_name,
    std::bind(&PauseResumeController::pause_service_callback, this, _1, _2),
    rmw_qos_profile_services_default, cb_group_);

  std::string resume_service_name;
  getInput("resume_service_name", resume_service_name);
  resume_srv_ = node_->create_service<Trigger>(
    resume_service_name,
    std::bind(&PauseResumeController::resume_service_callback, this, _1, _2),
    rmw_qos_profile_services_default, cb_group_);

  spinner_thread_ = std::make_unique<std::thread>(
    [&]() {
      executor_->spin();
    });
  spinner_thread_->detach();
}

PauseResumeController::~PauseResumeController()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down PauseResumeController BT node");
  executor_->cancel();
}

BT::NodeStatus PauseResumeController::tick()
{
  unsigned int children_count = children_nodes_.size();
  if (children_count < 1 || children_count > 4) {
    throw BT::LogicError(
            "PauseResumeController must have at least one and at most four children "
            "(currently has " + std::to_string(children_count) + ")");
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (status() == BT::NodeStatus::IDLE) {
    state_ = UNPAUSED;
  }
  if (state_ == PAUSE_REQUESTED) {
    resetChildren();
    state_ = ON_PAUSE;
    RCLCPP_INFO(node_->get_logger(), "PauseResumeController: switched to state ON_PAUSE");
  }
  if (state_ == RESUME_REQUESTED) {
    resetChildren();
    state_ = ON_RESUME;
    RCLCPP_INFO(node_->get_logger(), "PauseResumeController: switched to state ON_RESUME");
  }
  setStatus(BT::NodeStatus::RUNNING);

  if (state_ == ON_PAUSE) {
    if (children_count < 3) {
      RCLCPP_INFO(node_->get_logger(), "PauseResumeController: switched to state PAUSED");
      state_ = PAUSED;
    } else {
      const BT::NodeStatus child_status = children_nodes_[2]->executeTick();
      switch (child_status) {
        case BT::NodeStatus::RUNNING:
          break;
        case BT::NodeStatus::SUCCESS:
        case BT::NodeStatus::SKIPPED:
          RCLCPP_INFO(node_->get_logger(), "PauseResumeController: switched to state PAUSED");
          state_ = PAUSED;
          break;
        case BT::NodeStatus::FAILURE:
          RCLCPP_ERROR(node_->get_logger(), "PauseResumeController: ON_PAUSE child returned FAILURE");
          setStatus(BT::NodeStatus::FAILURE);
          break;
        default:
          throw BT::LogicError("A child node must never return IDLE");
      }
    }
  }

  if (state_ == ON_RESUME) {
    if (children_count < 4) {
      RCLCPP_INFO(node_->get_logger(), "PauseResumeController: switched to state UNPAUSED");
      state_ = UNPAUSED;
    } else {
      const BT::NodeStatus child_status = children_nodes_[3]->executeTick();
      switch (child_status) {
        case BT::NodeStatus::RUNNING:
          break;
        case BT::NodeStatus::SUCCESS:
        case BT::NodeStatus::SKIPPED:
          RCLCPP_INFO(node_->get_logger(), "PauseResumeController: switched to state UNPAUSED");
          state_ = UNPAUSED;
          break;
        case BT::NodeStatus::FAILURE:
          RCLCPP_ERROR(node_->get_logger(), "PauseResumeController: ON_RESUME child returned FAILURE");
          setStatus(BT::NodeStatus::FAILURE);
          break;
        default:
          throw BT::LogicError("A child node must never return IDLE");
      }
    }
  }

  if (state_ == PAUSED) {
    if (children_count >= 2) {
      const BT::NodeStatus child_status = children_nodes_[1]->executeTick();
      switch (child_status) {
        case BT::NodeStatus::RUNNING:
        case BT::NodeStatus::SUCCESS:
        case BT::NodeStatus::SKIPPED:
          break;
        case BT::NodeStatus::FAILURE:
          RCLCPP_ERROR(node_->get_logger(), "PauseResumeController: PAUSED child returned FAILURE");
          setStatus(BT::NodeStatus::FAILURE);
          break;
        default:
          throw BT::LogicError("A child node must never return IDLE");
      }
    }
  }

  if (state_ == UNPAUSED) {
    const BT::NodeStatus child_status = children_nodes_[0]->executeTick();
    switch (child_status) {
      case BT::NodeStatus::RUNNING:
        break;
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::SKIPPED:
        setStatus(BT::NodeStatus::SUCCESS);
        break;
      case BT::NodeStatus::FAILURE:
        RCLCPP_ERROR(node_->get_logger(), "PauseResumeController: UNPAUSED child returned FAILURE");
        setStatus(BT::NodeStatus::FAILURE);
        break;
      default:
        throw BT::LogicError("A child node must never return IDLE");
    }
  }

  return status();
}

void PauseResumeController::pause_service_callback(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  if (status() == BT::NodeStatus::IDLE) {
    std::string msg = "PauseResumeController has not been ticked yet";
    response->success = false;
    response->message = msg;
    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != PAUSED) {
    RCLCPP_INFO(node_->get_logger(), "PauseResumeController: PAUSE_REQUESTED");
    response->success = true;
    state_ = PAUSE_REQUESTED;
    return;
  }

  std::string msg = "PauseResumeController is already PAUSED";
  RCLCPP_WARN(node_->get_logger(), "%s", msg.c_str());
  response->success = false;
  response->message = msg;
}

void PauseResumeController::resume_service_callback(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  if (status() == BT::NodeStatus::IDLE) {
    std::string msg = "PauseResumeController has not been ticked yet";
    response->success = false;
    response->message = msg;
    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ == PAUSED) {
    RCLCPP_INFO(node_->get_logger(), "PauseResumeController: RESUME_REQUESTED");
    response->success = true;
    state_ = RESUME_REQUESTED;
    return;
  }

  std::string msg = "PauseResumeController is not currently PAUSED";
  RCLCPP_WARN(node_->get_logger(), "%s", msg.c_str());
  response->success = false;
  response->message = msg;
}

void PauseResumeController::halt()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = UNPAUSED;
  ControlNode::halt();
}

}  // namespace patrol_bt_plugins

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<patrol_bt_plugins::PauseResumeController>(name, config);
    };

  factory.registerBuilder<patrol_bt_plugins::PauseResumeController>(
    "PauseResumeController", builder);
}
