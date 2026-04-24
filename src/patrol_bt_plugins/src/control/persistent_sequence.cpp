// Based on navigation2/nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
#include "patrol_bt_plugins/control/persistent_sequence.hpp"

namespace patrol_bt_plugins
{

PersistentSequence::PersistentSequence(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf)
{
  setRegistrationID("PersistentSequence");
}

void PersistentSequence::halt()
{
  // Intentionally do NOT reset current_child_idx on the blackboard so that
  // execution resumes from the correct child after a pause/resume cycle.
  ControlNode::halt();
}

BT::NodeStatus PersistentSequence::tick()
{
  const int children_count = children_nodes_.size();

  int current_child_idx;
  getInput("current_child_idx", current_child_idx);

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx < children_count) {
    TreeNode * current_child_node = children_nodes_[current_child_idx];
    const BT::NodeStatus child_status = current_child_node->executeTick();

    switch (child_status) {
      case BT::NodeStatus::RUNNING:
        return child_status;

      case BT::NodeStatus::FAILURE:
        resetChildren();
        current_child_idx = 0;
        setOutput("current_child_idx", 0);
        return child_status;

      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::SKIPPED:
        current_child_idx++;
        setOutput("current_child_idx", current_child_idx);
        break;

      case BT::NodeStatus::IDLE:
        throw BT::LogicError("A child node must never return IDLE");
    }
  }

  // All children returned SUCCESS — reset index for next invocation.
  if (current_child_idx == children_count) {
    resetChildren();
    setOutput("current_child_idx", 0);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace patrol_bt_plugins

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<patrol_bt_plugins::PersistentSequence>(name, config);
    };

  factory.registerBuilder<patrol_bt_plugins::PersistentSequence>(
    "PersistentSequence", builder);
}
