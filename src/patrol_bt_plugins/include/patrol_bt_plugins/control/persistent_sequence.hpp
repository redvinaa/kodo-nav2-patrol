// Based on navigation2/nav2_behavior_tree

#ifndef PATROL_BT_PLUGINS__CONTROL__PERSISTENT_SEQUENCE_HPP_
#define PATROL_BT_PLUGINS__CONTROL__PERSISTENT_SEQUENCE_HPP_

#include <string>

#include "behaviortree_cpp_v3/control_node.h"

namespace patrol_bt_plugins
{

/**
 * @brief A Sequence node that persists its current child index in the blackboard.
 *
 * Unlike a standard SequenceWithMemory, when this node is halted (e.g. due to a
 * pause/resume cycle) it does NOT reset the blackboard key, so execution resumes
 * from the last running child after the node is ticked again.
 *
 * Blackboard port:
 *   current_child_idx — bidirectional; stores the index of the child to execute next.
 */
class PersistentSequence : public BT::ControlNode
{
public:
  PersistentSequence(const std::string & name, const BT::NodeConfiguration & conf);

  ~PersistentSequence() override = default;

  void halt() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<int>("current_child_idx", "The index of the current child"),
    };
  }

private:
  BT::NodeStatus tick() override;
};

}  // namespace patrol_bt_plugins

#endif  // PATROL_BT_PLUGINS__CONTROL__PERSISTENT_SEQUENCE_HPP_
