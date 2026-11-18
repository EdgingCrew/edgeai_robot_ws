// AngleTowardsPath: behavior tree action that outputs a small in-place turn (rad,
// signed) towards the planned path, and the same turn the other way, for Nav2's Spin.
// A robot wedged where it can't turn all the way (e.g. in a narrow gap, or next to a
// post that its rear would hit) can alternate such small turns with short steps
// forward or back until it is free (see navigate_to_pose_edgeai.xml).

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace edgeai_bt_plugins
{

class AngleTowardsPath : public BT::SyncActionNode
{
public:
  AngleTowardsPath(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config),
    tf_(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"))
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned path"),
      BT::InputPort<std::string>("robot_frame", "base_footprint", "Robot base frame"),
      BT::InputPort<double>("max_angle", 0.2, "Largest turn (rad)"),
      BT::InputPort<double>("lookahead", 0.3, "Distance to the path point to face (m)"),
      BT::OutputPort<double>("angle", "Turn towards the path (rad, + = left)"),
      BT::OutputPort<double>("opposite_angle", "The same turn the other way, for when the "
                             "turn towards the path is blocked"),
    };
  }

  // FAILURE when there is no path or the robot already faces it (nothing to turn)
  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    if (!getInput("path", path) || path.poses.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::TransformStamped robot;
    try {
      robot = tf_->lookupTransform(
        path.header.frame_id, getInput<std::string>("robot_frame").value(), tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return BT::NodeStatus::FAILURE;
    }
    const double rx = robot.transform.translation.x;
    const double ry = robot.transform.translation.y;

    // First path point at least lookahead away, else the path's end
    const double lookahead = getInput<double>("lookahead").value();
    auto target = path.poses.back().pose.position;
    for (const auto & p : path.poses) {
      if (std::hypot(p.pose.position.x - rx, p.pose.position.y - ry) >= lookahead) {
        target = p.pose.position;
        break;
      }
    }

    const auto & q = robot.transform.rotation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double error = std::remainder(std::atan2(target.y - ry, target.x - rx) - yaw, 2.0 * M_PI);
    if (std::abs(error) < 0.05) {
      return BT::NodeStatus::FAILURE;
    }
    const double max_angle = getInput<double>("max_angle").value();
    const double angle = std::clamp(error, -max_angle, max_angle);
    setOutput("angle", angle);
    setOutput("opposite_angle", -angle);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace edgeai_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<edgeai_bt_plugins::AngleTowardsPath>("AngleTowardsPath");
}
