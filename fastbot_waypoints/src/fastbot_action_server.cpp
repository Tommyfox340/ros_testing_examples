#include <chrono>
#include <cmath>
#include <memory>

#include <fastbot_waypoints/action/waypoint.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

class FastbotActionServer : public rclcpp::Node {
public:
  using Waypoint = fastbot_waypoints::action::Waypoint;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

  explicit FastbotActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("waypoint_action_server", options) {
    using namespace std::placeholders;

    // Declare parameters
    this->declare_parameter("max_linear_velocity", 0.5);
    this->declare_parameter("max_angular_velocity", 0.65);
    this->declare_parameter("kp_linear", 0.5);
    this->declare_parameter("kp_angular", 2.0);
    this->declare_parameter("yaw_precision", 0.05);
    this->declare_parameter("dist_precision", 0.05);

    // Get parameters
    max_linear_velocity_ =
        this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ =
        this->get_parameter("max_angular_velocity").as_double();
    kp_linear_ = this->get_parameter("kp_linear").as_double();
    kp_angular_ = this->get_parameter("kp_angular").as_double();
    yaw_precision_ = this->get_parameter("yaw_precision").as_double();
    dist_precision_ = this->get_parameter("dist_precision").as_double();

    // Action server to accept goals
    action_server_ = rclcpp_action::create_server<Waypoint>(
        this, "fastbot_as",
        std::bind(&FastbotActionServer::handle_goal, this, _1, _2),
        std::bind(&FastbotActionServer::handle_cancel, this, _1),
        std::bind(&FastbotActionServer::handle_accepted, this, _1));

    // Subscribe to odometry
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&FastbotActionServer::odom_callback, this, _1));

    // Publisher to command velocity
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Fastbot action server started");
  }

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // Odom data
  geometry_msgs::msg::Point current_position_;
  double current_yaw_ = 0.0;

  // Parameters
  double max_linear_velocity_;
  double max_angular_velocity_;
  double kp_linear_;
  double kp_angular_;
  double yaw_precision_;
  double dist_precision_;

  // Odom Callback
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update position
    current_position_ = msg->pose.pose.position;

    // Calculate the theta (yaw - orientation around the z-axis)
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    current_yaw_ = tf2::getYaw(q);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Waypoint::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with position: (%.2f, %.2f)",
                goal->position.x, goal->position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    using namespace std::placeholders;
    // This needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&FastbotActionServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result = std::make_shared<Waypoint::Result>();

    // Get desired position from goal
    const auto &desired_pos = goal->position;

    // Main control loop
    rclcpp::Rate loop_rate(25.0); // 25 Hz
    bool reached_goal = false;
    std::string state = "idle";

    while (rclcpp::ok() && !reached_goal) {
      // Check for cancellation
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(this->get_logger(), "Goal canceled by client");
        stop_robot();
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      // Compute distance and angle errors
      const double dx = desired_pos.x - current_position_.x;
      const double dy = desired_pos.y - current_position_.y;
      double dist_error = std::hypot(dx, dy);
      double desired_yaw = std::atan2(dy, dx);
      double yaw_error = normalize_angle(desired_yaw - current_yaw_);

      RCLCPP_INFO(this->get_logger(), "Distance error: %.2f, Yaw error: %.2f",
                  dist_error, yaw_error);

      geometry_msgs::msg::Twist cmd_vel;

      if (std::abs(yaw_error) > yaw_precision_) {
        // Rotate in place - proportional control
        RCLCPP_INFO(this->get_logger(), "Rotating in place");
        state = "fix yaw";
        double angular_cmd = kp_angular_ * yaw_error;
        cmd_vel.angular.z = std::clamp(angular_cmd, -max_angular_velocity_,
                                       max_angular_velocity_);
      } else if (dist_error > dist_precision_) {
        // Move forward - proportional control
        RCLCPP_INFO(this->get_logger(), "Moving forward");
        state = "go to point";
        double linear_cmd = kp_linear_ * dist_error;
        cmd_vel.linear.x = std::clamp(linear_cmd, 0.1, max_linear_velocity_);
      } else {
        // Goal reached
        RCLCPP_INFO(this->get_logger(), "Goal reached");
        reached_goal = true;
        state = "goal reached";
      }

      cmd_vel_publisher_->publish(cmd_vel);

      // Update feedback
      feedback->position = current_position_;
      feedback->state = state;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    // Stop the robot
    stop_robot();

    // Return result
    result->success = true;
    goal_handle->succeed(result);
  }

  void stop_robot() {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_publisher_->publish(cmd_vel);
  }

  double normalize_angle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<FastbotActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}