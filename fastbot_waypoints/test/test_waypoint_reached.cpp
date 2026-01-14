#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "action_msgs/msg/goal_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "gtest/gtest.h"

#include "fastbot_waypoints/action/waypoint.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

using namespace std::chrono_literals;
using Waypoint = fastbot_waypoints::action::Waypoint;
using GoalHandleWaypoint = rclcpp_action::ClientGoalHandle<Waypoint>;

#define CRASH_IF_FATAL(stmt)                                                   \
  do {                                                                         \
    EXPECT_NO_FATAL_FAILURE(stmt);                                             \
    if (::testing::Test::HasFatalFailure()) {                                  \
      std::abort();                                                            \
    }                                                                          \
  } while (0)

// Tolerances
constexpr double POSITION_TOLERANCE = 0.1; // 10 cm
constexpr double YAW_TOLERANCE = 0.35;     // ~20 degrees

// Target waypoint
struct Waypoint2D {
  double x;
  double y;
};

// ============================================================
// CHANGE TARGETS TO TEST FAILING CONDITIONS
// Set: -5.0, -5.0
// ============================================================
// Passing: Correct target coordinates
constexpr Waypoint2D TARGET_1 = {1.0, 1.0};
constexpr Waypoint2D TARGET_2 = {1.5, 1.5};

// Normalize angle to [-pi, pi] (pure function)
inline double normalizeAngle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

// Test fixture for Waypoint action client tests
class FastbotActionTestFixture : public ::testing::Test {
protected:
  // static void SetUpTestSuite() {
  //   rclcpp::init(0, nullptr);  // Or pass argc/argv if needed
  // }

  // static void TearDownTestSuite() {
  //   rclcpp::shutdown();
  // }

  void SetUp() override {
    node_ = rclcpp::Node::make_shared("waypoint_action_test");

    action_client_ =
        rclcpp_action::create_client<Waypoint>(node_, "fastbot_as");
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          current_odom_ = msg;
          odom_received_ = true;
        });

    // Wait for server (short timeout for failing condition test)
    ASSERT_TRUE(action_client_->wait_for_action_server(10s))
        << "Action server not available";

    // Wait for odometry
    auto start = std::chrono::steady_clock::now();
    while (!odom_received_ && rclcpp::ok()) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(100ms);
      if (std::chrono::steady_clock::now() - start > 5s) {
        FAIL() << "Timeout waiting for odometry";
      }
    }
  }

  void TearDown() override {
    // Cancel any ongoing goal
    if (goal_handle_) {
      auto status = goal_handle_->get_status();
      if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
          status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        action_client_->async_cancel_goal(goal_handle_);
        rclcpp::spin_some(node_);
      }
      goal_handle_.reset();
    }
    current_odom_.reset();
  }

  void sendGoalAndWait(double x, double y) {
    auto goal_msg = Waypoint::Goal();
    goal_msg.position.x = x;
    goal_msg.position.y = y;
    goal_msg.position.z = 0.0;

    auto send_goal_options = rclcpp_action::Client<Waypoint>::SendGoalOptions();

    auto send_goal_future =
        action_client_->async_send_goal(goal_msg, send_goal_options);

    // Wait for goal to be accepted, spinning the node while waiting
    auto send_status = rclcpp::spin_until_future_complete(
        node_, send_goal_future, 5s); // Increase timeout if needed

    // if (send_status != rclcpp::FutureReturnCode::SUCCESS) {
    //   FAIL() << "Failed to send goal (timeout or interrupted)";
    //   return;
    // }

    // goal_handle_ = send_goal_future.get();
    // if (!goal_handle_) {
    //   FAIL() << "Goal rejected by server";
    //   return;
    // }

    ASSERT_EQ(send_status, rclcpp::FutureReturnCode::SUCCESS)
        << "Failed to send goal (timeout or interrupted)";

    goal_handle_ = send_goal_future.get();
    ASSERT_TRUE(goal_handle_) << "Goal rejected by server";

    // Wait for result
    auto result_future = action_client_->async_get_result(goal_handle_);
    auto status = rclcpp::spin_until_future_complete(node_, result_future, 20s);

    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS)
        << "Timeout waiting for result";

    auto result = result_future.get();
    ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED)
        << "Action failed";

    // Spin a bit more to get final odom
    for (int i = 0; i < 10; ++i) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(50ms);
    }
  }

  // Get current Yaw from odom
  double getYaw() {
    tf2::Quaternion q(current_odom_->pose.pose.orientation.x,
                      current_odom_->pose.pose.orientation.y,
                      current_odom_->pose.pose.orientation.z,
                      current_odom_->pose.pose.orientation.w);
    return tf2::getYaw(q);
  }

  // Member variables
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Waypoint>::SharedPtr action_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  GoalHandleWaypoint::SharedPtr goal_handle_;

  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  bool odom_received_ = false;
};

// Check end position [X, Y] is correct within tolerance
TEST_F(FastbotActionTestFixture, TestEndPosition) {
  //   ASSERT_NO_FATAL_FAILURE(sendGoalAndWait(TARGET_1.x, TARGET_1.y));
  CRASH_IF_FATAL(sendGoalAndWait(TARGET_1.x, TARGET_1.y));

  // Check: End position [X, Y] within tolerance
  EXPECT_NEAR(current_odom_->pose.pose.position.x, TARGET_1.x,
              POSITION_TOLERANCE)
      << "X position error";
  EXPECT_NEAR(current_odom_->pose.pose.position.y, TARGET_1.y,
              POSITION_TOLERANCE)
      << "Y position error";
}

// Check end rotation [Yaw] is correct within tolerance
TEST_F(FastbotActionTestFixture, TestEndYaw) {
  //   ASSERT_NO_FATAL_FAILURE(sendGoalAndWait(TARGET_2.x, TARGET_2.y));
  CRASH_IF_FATAL(sendGoalAndWait(TARGET_2.x, TARGET_2.y));

  // Check: End yaw within tolerance
  double expected_yaw =
      std::atan2(TARGET_2.y, TARGET_2.x); // direction to target from origin
  double current_yaw = getYaw();
  double yaw_error = std::abs(normalizeAngle(current_yaw - expected_yaw));
  EXPECT_LT(yaw_error, YAW_TOLERANCE)
      << "Yaw: " << current_yaw << ", expected: " << expected_yaw
      << ", error: " << yaw_error;
}

// Test 3: GTEST_SKIP() = skipped
// TEST_F(FastbotActionTestFixture, TestEndYaw) {
//   //   GTEST_SKIP() << "Failing condition: This test is skipped";
// }

// Custom main with ROS init
// int main(int argc, char** argv) {
//   testing::InitGoogleTest(&argc, argv);
//   rclcpp::init(argc, argv);  // Pass args for remappings, etc.

//   int result = RUN_ALL_TESTS();

//   rclcpp::shutdown();
//   return result;
// }