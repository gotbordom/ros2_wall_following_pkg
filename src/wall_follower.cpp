// std headers
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// third party headers
#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

// project headers

using namespace std::chrono_literals;

/// NOTE: This is all done in ROBOT frame of reference.

/// @brief Enum to make code clearer when determining direction of wall
///        following in later code.
enum class WallFollowingDirection {
  NotFound = 0,
  LeftHandSide = 1,
  RightHandSide = 2
};

struct ControllerInfos {

  // Fixed values
  float linear_velocity_stop;
  float linear_velocity_slow;
  float linear_velocity_fast;
  float angular_velocity_stop;
  float angular_velocity_slow;
  float angular_velocity_fast;
  float front_treshold;
  float side_min_threshold;
  float side_max_threshold;

  // Values that shouldn't change once set
  bool wall_found;
  WallFollowingDirection direction_to_follow;

  // values that could change every iteration
  geometry_msgs::msg::Twist next_command;
};

struct OdomInfos {
  float yaw_radians;
  geometry_msgs::msg::PoseWithCovariance pose;
  geometry_msgs::msg::Point distance_front_left;
  geometry_msgs::msg::Point distance_front_right;
  geometry_msgs::msg::Point distance_left;
  geometry_msgs::msg::Point distance_right;
};

struct RangeInfos {
  // Data that should only ever be updated in setup
  // range pairs are [ (inclusive), (exclusive) ]
  // NOTE: front is broken into left and because the it is [0, (1/4)pi] and
  // [(7/4)pi, 2pi] instead of handling a discontinuity I will just make two
  // vectors
  std::pair<int, int> idx_front_left;
  std::pair<int, int> idx_front_right;
  std::pair<int, int> idx_left;
  std::pair<int, int> idx_right;
  // min max pairs are [ min, max ]
  std::pair<float, float> angle_rad_min_max_front_left;
  std::pair<float, float> angle_rad_min_max_front_right;
  std::pair<float, float> angle_rad_min_max_left;
  std::pair<float, float> angle_rad_min_max_right;
  // should be the same for all data ranges
  std::pair<float, float> range_meter_min_max;
  float angle_rad_increment;

  // Data that will be updated every iteration of a state.
  std::vector<float> range_front_left;
  std::vector<float> range_front_right;
  std::vector<float> range_left;
  std::vector<float> range_right;

  // keep track of the min value, and its respective angle it was acquired at
  std::pair<float, float> min_front_left;
  std::pair<float, float> min_front_right;
  std::pair<float, float> min_left;
  std::pair<float, float> min_right;
};

struct RobotState {
  ControllerInfos controller_infos;
  OdomInfos odom_infos;
  RangeInfos range_infos;
};

/// @brief WallFollowerNode class that handles listening to the laser scanner
///        then determining next velocity command, and publishing it to the
///        correct topic.
///        The aim is the have the robot follow the wall at a distance of
///        approximately 0.2 -> 0.3 meters
class WallFollowerNode : public rclcpp::Node {
public:
  WallFollowerNode()
      : Node("wall_follower_node"), laser_scan_setup_done_(false),
        odom_setup_done_(false) {

    // TODO: I have this setup and want to eventually have it load from a
    // config, or launch, file. Hard coded here for now. initilize controller
    curr_state_.controller_infos =
        ControllerInfos{0.0,   // linear stop
                        0.01,  // linear slow
                        0.1,   // linear fast
                        0.0,   // angular stop
                        0.2,   // angular slow
                        0.2,   // angular fast
                        0.5,   // Front threshold for obstacle and walls
                        0.2,   // side minimum follow distance
                        0.3,   // side maximum follow distance
                        false, // wall found
                        WallFollowingDirection::NotFound,
                        geometry_msgs::msg::Twist()};

    callback_group_1_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_3_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_1_;
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&WallFollowerNode::laser_scan_callback, this,
                  std::placeholders::_1),
        options1);

    rclcpp::SubscriptionOptions options2;
    options1.callback_group = callback_group_2_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&WallFollowerNode::odom_callback, this,
                  std::placeholders::_1),
        options2);

    // The scanner is publishing ~ 8Hz or every 125ms.
    // So I want to publish commands at the same or slower (5Hz).
    controller_timer = this->create_wall_timer(
        200ms, std::bind(&WallFollowerNode::controller_callback, this),
        callback_group_3_);

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  /// @brief   The controller callback in charge of publishing velocity values
  ///          based on the current state of the robot.
  auto controller_callback() -> void {
    auto start_time = std::chrono::high_resolution_clock::now();

    /// NOTE: Remember X is forward Y is left Z is up.
    auto dist_to_front_left = curr_state_.odom_infos.distance_front_left.x;
    auto dist_to_front_right = curr_state_.odom_infos.distance_front_right.x;
    auto front_threshold = curr_state_.controller_infos.front_treshold;

    // Wall not found - go find it
    if (!curr_state_.controller_infos.wall_found) {
      // No wall selected, object found  in front
      if (dist_to_front_left < front_threshold ||
          dist_to_front_right < front_threshold) {

        // Stop moving to pick a wall.
        curr_state_.controller_infos.next_command.linear.x =
            curr_state_.controller_infos.linear_velocity_stop;
        curr_state_.controller_infos.next_command.angular.z =
            curr_state_.controller_infos.angular_velocity_stop;

        auto dist_to_right = curr_state_.odom_infos.distance_right;
        auto dist_to_left = curr_state_.odom_infos.distance_left;
        // Make sure we haven't already picked a wall
        if (curr_state_.controller_infos.direction_to_follow ==
            WallFollowingDirection::NotFound) {
          // If right is closer pick it
          if (dist_to_right.y < dist_to_left.y) {
            curr_state_.controller_infos.direction_to_follow =
                WallFollowingDirection::RightHandSide;
            curr_state_.controller_infos.wall_found = true;
          }
          // If left  is closer pick it
          else if (dist_to_right.y > dist_to_left.y) {
            curr_state_.controller_infos.direction_to_follow =
                WallFollowingDirection::LeftHandSide;
            curr_state_.controller_infos.wall_found = true;
          }
          // Otherwise we do nothing
        }

      }
      // No wall selected, nothing in front.
      else {
        // Drive forward until we find one.
        curr_state_.controller_infos.next_command.linear.x =
            curr_state_.controller_infos.linear_velocity_slow;
        curr_state_.controller_infos.next_command.angular.z =
            curr_state_.controller_infos.angular_velocity_stop;
      }
    }
    // We have a wall to follow
    else {
      auto wall_chosen = curr_state_.controller_infos.direction_to_follow;
      auto turn_left = curr_state_.controller_infos.angular_velocity_fast;
      auto turn_right = -1 * curr_state_.controller_infos.angular_velocity_fast;

      // We have encountered something in front of us
      if (dist_to_front_left < front_threshold ||
          dist_to_front_right < front_threshold) {

        // Set linear velocity to slow
        curr_state_.controller_infos.next_command.linear.x =
            curr_state_.controller_infos.linear_velocity_slow;

        // following left so turn right to avoid collision
        if (wall_chosen == WallFollowingDirection::LeftHandSide) {
          RCLCPP_INFO(this->get_logger(),
                      "CONTROLLER CALLBACK: WALL IN FRONT: TURNING RIGHT");
          curr_state_.controller_infos.next_command.angular.z = turn_right;
        }
        // following right so turn left to avoid collision
        else if (wall_chosen == WallFollowingDirection::RightHandSide) {
          RCLCPP_INFO(this->get_logger(),
                      "CONTROLLER CALLBACK: WALL IN FRONT: TURNING LEFT");
          curr_state_.controller_infos.next_command.angular.z = turn_left;
        }
        // otherwise we don't do anything an drop through

      }
      // We have a wall to follow, and no obstacle yet.
      else {
        // Set linear velocity to fast
        curr_state_.controller_infos.next_command.linear.x =
            curr_state_.controller_infos.linear_velocity_fast;

        auto side_dist_min = curr_state_.controller_infos.side_min_threshold;
        auto side_dist_max = curr_state_.controller_infos.side_max_threshold;
        // stay within bounds of left hand side wall
        if (wall_chosen == WallFollowingDirection::LeftHandSide) {
          auto dist_to_left = curr_state_.odom_infos.distance_left.y;
          // Too close - turn away ( turn right )
          if (dist_to_left < side_dist_min) {
            curr_state_.controller_infos.next_command.angular.z = turn_right;
          }
          // Too far - turn towards ( turn left )
          else if (dist_to_left > side_dist_max) {
            curr_state_.controller_infos.next_command.angular.z = turn_left;
          }
          // Otherwise we don't change anything
        }
        // stay within bounds of the right hand side wall
        else if (wall_chosen == WallFollowingDirection::RightHandSide) {
          auto dist_to_right = curr_state_.odom_infos.distance_right.y;
          // Too close - turn away ( turn left )
          if (dist_to_right < side_dist_min) {
            curr_state_.controller_infos.next_command.angular.z = turn_left;
          }
          // Too far - turn towards ( turn right )
          else if (dist_to_right > side_dist_max) {
            curr_state_.controller_infos.next_command.angular.z = turn_right;
          }
          // Otherwise we don't change anything
        }
      }
    }

    print_next_command();
    cmd_vel_pub_->publish(curr_state_.controller_infos.next_command);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time)
                        .count();
    RCLCPP_INFO(this->get_logger(),
                "CONTROLLER CALLBACK: execution time: %ld microseconds",
                duration);
  }

  auto print_next_command() -> void {
    auto wall_found =
        curr_state_.controller_infos.wall_found ? "TRUE" : "FALSE";
    auto wall_enum = curr_state_.controller_infos.direction_to_follow;
    RCLCPP_INFO(this->get_logger(),
                "\nNEXT VEL CMD\n"
                "============\n"
                "LIN: X = %f, Y = %f, Z = %f\n"
                "ANG: X = %f, Y = %f, Z = %f\n"
                "WALL FOUND: %s: %i",
                curr_state_.controller_infos.next_command.linear.x,
                curr_state_.controller_infos.next_command.linear.y,
                curr_state_.controller_infos.next_command.linear.z,
                curr_state_.controller_infos.next_command.angular.x,
                curr_state_.controller_infos.next_command.angular.y,
                curr_state_.controller_infos.next_command.angular.z, wall_found,
                wall_enum);
  }

  /// @brief   The odometry callback is in charge of determining the previous,
  ///          and current, state of the robot given sensor data
  auto odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) -> void {
    auto start_time = std::chrono::high_resolution_clock::now();
    if (!odom_setup_done_) {
      RCLCPP_INFO(this->get_logger(), "ODOMETRY CALLBACK: running setup");
      curr_state_.odom_infos.pose = msg->pose;
      curr_state_.odom_infos.yaw_radians =
          calculate_heading_in_rad(msg->pose.pose.orientation);
      prev_state_ = curr_state_;
      odom_setup_done_ = true;

      print_odom_infos();
    }

    // Always start by moving current to previous
    prev_state_ = curr_state_;

    // store new x, y, yaw
    curr_state_.odom_infos.pose = msg->pose;
    curr_state_.odom_infos.yaw_radians =
        calculate_heading_in_rad(msg->pose.pose.orientation);

    // calculate/store distances
    curr_state_.odom_infos.distance_front_left =
        calculate_vector_components(curr_state_.range_infos.min_front_left);

    curr_state_.odom_infos.distance_front_right =
        calculate_vector_components(curr_state_.range_infos.min_front_right);

    curr_state_.odom_infos.distance_left =
        calculate_vector_components(curr_state_.range_infos.min_left);

    curr_state_.odom_infos.distance_right =
        calculate_vector_components(curr_state_.range_infos.min_right);

    // now print out logging
    print_odom_infos();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time)
                        .count();
    RCLCPP_INFO(this->get_logger(),
                "ODOM CALLBACK: execution time: %ld microseconds", duration);
  }

  auto calculate_heading_in_rad(const geometry_msgs::msg::Quaternion &q)
      -> float {
    auto yaw = std::atan2(2 * (q.x * q.w - q.y * q.z),
                          1 - 2 * (q.x * q.x + q.y * q.y));
    return yaw;
  }

  auto calculate_vector_differences(const geometry_msgs::msg::Point &a,
                                    const geometry_msgs::msg::Point &b)
      -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point diff;
    diff.x = std::abs(b.x - a.x);
    diff.y = std::abs(b.y - a.y);
    diff.z = std::abs(b.z - a.z);

    return diff;
  }

  auto calculate_vector_components(const std::pair<float, float> &v)
      -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point components;
    components.x = std::abs(v.first * std::cos(v.second));
    components.y = std::abs(v.first * std::sin(v.second));
    components.z = 0;
    return components;
  }

  // TODO - I should just overwrite the struct with an << operator or a print
  // function itself
  /// @brief Print out the current state range info
  auto print_odom_infos() -> void {
    RCLCPP_INFO(this->get_logger(),
                "\nODOM STATES\n"
                "===========\n"
                "PREV: X = %f, Y = %f, Yaw = %f\n"
                "CURR: X = %f, Y = %f, Yaw = %f\n",
                prev_state_.odom_infos.pose.pose.position.x,
                prev_state_.odom_infos.pose.pose.position.y,
                prev_state_.odom_infos.yaw_radians,
                curr_state_.odom_infos.pose.pose.position.x,
                curr_state_.odom_infos.pose.pose.position.y,
                curr_state_.odom_infos.yaw_radians);

    RCLCPP_INFO(this->get_logger(),
                "\nODOM DISTANCES\n"
                "==============\n"
                "FRONT LEFT:  dX = %f, dY = %f\n"
                "FRONT RIGHT: dX = %f, dY = %f\n"
                "LEFT:        dX = %f, dY = %f\n"
                "RIGHT:       dX = %f, dY = %f\n",
                curr_state_.odom_infos.distance_front_left.x,
                curr_state_.odom_infos.distance_front_left.y,
                curr_state_.odom_infos.distance_front_right.x,
                curr_state_.odom_infos.distance_front_right.y,
                curr_state_.odom_infos.distance_left.x,
                curr_state_.odom_infos.distance_left.y,
                curr_state_.odom_infos.distance_right.x,
                curr_state_.odom_infos.distance_right.y);
  }

  /// @brief The laser scan callback is tasked with updating the state object
  ///        with the correct Left, Right and Front ranges as well as minimum
  ///        values for each.
  /// @param msg the last published LaserScan object
  /// @note  Unfortunately I can't find documentation on this laser scanner
  /// and
  ///        if angle 0 is ranges[0]... angle 360 is ranges[360]

  /// ASSUMPTION:
  /// 1. 0 degrees is forward
  /// 2. 0 degress DIST is stored in msg->ranges[0]
  /// 3. Degrees increment CCW such that msg->ranges[180] would be backward
  ///    Looking at the data with a debugging topic and rviz2, assumptions 1-3
  ///    are true.
  auto laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
      -> void {

    auto start_time = std::chrono::high_resolution_clock::now();
    if (!laser_scan_setup_done_) {
      RCLCPP_INFO(this->get_logger(), "LASER SCAN CALLBACK: running setup");
      laser_scan_setup_done_ = laser_scan_setup(msg);
    }

    // Make sure to store current in previous state.
    prev_state_ = curr_state_;

    // Load current state
    curr_state_.range_infos.range_front_left = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.second);

    curr_state_.range_infos.range_front_right = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.second);

    curr_state_.range_infos.range_left = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_left.second);

    curr_state_.range_infos.range_right = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_right.second);

    // min values and their angles
    auto it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.second);
    auto min_value_angle = std::distance(msg->ranges.begin(), it) *
                           curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_front_left = {*it, min_value_angle};

    it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.second);
    min_value_angle = std::distance(msg->ranges.begin(), it) *
                      curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_front_right = {*it, min_value_angle};

    it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_left.second);
    min_value_angle = std::distance(msg->ranges.begin(), it) *
                      curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_left = {*it, min_value_angle};

    it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_right.second);
    min_value_angle = std::distance(msg->ranges.begin(), it) *
                      curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_right = {*it, min_value_angle};

    print_range_infos();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time)
                        .count();
    RCLCPP_INFO(this->get_logger(),
                "LASER SCAN CALLBACK: execution time: %ld microseconds",
                duration);
  }

  //  TODO - This needs to be broken into smaller helper functions that can
  //  all get unit tested.
  /// @brief   Run the setup for the laser scan callback. This will calculate
  ///          the correct index values, etc for each range and store them.
  /// @return  boolean - if configuration was successful.
  auto laser_scan_setup(const sensor_msgs::msg::LaserScan::SharedPtr msg)
      -> bool {

    // Set the data we take directly from the msg
    curr_state_.range_infos.angle_rad_increment = msg->angle_increment;
    curr_state_.range_infos.range_meter_min_max.first = msg->range_min;
    curr_state_.range_infos.range_meter_min_max.second = msg->range_max;

    // Set the index values to use for each range.
    // Since each messaage gets a full 360 readings for a single 360 degree
    // rotation, these numbers are easier to use.
    curr_state_.range_infos.idx_front_left = {0, 5};
    curr_state_.range_infos.idx_front_right = {355, 360};
    curr_state_.range_infos.idx_left = {45, 135};
    curr_state_.range_infos.idx_right = {225, 315};

    // Set the radian min/max ranges encase I need to compute the degree of
    // any of my stored range's elements store the radiands start  and end
    // values for each range.
    const float PI = 3.14;
    const float degree_to_rad = PI / 180;
    curr_state_.range_infos.angle_rad_min_max_front_left = {
        degree_to_rad * curr_state_.range_infos.idx_front_left.first,
        degree_to_rad * curr_state_.range_infos.idx_front_left.second};
    curr_state_.range_infos.angle_rad_min_max_front_right = {
        degree_to_rad * curr_state_.range_infos.idx_front_right.first,
        degree_to_rad * curr_state_.range_infos.idx_front_right.second};
    curr_state_.range_infos.angle_rad_min_max_left = {
        degree_to_rad * curr_state_.range_infos.idx_left.first,
        degree_to_rad * curr_state_.range_infos.idx_left.second};
    curr_state_.range_infos.angle_rad_min_max_right = {
        degree_to_rad * curr_state_.range_infos.idx_right.first,
        degree_to_rad * curr_state_.range_infos.idx_right.second};

    // Store ranges
    curr_state_.range_infos.range_front_left = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.second);

    curr_state_.range_infos.range_front_right = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.second);

    curr_state_.range_infos.range_left = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_left.second);

    curr_state_.range_infos.range_right = std::vector<float>(
        msg->ranges.begin() + curr_state_.range_infos.idx_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_right.second);

    // Save initial min value per range
    auto it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_left.second);
    auto min_value_angle = std::distance(msg->ranges.begin(), it) *
                           curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_front_left = {*it, min_value_angle};

    it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_front_right.second);
    min_value_angle = std::distance(msg->ranges.begin(), it) *
                      curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_front_right = {*it, min_value_angle};

    it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_left.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_left.second);
    min_value_angle = std::distance(msg->ranges.begin(), it) *
                      curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_left = {*it, min_value_angle};

    it = std::min_element(
        msg->ranges.begin() + curr_state_.range_infos.idx_right.first,
        msg->ranges.begin() + curr_state_.range_infos.idx_right.second);
    min_value_angle = std::distance(msg->ranges.begin(), it) *
                      curr_state_.range_infos.angle_rad_increment;
    curr_state_.range_infos.min_right = {*it, min_value_angle};

    prev_state_ = curr_state_;
    print_range_infos();
    return true;
  }

  // TODO - I should just overwrite the struct with an << operator or a print
  // function itself
  /// @brief Print out the current state range info
  auto print_range_infos() -> void {
    // const auto info = curr_state_.range_infos;
    RCLCPP_INFO(this->get_logger(),
                "\nCURRENT STATE\n"
                "=============\n"
                "LEFT:        SIZE = %d, MIN VALUE = %f, MIN ANGLE = %f\n"
                "RIGHT:       SIZE = %d, MIN VALUE = %f, MIN ANGLE = %f\n"
                "FRONT LEFT:  SIZE = %d, MIN VALUE = %f, MIN ANGLE = %f\n"
                "FRONT RIGHT: SIZE = %d, MIN VALUE = %f, MIN ANGLE = %f\n",
                curr_state_.range_infos.range_left.size(),
                curr_state_.range_infos.min_left.first,
                curr_state_.range_infos.min_left.second,
                curr_state_.range_infos.range_right.size(),
                curr_state_.range_infos.min_right.first,
                curr_state_.range_infos.min_right.second,
                curr_state_.range_infos.range_front_left.size(),
                curr_state_.range_infos.min_front_left.first,
                curr_state_.range_infos.min_front_left.second,
                curr_state_.range_infos.range_front_right.size(),
                curr_state_.range_infos.min_front_right.first,
                curr_state_.range_infos.min_front_right.second);

    std::stringstream front_left_ss, front_right_ss, left_ss, right_ss;
    for (const auto &val : curr_state_.range_infos.range_front_left)
      front_left_ss << val << " ";
    for (const auto &val : curr_state_.range_infos.range_front_right)
      front_right_ss << val << " ";
    for (const auto &val : curr_state_.range_infos.range_left)
      left_ss << val << " ";
    for (const auto &val : curr_state_.range_infos.range_right)
      right_ss << val << " ";

    std::cout << "SCAN RANGES:\n"
                 "===========\n";
    std::cout << "FRONT LEFT:  " << front_left_ss.str() << "\n";
    std::cout << "LEFT:        " << left_ss.str() << "\n";
    std::cout << "RIGHT:       " << right_ss.str() << "\n";
    std::cout << "FRONT RIGHT: " << front_right_ss.str() << "\n";
    // RCLCPP_INFO(this->get_logger(),
    //             "\nSCAN RANGES\n"
    //             "===========\n"
    //             "FRONT LEFT:  %s\n"
    //             "LEFT:        %s\n"
    //             "RIGHT:       %s\n"
    //             "FRONT RIGHT: %s\n",
    //             front_left_ss.str(), left_ss.str(), right_ss.str(),
    //             front_right_ss.str());
  }

  RobotState curr_state_, prev_state_;
  bool laser_scan_setup_done_, odom_setup_done_;
  rclcpp::CallbackGroup::SharedPtr callback_group_1_, callback_group_2_,
      callback_group_3_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr controller_timer;
};

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);

  auto wall_follower_node = std::make_shared<WallFollowerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(wall_follower_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}