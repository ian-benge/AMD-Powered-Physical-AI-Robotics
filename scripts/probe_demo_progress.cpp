// Exercise the installed Nav2 plugins, including their actual ROS-clock timeout.
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/progress_checker.hpp>
#include <pluginlib/class_loader.hpp>
#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    if (argc != 2) {throw std::runtime_error("Pass the recorded return CSV");}
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("progress_probe");
    for (const auto & name : {"simple", "pose"}) {
      node->declare_parameter(std::string(name) + ".required_movement_radius", 0.10);
      node->declare_parameter(std::string(name) + ".movement_time_allowance", 20.0);
    }
    node->declare_parameter("pose.required_movement_angle", 0.50);
    pluginlib::ClassLoader<nav2_core::ProgressChecker> loader("nav2_core", "nav2_core::ProgressChecker");
    auto simple = loader.createSharedInstance("nav2_controller::SimpleProgressChecker");
    auto pose = loader.createSharedInstance("nav2_controller::PoseProgressChecker");
    simple->initialize(node, "simple");
    pose->initialize(node, "pose");
    auto clock = node->get_clock()->get_clock_handle();
    if (rcl_enable_ros_time_override(clock) != RCL_RET_OK) {
      throw std::runtime_error("Cannot control isolated probe clock");
    }
    auto query = [&](const std::shared_ptr<nav2_core::ProgressChecker> & checker,
        double seconds, double x, double y, double yaw) {
        if (rcl_set_ros_time_override(clock, static_cast<int64_t>((100.0 + seconds) * 1e9)) != RCL_RET_OK) {
          throw std::runtime_error("Cannot set probe time");
        }
        geometry_msgs::msg::PoseStamped sample;
        sample.pose.position.x = x;
        sample.pose.position.y = y;
        sample.pose.orientation.z = std::sin(yaw / 2.0);
        sample.pose.orientation.w = std::cos(yaw / 2.0);
        return checker->check(sample);
      };
    auto require = [](bool condition, const std::string & reason) {
        if (!condition) {throw std::runtime_error(reason);}
      };
    // Recorded V25 rotation: the last pose is held for 0.25 s to cover the
    // controller's abort instant, which fell just after the final trajectory.
    std::ifstream input(argv[1]);
    require(input.good(), "Missing recorded trajectory");
    double t, x, y, yaw, last_t = -1, last_x = 0, last_y = 0, last_yaw = 0;
    bool simple_failed = false;
    size_t count = 0;
    while (input >> t >> x >> y >> yaw) {
      require(t >= 0 && t > last_t, "Trajectory time must increase");
      simple_failed |= !query(simple, t, x, y, yaw);
      require(query(pose, t, x, y, yaw), "Rotation-aware checker rejected recorded progress");
      last_t = t; last_x = x; last_y = y; last_yaw = yaw; ++count;
    }
    require(count > 50 && last_t >= 19.0, "Recorded return is too short");
    simple_failed |= !query(simple, last_t + .25, last_x, last_y, last_yaw);
    require(query(pose, last_t + .25, last_x, last_y, last_yaw), "Rotation-aware checker rejected final pose");
    require(simple_failed, "Recorded return did not reproduce translation-only timeout");
    for (auto checker : {simple, pose}) {
      checker->reset();
      require(query(checker, 0., 0., 0., 0.), "Initial pose rejected");
      require(query(checker, 20., 0., 0., 0.), "Deadline boundary changed");
      require(!query(checker, 20.001, 0., 0., 0.), "Stationary stall did not expire");
      checker->reset();
      require(query(checker, 0., 0., 0., 0.), "Initial jitter pose rejected");
      require(!query(checker, 20.001, .01, -.01, .02), "Small pose noise bypassed timeout");
      checker->reset();
      require(query(checker, 0., 0., 0., 0.), "Initial translation pose rejected");
      require(query(checker, 19., .11, 0., 0.), "Measured translation was rejected");
      require(query(checker, 39., .11, 0., 0.), "Translation deadline boundary changed");
      require(!query(checker, 39.001, .11, 0., 0.), "Stall after translation did not expire");
    }
    pose->reset();
    for (int step = 0; step <= 300; ++step) {
      require(query(pose, step / 10., 0., 0., step / 10. * .3), "Continuous rotation rejected at yaw wrap");
    }
    std::cout << "{\"passed\":true,\"recorded_samples\":" << count
              << ",\"translation_only_reproduction\":true,\"stationary_timeout_seconds\":20.0,"
              << "\"translation_radius_m\":0.1,\"rotation_threshold_rad\":0.5,"
              << "\"stationary_noise_and_translation_stalls_rejected\":true,\"yaw_wrap_passed\":true}" << std::endl;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
