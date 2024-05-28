#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cmath> // for std::isinf()

using namespace std::chrono_literals; // for 200ms
using std::placeholders::_1;          // for callback parameter placeholder in
                                      // std::bind()

class BallAvoider : public rclcpp::Node {
public:
  BallAvoider() : Node("topics_quiz_node") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&BallAvoider::velocity_callback, this));
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&BallAvoider::laser_scan_callback, this, _1));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  sensor_msgs::msg::LaserScan laser_scan_data_;
  geometry_msgs::msg::Twist vel_cmd_msg_;

  // angle ranges
  const int RIGHT_FROM = 0, RIGHT_TO = 89;
  const int LEFT_FROM = 630, LEFT_TO = 719;
  const int FRONT_FROM = 220, FRONT_TO = 499; // wider by 50 degrees

  const double VELOCITY_INCREMENT = 0.1;
  const double ANGULAR_BASE = 0.5;
  const double LINEAR_BASE = 0.8;

  // publisher
  void velocity_callback();

  // subscriber
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  bool obstacle_in_range(int from, int to, double dist);
};

// callbacks

// publisher
void BallAvoider::velocity_callback() {
  /*
  Applroximate logic:

  1. If the laser reading in front of the robot is higher than one meter (there
     is no obstacle closer than one meter in front of the robot), the robot will
     move forward.
  2. If the laser reading in front of the robot is lower than one meter
     (there is an obstacle closer than one meter in front of the robot), the
  robot will turn left.
  3. If the laser reading on the right side of the robot is lower
     than one meter (there is an obstacle closer than one meter on the right
  side of the robot), the robot will turn left.
  4. If the laser reading on the left side
     of the robot is lower than one meter (there is an obstacle closer than one
     meter on the left side of the robot), the robot will turn right.

  A robust algorithm would be a lot more complex. This one is fun :)
  */

  if (!obstacle_in_range(FRONT_FROM, FRONT_TO, 1.0)) {
    vel_cmd_msg_.linear.x = 0.8;
    vel_cmd_msg_.angular.z = 0.0;
  } else {
    // stop forward motion
    vel_cmd_msg_.linear.x = 0.0;

    bool obstacle_left = obstacle_in_range(LEFT_FROM, LEFT_TO, 1.0);
    bool obstacle_right = obstacle_in_range(RIGHT_FROM, RIGHT_TO, 1.0);
    bool turning_left = vel_cmd_msg_.angular.z > 0.0;
    bool turning_right = vel_cmd_msg_.angular.z < 0.0;

    // this one is robus but for a corner case of going straight very near a wall
    // can correct with some logic to anticipate obstacles in front
    if (obstacle_left && turning_right) {
      vel_cmd_msg_.angular.z += - VELOCITY_INCREMENT;
    } else if (obstacle_right && turning_left) {
      vel_cmd_msg_.angular.z += VELOCITY_INCREMENT;
    } else {
      vel_cmd_msg_.angular.z = 0.5;
    }
  }

  publisher_->publish(vel_cmd_msg_);
}

// subscriber
void BallAvoider::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  laser_scan_data_ = *msg;
  // ranges[0] is to the right
  // ranges[719] is to the left
  RCLCPP_DEBUG(this->get_logger(), "Distance to the left is %f",
               laser_scan_data_.ranges[719]);
}

bool BallAvoider::obstacle_in_range(int from, int to, double dist) {
  bool is_obstacle = false;
  for (int i = from; i <= to; ++i)
    // inf >> dist
    if (!std::isinf(laser_scan_data_.ranges[i]))
      if (laser_scan_data_.ranges[i] <= dist)
        is_obstacle = true;
  return is_obstacle;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallAvoider>());
  rclcpp::shutdown();
  return 0;
}