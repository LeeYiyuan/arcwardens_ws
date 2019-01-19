#include <string.h>
#include <cstdint>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class LowLevelController
{
public:
  LowLevelController();

private:
  ros::Subscriber cmd_vel_sub_;
  serial::Serial serial_;
 
  void cmdVelCallback(geometry_msgs::Twist::ConstPtr const& cmd_vel);
};

LowLevelController::LowLevelController()
{
  ros::NodeHandle handle;
  ros::NodeHandle handle_private("~");

  std::string cmd_vel_topic;
  ROS_ASSERT(handle_private.getParam("cmd_vel_topic", cmd_vel_topic));
  std::string port;
  ROS_ASSERT(handle_private.getParam("port", port));
  int baudrate;
  ROS_ASSERT(handle_private.getParam("baudrate", baudrate));
  cmd_vel_sub_ = handle.subscribe(cmd_vel_topic, 1, &LowLevelController::cmdVelCallback, this);

  try
  {
    serial_.setPort(port);
    serial_.setBaudrate(baudrate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(timeout);
    serial_.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Failed to open low level serial port : " << port);
    return;
  }

  ROS_INFO_STREAM("Low level serial port opened." << port);
}

void LowLevelController::cmdVelCallback(geometry_msgs::Twist::ConstPtr const& cmd_vel)
{
  std::vector<uint8_t> packet;
  packet.emplace_back(0x3F); // Header.
  packet.emplace_back((uint8_t)(50 + cmd_vel->linear.x * 50)); // Throttle.
  packet.emplace_back((uint8_t)(50 + cmd_vel->angular.z * 50)); // Steering.
  packet.emplace_back(0x4F); // Checksum.

  serial_.write(packet);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "low_level_controller");
  LowLevelController low_level_controller;
  ros::spin();
}
