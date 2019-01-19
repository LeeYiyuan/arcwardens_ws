#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

class JoystickTeleop
{
public:
  JoystickTeleop();

private:
  ros::NodeHandle nh_;

  ros::Subscriber stop_flag_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber cmd_vel_nav_sub_;
  ros::Publisher cmd_vel_pub_;

  std::string topic_stop_flag_;
  std::string topic_cmd_vel_nav_;
  std::string topic_cmd_vel_;
  int button_nav_;
  int button_manual_;
  int axis_linear_;
  int axis_angular_;
  double scale_linear_;
  double scale_angular_;
  bool is_nav_ = false;
  bool is_stop_ = false;
  geometry_msgs::Twist cmd_vel_nav_;
  geometry_msgs::Twist cmd_vel_manual_;
  
  void stopFlagCallback(const std_msgs::Bool::ConstPtr& stop_flag);
  void navCmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_nav);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();
};

JoystickTeleop::JoystickTeleop()
{
  ros::NodeHandle nh_private("~");
  nh_private.getParam("topic_stop_flag", topic_stop_flag_);
  nh_private.getParam("topic_cmd_vel_nav", topic_cmd_vel_nav_);
  nh_private.getParam("topic_cmd_vel", topic_cmd_vel_);
  nh_private.getParam("button_nav", button_nav_);
  nh_private.getParam("button_manual", button_manual_);
  nh_private.getParam("axis_linear", axis_linear_);
  nh_private.getParam("axis_angular", axis_angular_);
  nh_private.param("scale_linear", scale_linear_, 1.0);
  nh_private.param("scale_angular", scale_angular_, 1.0);

  stop_flag_sub_ = nh_.subscribe<std_msgs::Bool>(topic_stop_flag_, 1, &JoystickTeleop::stopFlagCallback, this);
  cmd_vel_nav_sub_ =
      nh_.subscribe<geometry_msgs::Twist>(topic_cmd_vel_nav_, 1, &JoystickTeleop::navCmdVelCallback, this);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &JoystickTeleop::joyCallback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_cmd_vel_, 1);
      
  geometry_msgs::Twist cv;
  cmd_vel_pub_.publish(cv);
}

void JoystickTeleop::publish()
{
  if (is_nav_)
  {
    if (is_stop_)
    {
      geometry_msgs::Twist cv;
      cmd_vel_pub_.publish(cv);
    }
    else
    {
      cmd_vel_pub_.publish(cmd_vel_nav_);
    }
  }
  else
  {
    cmd_vel_pub_.publish(cmd_vel_manual_);
  }
}

void JoystickTeleop::stopFlagCallback(const std_msgs::Bool::ConstPtr& stop_flag)
{
  is_stop_ = stop_flag->data;
  publish();
}

void JoystickTeleop::navCmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_nav)
{
  cmd_vel_nav_.linear.x = cmd_vel_nav->linear.x;
  cmd_vel_nav_.angular.z = cmd_vel_nav->angular.z;

  publish();
}

void JoystickTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (joy->buttons[button_manual_])
    is_nav_ = false;
  else if (joy->buttons[button_nav_])
    is_nav_ = true;

  cmd_vel_manual_.linear.x = scale_linear_ * (joy->axes[axis_linear_]);
  cmd_vel_manual_.angular.z = scale_angular_ * (joy->axes[axis_angular_]);

  publish();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_teleop");
  JoystickTeleop joystick_teleop;

  ros::spin();
}
