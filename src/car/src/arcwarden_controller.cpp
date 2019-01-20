#include <ros/ros.h>
#include <car/person.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

double maxAngle = M_PI / 6;
double distance_constant = 0.0015;

ros::Publisher pub;

void callback(const car::person::ConstPtr &person) {
  geometry_msgs::Twist twist_msg;
  int y = person->y; // in pixel
  double z = person->z; // in m.
  int image_width = person->image_width;
  int xPixel = person->x - (image_width/2);
  double x = xPixel*distance_constant*z;

  if  (std::isnan(z) || z <= 1.5) {
    ROS_INFO_STREAM("Distance = NaN");

    twist_msg.linear.x = 0;
    twist_msg.angular.z = 0;
  } else {
    double distance = sqrt(pow(x,2.0)+pow(z,2.0));
    double angle = -atan(x/z);
    double angularVelocity = angle/maxAngle;

  ROS_INFO_STREAM("Distance = " << distance << ", Angle = " << angle << ", Angular Velocity = " << angularVelocity);

    twist_msg.linear.x = 0.3;
    twist_msg.angular.z = angularVelocity;
  }

  pub.publish(twist_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arcwarden_controller");
  ros::NodeHandle nh;
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::Subscriber sub = nh.subscribe("/person", 1, callback);

  ros::spin();

}
