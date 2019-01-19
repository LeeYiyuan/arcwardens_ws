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

  double distance = sqrt(pow(x,2.0)+pow(z,2.0));
  double angle = -atan(x/z);
  double angularVelocity = angle/maxAngle;
  /*if(angularVelocity>1.0){angularVelocity=1.0;}
  if(angularVelocity<1.0){angularVelocity=-1.0;}
  */

  ROS_INFO_STREAM("Distance = " << distance << ", Angle = " << angle << ", Angular Velocity = " << angularVelocity);
  // Assume that the speed is 0.5 ms-1
  double linearVelocity = 1.0/64;
  //if(!distance.isNaN()){linearVelocity=0.0;}

  twist_msg.angular.z=angularVelocity;
  twist_msg.linear.x = linearVelocity;
  pub.publish(twist_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arcwarden_controller");
  ros::NodeHandle nh;
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::Subscriber sub = nh.subscribe("/person", 1, callback);

  ros::spin();

}
