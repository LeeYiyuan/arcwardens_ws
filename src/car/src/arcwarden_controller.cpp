#include <ros/ros.h>
#include <people_depth/person.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

double maxAngle = 30.0 ;
double distance_constant = 0.0015;

void callback(const people_depth::person::ConstPtr &person) {
        geometry_msgs::Twist twist_msg;
        int y = person->y; // in pixel
        int z = person->z; // in m.
        int image_width = person->image_width;
        int xPixel = person-x - (image_width/2);
        int x = xPixel*distance_constant*z;

        float distance = sqrt(pow(x,2.0)+pow(z,2.0));
        float angle = atan(x/z);
        float angularVelocity = angle/maxAngle;
        if(angularVelocity>1.0){angularVelocity=1.0;}
        if(angularVelocity<1.0){angularVelocity=-1.0;}

        // Assume that the speed is 0.5 ms-1
        float linearVelocity = 1/64;
        if(distance <= 3.0){linearVelocity=0.0;}

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
