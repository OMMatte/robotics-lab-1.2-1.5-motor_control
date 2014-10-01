#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_cartesian_controller");

  ros::NodeHandle n;

  ros::Publisher pwm_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    geometry_msgs::Twist msg;
    msg.linear.x = 0.2;
    msg.angular.z = 0.0;

    pwm_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
