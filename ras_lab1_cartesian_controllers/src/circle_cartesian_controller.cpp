#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

const float PI =  3.14159265;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "circle_cartesian_controller");

  ros::NodeHandle n;

  ros::Publisher pwm_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    float angularSpeed = 2.0 * PI / 10;
    float linearSpeed = 0.5 * angularSpeed;
    geometry_msgs::Twist msg;
    msg.linear.x = linearSpeed;
    msg.angular.z = angularSpeed;

    pwm_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
