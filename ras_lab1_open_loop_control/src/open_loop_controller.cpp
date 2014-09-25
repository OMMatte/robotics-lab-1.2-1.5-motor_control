#include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_loop_controller");

  ros::NodeHandle n;

  ros::Publisher pwm_pub = n.advertise<ras_arduino_msgs::PWM>("kobuki/pwm", 1);

  ros::Rate loop_rate(100);

  float distanceMeters = 0.0f;
  while (ros::ok())
  {

    ras_arduino_msgs::PWM msg;
    msg.PWM1 = -255;
    msg.PWM2 = 255;

    pwm_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
