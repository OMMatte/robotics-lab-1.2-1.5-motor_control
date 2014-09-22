#include "ros/ros.h"
#include "ras_arduino_msgs/PWM.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/Encoders.h"
#include <math.h>

#define PI 3.14159265

const float ALPHA = 1;
const float BIAS = 0.23;
const float WHEEL_RADIUS = 0.0352;


class MotorControllerNode
{
public:

    ros::NodeHandle n_;
    ros::Subscriber car_sub;
    ros::Subscriber enc_sub;
    ros::Publisher pwm_pub;

    MotorControllerNode()
    {
        old_pwr_left_ = 0;
        old_pwr_right = 0;
        linear_velocity_ = 0;
        angular_velocity_ = 0;
        wheel_encoder_delta_left_ = 0;
        wheel_encoder_delta_right_ = 0;
        desired_w_left_ = 0;
        desired_w_right_ = 0;

        n_ = ros::NodeHandle();
        car_sub = n_.subscribe("motor_controller/twist", 1, &MotorControllerNode::cartesianCallback, this);

        enc_sub = n_.subscribe("kobuki/encoders", 1, &MotorControllerNode::encoderCallback, this);

        pwm_pub = n_.advertise<ras_arduino_msgs::PWM>("kobuki/pwm", 1);

    }

    void cartesianCallback(const geometry_msgs::Twist::ConstPtr &msg)
        {
            linear_velocity_ = msg->linear.x;
            angular_velocity_ = msg->angular.z;

            desired_w_left_ = getWheelAngularVelocity(linear_velocity_, angular_velocity_, true);
            desired_w_right_ = getWheelAngularVelocity(linear_velocity_, angular_velocity_, false);

            ROS_INFO("desired_w_left is %f", desired_w_left_);
            ROS_INFO("message_w_right is %f", desired_w_right_);

        }

    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
    {
        wheel_encoder_delta_left_ =  msg->delta_encoder1;
        wheel_encoder_delta_right_ = msg->delta_encoder2;

        ROS_INFO("wheel_encoder_delta_left is %i", wheel_encoder_delta_left_);
        ROS_INFO("wheel_encoder_delta_right is %i", wheel_encoder_delta_right_);

        estimated_w_left_ = wheel_encoder_delta_left_*(M_PI / 180) * 10;
        estimated_w_right_ = wheel_encoder_delta_right_*(M_PI / 180) * 10;

        ROS_INFO("estimated_w_left_ is %f", estimated_w_left_);
        ROS_INFO("estimated_w_right is %f", estimated_w_right_);
    }

    void update()
    {

        ras_arduino_msgs::PWM pwm;

        pwm.PWM1 = old_pwr_left_ + 0.5 + ALPHA*(desired_w_left_ - estimated_w_left_);
        pwm.PWM2 = old_pwr_right + 0.5 + ALPHA*(desired_w_right_ - estimated_w_right_);

        ROS_INFO("old_pwr_left is %i", old_pwr_left_);
        ROS_INFO("old_pwr_right is %i", old_pwr_right);
        ROS_INFO("new_pwr_left is %i", pwm.PWM1);
        ROS_INFO("new_pwr_right is %i", pwm.PWM2);

        old_pwr_left_ = pwm.PWM1;
        old_pwr_right = pwm.PWM2;

        pwm_pub.publish(pwm);

    }

private:
    int old_pwr_left_;
    int old_pwr_right;

    int wheel_encoder_delta_left_;
    int wheel_encoder_delta_right_;

    float linear_velocity_;
    float angular_velocity_;

    float desired_w_left_;
    float desired_w_right_;
    float estimated_w_left_;
    float estimated_w_right_;

    float getWheelAngularVelocity(float velocity, float angularVelocity, bool left_wheel)
    {

        return (velocity + (BIAS/2)*angularVelocity * (left_wheel ? -1 : 1)) / WHEEL_RADIUS;
    }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");

  MotorControllerNode motor_controller_node;

  ros::Rate loop_rate(10);

  float distanceMeters = 0.0f;
  while (motor_controller_node.n_.ok())
  {
    motor_controller_node.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


