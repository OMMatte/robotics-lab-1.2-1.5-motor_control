#include "ros/ros.h"
#include "ras_arduino_msgs/PWM.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/Encoders.h"
#include <math.h>

#define PI 3.14159265

const float ALPHA = 1.0;
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
        wheel_encoder_left_ = 0;
        wheel_encoder_right_ = 0;
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

        }

    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
    {
        wheel_encoder_left_ =  msg->delta_encoder1;
        wheel_encoder_right_ = msg->delta_encoder2;

        estimated_w_left_ = wheel_encoder_left_*(180 / M_PI) * 10;
        estimated_w_right_ = wheel_encoder_right_*(180 / M_PI) * 10;
    }

    void update()
    {

        ras_arduino_msgs::PWM pwm;

        pwm.PWM1 = old_pwr_left_ + ALPHA*(desired_w_left_ - estimated_w_left_);
        pwm.PWM2 = old_pwr_right + ALPHA*(desired_w_right_ - estimated_w_right_);

        old_pwr_left_ = pwm.PWM1;
        old_pwr_right = pwm.PWM2;

        pwm_pub.publish(pwm);

    }

private:
    int old_pwr_left_;
    int old_pwr_right;

    float wheel_encoder_left_;
    float wheel_encoder_right_;

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


