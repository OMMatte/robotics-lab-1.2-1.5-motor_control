#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/PWM.h>
#include <math.h>

const float PI =  3.14159265;
const float ALPHA = 1;
const float BIAS = 0.23;
const float WHEEL_RADIUS = 0.0352;


class MotorControllerNode
{
public:

    ros::NodeHandle n;

    MotorControllerNode() :
        old_pwr_left(0),
        old_pwr_right(0),
        desired_w_left(0),
        desired_w_right(0),
        estimated_w_left(0),
        estimated_w_right(0)
    {
        n = ros::NodeHandle();

        car_sub = n.subscribe("motor_controller/twist", 1, &MotorControllerNode::cartesianCallback, this);
        enc_sub = n.subscribe("kobuki/encoders", 1, &MotorControllerNode::encoderCallback, this);
        pwm_pub = n.advertise<ras_arduino_msgs::PWM>("kobuki/pwm", 1);

    }

    void cartesianCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        setDesiredAngularVelocity(msg->linear.x, msg->angular.z);
    }

    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
    {
        setEstimatedAngularVelocity(msg->delta_encoder1, msg->delta_encoder2);
    }

    void update()
    {

        ras_arduino_msgs::PWM pwm;

        pwm.PWM1 = old_pwr_left + 0.5 + ALPHA*(desired_w_left - estimated_w_left);
        pwm.PWM2 = old_pwr_right + 0.5 + ALPHA*(desired_w_right - estimated_w_right);

        ROS_INFO("old_pwr_left is %i", old_pwr_left);
        ROS_INFO("old_pwr_right is %i", old_pwr_right);
        ROS_INFO("new_pwr_left is %i", pwm.PWM1);
        ROS_INFO("new_pwr_right is %i", pwm.PWM2);

        old_pwr_left = pwm.PWM1;
        old_pwr_right = pwm.PWM2;

        pwm_pub.publish(pwm);

    }

private:
    int old_pwr_left;
    int old_pwr_right;

    float desired_w_left;
    float desired_w_right;
    float estimated_w_left;
    float estimated_w_right;

    ros::Subscriber car_sub;
    ros::Subscriber enc_sub;
    ros::Publisher pwm_pub;

    void setDesiredAngularVelocity(float velocity, float angularVelocity) {
        float tempFirstCalculation = velocity / WHEEL_RADIUS;
        float tempSecondCalculation = (BIAS/2) * angularVelocity / WHEEL_RADIUS;

        desired_w_left = tempFirstCalculation - tempSecondCalculation;
        desired_w_right = tempFirstCalculation + tempSecondCalculation;
    }

    void setEstimatedAngularVelocity(int deltaLeft, int deltaRight) {
        float tempCalculation = (M_PI / 180.0) * 10;
        estimated_w_left = deltaLeft * tempCalculation;
        estimated_w_right = deltaRight * tempCalculation;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller");

  MotorControllerNode motor_controller_node;

  ros::Rate loop_rate(10);

  float distanceMeters = 0.0f;
  while (motor_controller_node.n.ok())
  {
    motor_controller_node.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


