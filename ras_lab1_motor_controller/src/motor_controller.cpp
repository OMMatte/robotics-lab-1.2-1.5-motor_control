#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/PWM.h>
#include <math.h>

const float PI =  3.14159265;
const float ALPHA = 1;
const float RELATIVE_PWR_FIXER_MOD = 0.001;
const float BIAS = 0.23;
const float WHEEL_RADIUS = 0.0352;
const int MAX_POWER = 255;
const float EPSILON = 0.000001;

bool INFO = true;


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
        estimated_w_right(0),
        relative_pwr_fixer(1.0)
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

        float pwm1 = relative_pwr_fixer*(old_pwr_left + ALPHA*(desired_w_left - estimated_w_left));
        float pwm2 = (old_pwr_right + ALPHA*(desired_w_right - estimated_w_right));



        {
            float pwm1_abs = std::abs(pwm1);
            float pwm2_abs = std::abs(pwm2);

            float max_pwm = std::max(pwm1_abs, pwm2_abs);

            if(max_pwm > MAX_POWER) {
                // The power is to strong (forward or backwards) for some or both engines. Scale it down and hope it still runs ok
                // Meaning estimated can never reach desired power
                float scale_percent = (float)MAX_POWER / max_pwm;

                pwm1 = pwm1 * scale_percent;
                pwm2 = pwm2 * scale_percent;

                if(estimated_w_left / desired_w_left > estimated_w_right / desired_w_right) {
                    relative_pwr_fixer -= RELATIVE_PWR_FIXER_MOD;
                } else {
                    relative_pwr_fixer += RELATIVE_PWR_FIXER_MOD;
                }
            }
            else {
                relative_pwr_fixer = 1.0;
            }
        }



        old_pwr_left = pwm1;
        old_pwr_right = pwm2;

        pwm.PWM1 = (int)old_pwr_left + 0.5;
        pwm.PWM2 = (int)old_pwr_right + 0.5;

        pwm_pub.publish(pwm);

        if(INFO) {
            ROS_INFO("PWM1: %f", old_pwr_left);
            ROS_INFO("PWM2: %f", old_pwr_right);
            ROS_INFO("Desired1: %f", desired_w_left);
            ROS_INFO("Desired2: %f", desired_w_right);
            ROS_INFO("Estimated1: %f", estimated_w_left);
            ROS_INFO("Estimated2: %f", estimated_w_right);
            ROS_INFO("Relative_pwr_fixer %f", relative_pwr_fixer);
        }
    }

private:
    float relative_pwr_fixer;

    float old_pwr_left;
    float old_pwr_right;

    float desired_w_left;
    float desired_w_right;
    float estimated_w_left;
    float estimated_w_right;

    ros::Subscriber car_sub;
    ros::Subscriber enc_sub;
    ros::Publisher pwm_pub;

    void setDesiredAngularVelocity(float velocity, float angular_velocity) {
        float tempFirstCalculation = velocity / WHEEL_RADIUS;
        float tempSecondCalculation = (BIAS/2) * angular_velocity / WHEEL_RADIUS;

        desired_w_left = tempFirstCalculation - tempSecondCalculation;
        desired_w_right = tempFirstCalculation + tempSecondCalculation;
    }

    void setEstimatedAngularVelocity(int deltaLeft, int deltaRight) {
        float tempCalculation = (M_PI / 180.0) * 10.0;
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


