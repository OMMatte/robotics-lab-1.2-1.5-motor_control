#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/PWM.h>
#include <math.h>
#include <vector>

#include <ras_lab1_motor_controller/pid_controller.h>
#include <ras_lab1_motor_controller/general_functions.h>

const float PI =  3.14159265;

const float ALPHA_A = 1;
const float ALPHA_D = 0.05;
const float ALPHA_I = 0.005;

const float BIAS = 0.23;
const float WHEEL_RADIUS = 0.0352;

const int MAX_INPUT_TIME_MS = 2000; //2000 = 2 seconds
const int MAX_POWER = 255;

bool INFO = true;


class MotorControllerNode
{
public:

    ros::NodeHandle n;

    MotorControllerNode() :
        current_pwr_left(0),
        current_pwr_right(0),
        desired_w_left(0),
        desired_w_right(0),
        acc_mod_left(0),
        acc_mod_right(0)
    {
        n = ros::NodeHandle();

        last_input_time = ros::Time::now();
        start_time = ros::Time::now();

        pid_controller_left.reset(new PidController(ALPHA_A, ALPHA_D, ALPHA_I, INFO));
        pid_controller_right.reset(new PidController(ALPHA_A, ALPHA_D, ALPHA_I));

        car_sub = n.subscribe("motor_controller/twist", 1, &MotorControllerNode::cartesianCallback, this);
        enc_sub = n.subscribe("kobuki/encoders", 1, &MotorControllerNode::encoderCallback, this);
        pwm_pub = n.advertise<ras_arduino_msgs::PWM>("kobuki/pwm", 1);
    }

    void cartesianCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        last_input_time = ros::Time::now();
        setDesiredAngularVelocity(msg->linear.x, msg->angular.z);
    }


    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
    {
        std::vector<float> estimated = getEstimatedAngularVelocity(msg->delta_encoder1, msg->delta_encoder2);

        float pwm_mod_left = pid_controller_left->calculateNewMotorPower(estimated[0], desired_w_left);
        float pwm_mod_right = pid_controller_right->calculateNewMotorPower(estimated[1], desired_w_right);

        current_pwr_left = current_pwr_left + pwm_mod_left;
        current_pwr_right = current_pwr_right + pwm_mod_right;


        acc_mod_left += std::abs(pwm_mod_left);
        acc_mod_right += std::abs(pwm_mod_right);

        if(INFO) {
            float timePassed = (ros::Time::now() - start_time).toSec();
            printInfo("PWM", current_pwr_left, current_pwr_right);
            printInfo("Desired", desired_w_left, desired_w_right);
            printInfo("Estimated", estimated[0], estimated[1]);
            printInfo("Pwm Mod", pwm_mod_left, pwm_mod_right);
            printInfo("Acc mod", acc_mod_left, acc_mod_right);
            printInfo("Time", timePassed);
            printInfo("Avg error/s", acc_mod_left/timePassed, acc_mod_right/timePassed);
        }

    }

    void update()
    {
        if(MAX_INPUT_TIME_MS > 0 && (ros::Time::now() - last_input_time).toSec() > (MAX_INPUT_TIME_MS / 1000.0)) {
            return;
        }

        ras_arduino_msgs::PWM pwm;

        pwm.PWM1 = (int)current_pwr_left + 0.5;
        pwm.PWM2 = (int)current_pwr_right + 0.5;


        pwm_pub.publish(pwm);
    }

private:
    std::unique_ptr<PidController> pid_controller_left;
    std::unique_ptr<PidController> pid_controller_right;

    float acc_mod_left;
    float acc_mod_right;

    float current_pwr_left;
    float current_pwr_right;

    float desired_w_left;
    float desired_w_right;

    ros::Time last_input_time;
    ros::Time start_time;

    ros::Subscriber car_sub;
    ros::Subscriber enc_sub;
    ros::Publisher pwm_pub;

    void setDesiredAngularVelocity(float velocity, float angular_velocity) {
        float tempFirstCalculation = velocity / WHEEL_RADIUS;
        float tempSecondCalculation = (BIAS/2) * angular_velocity / WHEEL_RADIUS;

        desired_w_left = tempFirstCalculation - tempSecondCalculation;
        desired_w_right = tempFirstCalculation + tempSecondCalculation;
    }

    std::vector<float> getEstimatedAngularVelocity(int deltaLeft, int deltaRight) {
        float tempCalculation = (M_PI / 180.0) * 10.0;
        std::vector<float> estimated_vector = {deltaLeft * tempCalculation, deltaRight*tempCalculation};
        return estimated_vector;
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


