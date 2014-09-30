#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/PWM.h>
#include <math.h>
#include <vector>

const float PI =  3.14159265;
const float ALPHA = 1;
const float ALPHA_D = 0.01;
const float ALPHA_I = 0.001;
const float RELATIVE_PWR_FIXER_MOD = 0.001;
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
        estimated_w_left(0),
        estimated_w_right(0),
        relative_pwr_fixer(1.0),
        error_prev_left(0),
        error_prev_right(0),
        total_I_value_left(0),
        total_I_value_right(0)
    {
        n = ros::NodeHandle();
        last_error_check_time = ros::Time::now();

        car_sub = n.subscribe("motor_controller/twist", 1, &MotorControllerNode::cartesianCallback, this);
        enc_sub = n.subscribe("kobuki/encoders", 1, &MotorControllerNode::encoderCallback, this);
        pwm_pub = n.advertise<ras_arduino_msgs::PWM>("kobuki/pwm", 1);

        /*
        error_vector(1000, 0.0f);
        error_total = 0;
        error_counter = 0;
        */

    }

    void cartesianCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        last_input_time = ros::Time::now();
        setDesiredAngularVelocity(msg->linear.x, msg->angular.z);
    }


    void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
    {
        setEstimatedAngularVelocity(msg->delta_encoder1, msg->delta_encoder2);

        float error_now_left = desired_w_left - estimated_w_left;
        float error_now_right = desired_w_right - estimated_w_right;

        /*
        error_total -= error_vector[error_counter] + std::abs(error_now_left);
        error_vector[error_counter] = std::abs(error_now_left);
        error_counter = (error_counter + 1) % 1000;
        */

        float P_fix_left = ALPHA*(error_now_left);
        float P_fix_right = ALPHA*(error_now_right);

        float error_time_diff = (ros::Time::now() - last_error_check_time).toSec();

        float D_fix_left = ALPHA_D*(error_now_left - error_prev_left) / error_time_diff;
        float D_fix_right = ALPHA_D*(error_now_right - error_prev_right) / error_time_diff;

        error_prev_left = error_now_left;
        error_prev_right = error_now_right;

        last_error_check_time = ros::Time::now();

        float I_fix_left = total_I_value_left * ALPHA_I;
        float I_fix_right = total_I_value_right * ALPHA_I;


        float pwm1 = relative_pwr_fixer*(current_pwr_left + P_fix_left + D_fix_left + I_fix_left);
        float pwm2 = (current_pwr_right + P_fix_right + D_fix_right + I_fix_right);

        total_I_value_left += error_now_left / error_time_diff;
        total_I_value_right += error_now_right / error_time_diff;

        //calculations for "to fast"
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

        current_pwr_left = pwm1;
        current_pwr_right = pwm2;


        if(INFO) {
            ROS_INFO("PWM: %f | %f", current_pwr_left, current_pwr_right);
            ROS_INFO("Desired: %f | %f", desired_w_left, desired_w_right);
            ROS_INFO("Estimated: %f | %f", estimated_w_left, estimated_w_right);
            ROS_INFO("Error_now: %f | %f", error_now_left, error_now_right);
            ROS_INFO("P_fix: %f | %f", P_fix_left, P_fix_right);
            ROS_INFO("D_fix: %f | %f", D_fix_left, D_fix_right);
            ROS_INFO("I_fix: %f | %f", I_fix_left, I_fix_right);
            ROS_INFO("Relative_pwr_fixer: %f", relative_pwr_fixer);
            ROS_INFO("");
            //ROS_INFO("Error_avarage: %f", error_total / 1000.0);
        }
    }

    void update()
    {
        //if(MAX_INPUT_TIME_MS > 0 && (ros::Time::now() - last_input_time).toSec() > (MAX_INPUT_TIME_MS / 1000.0)) {
       //     return;
        //}

        ras_arduino_msgs::PWM pwm;

        pwm.PWM1 = (int)current_pwr_left + 0.5;
        pwm.PWM2 = (int)current_pwr_right + 0.5;


        pwm_pub.publish(pwm);


    }

private:
    float total_I_value_left;
    float total_I_value_right;

    float relative_pwr_fixer;

    float current_pwr_left;
    float current_pwr_right;

    float desired_w_left;
    float desired_w_right;
    float estimated_w_left;
    float estimated_w_right;

    float error_prev_left;
    float error_prev_right;

    ros::Time last_input_time;
    ros::Time last_error_check_time;

    ros::Subscriber car_sub;
    ros::Subscriber enc_sub;
    ros::Publisher pwm_pub;

    /*
    float error_total;
    std::vector<float> error_vector;
    int error_counter;
    */

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


