#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <ros/ros.h>
#include <vector>

#include <ras_lab1_motor_controller/general_functions.h>

class PidController
{
public:
    PidController(float p, float d, float i, bool print_info = false) : alpha_p(p), alpha_d(d), alpha_i(i), i_value_total(0), prev_error(0), print_info(print_info) {}


    float calculateNewMotorPower(float e_w, float d_w) {

        float current_error = d_w - e_w;

        float value_p = getPValue(current_error);

        float value_d = getDValue(current_error);

        float value_i = getIValue();

        updateIValue(current_error);

        prev_error = current_error;

        if(print_info) {
            printInfo("Current Error", current_error);
            printInfo("P_value",value_p);
            printInfo("D_value", value_d);
            printInfo("I_value", value_i);
            printInfo("I_value total", i_value_total);
        }

        return value_p + value_d + value_i;
    }


private:

    bool print_info;
    float prev_error;

    float getPValue(float current_error) {
        return alpha_p * current_error;
    }

    float getDValue(float current_error) {
        return alpha_d * (current_error - prev_error);
    }

    float getIValue() {
        return alpha_i * i_value_total;
    }

    void updateIValue(float current_error) {
        i_value_total += current_error;
    }

    float alpha_p;
    float alpha_d;
    float alpha_i;

    float i_value_total;

};

#endif
