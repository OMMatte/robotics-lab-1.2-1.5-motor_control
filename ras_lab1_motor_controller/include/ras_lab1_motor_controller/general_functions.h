#ifndef GENERAL_FUNCTIONS_H_
#define GENERAL_FUNCTIONS_H_

#include <ros/ros.h>

void printInfo(std::string text, float value) {
    ROS_INFO((text + ": %f").c_str(), value);
}

void printInfo(std::string text, float value1, float value2) {
    ROS_INFO((text + ": %f | %f").c_str(), value1, value2);
}

#endif
