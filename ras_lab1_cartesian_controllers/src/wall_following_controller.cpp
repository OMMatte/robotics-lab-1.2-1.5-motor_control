#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <geometry_msgs/Twist.h>

const float PI =  3.14159265;

const float LINEAR_VELOCITY = 0.3;
const float ALPHA = 0.005;
const float DISTANCE_TO_WALL = 5.0;

const int HZ = 10;

class WallFollowingController {
public:
    ros::NodeHandle n;

    WallFollowingController() :
        frontSensor(0),
        backSensor(0)
    {
        adc_sub = n.subscribe("kobuki/adc", 1, &WallFollowingController::adcCallback, this);
        mot_pub = n.advertise<geometry_msgs::Twist>("motor_controller/twist", 1);
    }

    void update(){
        geometry_msgs::Twist mot_msg;

        mot_msg.linear.x = LINEAR_VELOCITY;
        mot_msg.angular.z = ALPHA*(backSensor - frontSensor);

        mot_pub.publish(mot_msg);
    }

private:
    ros::Subscriber adc_sub;
    ros::Publisher mot_pub;

    int frontSensor;
    int backSensor;

    void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg) {
        frontSensor = msg->ch1;
        backSensor = msg->ch2;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_following_controller");
    WallFollowingController wallFollowingController;

    ros::Rate loop_rate(HZ);

    while (wallFollowingController.n.ok()){
        wallFollowingController.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
  return 0;
}
