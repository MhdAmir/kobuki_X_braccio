#ifndef MAIN_H
#define MAIN_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <cmath>
#include "brac_msgs/getTarget.h"


class InverseKinematics {
public:
    static std::vector<int> computeAngles(double x, double y, double z);
};

class ArduinoCommunicator {
private:
    serial::Serial serialPort;
public: 
    ArduinoCommunicator(const std::string &port, int baudrate);
    void sendCommand(const std::string &command, double delay);
};

class GetObject {
private:
    ros::NodeHandle nh_;
    ArduinoCommunicator arduino;

    bool get_target_position_(brac_msgs::getTarget::Request & req, brac_msgs::getTarget::Response & res);
    // bool init_position_(msgs::setInit::Request & req, msgs::setInit::Response & res);

    ros::ServiceServer get_target_position_srv_, init_position_srv_;
public:
    GetObject(int argc, char **argv);
    void run();
};

#endif // MAIN_H
