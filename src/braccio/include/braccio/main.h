#ifndef MAIN_H
#define MAIN_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <cmath>
// #include "custom_msgs/getTarget.h"
// #include "braccio/PID.h"
#include "custom_msgs/Realsense.h"
#include "custom_msgs/Object.h"

struct Head
{
public:
    void Init() { degree = init_degree; }

    void Set(const float degreeValue) { degree = degreeValue; }

    bool IsLocked() const { return lock; }
    void Unlock() { lock = false; }
    void Lock()
    {
        if (!lock)
        {
            lock = true;
            degree = lock_degree;
        }
    }

    // PID pid;

    int degree;
    float mid_degree;
    float min_degree;
    float max_degree;
    float degree_offset;

    float shifter;

    float init_degree;
    float lock_degree;
    float mid_degree_limit;
    float min_degree_limit;
    float max_degree_limit;

    unsigned char range;
    unsigned char state;

private:
    bool lock;
};

struct FlagList
{
    bool tracking_object;
};

struct Point
{
    Point(const short x_ = 0, const short y_ = 0) : x(x_), y(y_) {}
    short x;
    short y;
};

class Object
{
public:
    Point error;
    Point frame;
    std::string name;
    float distance;
};

class InverseKinematics
{
public:
    static std::vector<int> computeAngles(double x, double y, double z);
};

class ArduinoCommunicator
{
private:
    serial::Serial serialPort;
public:
    ArduinoCommunicator(const std::string &port, int baudrate);
    bool sendCommand(const std::string &command, double delay); // Changed return type to bool
    bool sendCommand(const std::string &command); // Changed return type to bool
    std::string readResponse(int timeout_ms);
    bool isIdle();
};

class GetObject
{
private:
    Object object_[2];
    FlagList flag_;
    Head arm_pan_, arm_tilt_;
    ros::NodeHandle nh_;
    ArduinoCommunicator arduino;
    
    // bool get_target_position_(custom_msgs::getTarget::Request &req, custom_msgs::getTarget::Response &res);
    void realsenseCallback(const custom_msgs::Realsense::ConstPtr& msg);
    void initTrackingParameters();
    void updateTracking(); // Add this line
    // bool init_position_(msgs::setInit::Request & req, msgs::setInit::Response & res);
    
    ros::ServiceServer get_target_position_srv_, init_position_srv_;
    ros::Subscriber sub_;
    
public:
    void TrackObject(int i);
    GetObject(int argc, char **argv);
    void run();
};

#endif // MAIN_H
