#ifndef INTELLIGENT_H_
#define INTELLIGENT_H_

#include <ros/ros.h>
#include <cmath>
#include "custom_msgs/Realsense.h"
#include "custom_msgs/Object.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/MotorPower.h"
#include "kobuki_msgs/BumperEvent.h"
#include "kobuki_intelligent/object.h"
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>
#include <chrono>
#include <deque>

#define kPerson 0
#define kCup 1

#define kHalfFrameWidth 212
#define kHalfFrameHeight 120

class Intelligent {
private:
    void RealsenseCallback(const custom_msgs::Realsense::ConstPtr& msg);
    void ArmCallback(const std_msgs::Bool::ConstPtr& msg);
    void ObjectCallback(const custom_msgs::Object::ConstPtr& msg);
    void BumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
    
    ros::NodeHandle node_handle_;
    
    ros::Subscriber sub_realsense_human_;
    ros::Subscriber sub_realsense_object_;
    ros::Subscriber sub_communication_;
    ros::Subscriber sub_odometry_;
    ros::Subscriber sub_bump_;
    ros::Subscriber sub_arm_;
    
    ros::Publisher pub_communication_;
    ros::Publisher pub_arm_;

    ros::Publisher pub_kobuki_velocity_;
    ros::Publisher pub_kobuki_power_;

    Object object_[3];

    double move_fb = 0;
    double move_rl = 0;

    double target_fb = 0;
    double target_rl = 0;

    
    void AutomaticSearchCup();
    void ApplyAcceleration();
    double GetFilteredDistance(double);
    
    void PublishMessage();
    
    bool arm_moving_ = false;
    
    int state_ = 0;

    const int HISTORY_SIZE = 15;
    std::vector<double> distance_history_;

    std::vector<uint8_t> angles_ = {90, 90, 90, 90, 90, 73};
    std::vector<uint8_t> last_angles_ = {90, 90, 90, 90, 90, 73};
    
public:
    Intelligent();
    void Loop();
};

#endif // INTELLIGENT_H_
