#ifndef INTELLIGENT_H_
#define INTELLIGENT_H_

#include <ros/ros.h>
#include <cmath>
#include "custom_msgs/Comm.h"
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
#include <geometry_msgs/Pose2D.h>

#define kPerson 0
#define kCup 1

#define kHalfFrameWidth 212
#define kHalfFrameHeight 120

class Intelligent {
private:
    void OdomCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void ArmCallback(const std_msgs::Bool::ConstPtr& msg);
    void ObjectCallback(const custom_msgs::Realsense::ConstPtr& msg);
    void HumanTrackCallback(const custom_msgs::Realsense::ConstPtr& msg);
    void CommunicationCallback(const custom_msgs::Comm::ConstPtr& msg);
    
    void BumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
    void CliffCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
    
    ros::NodeHandle node_handle_;
    
    ros::Subscriber sub_realsense_human_;
    ros::Subscriber sub_realsense_object_;
    ros::Subscriber sub_communication_;
    ros::Subscriber sub_odometry_;
    ros::Subscriber sub_bump_;
    ros::Subscriber sub_arm_;
    ros::Subscriber sub_cliff;
    
    ros::Publisher pub_communication_;
    ros::Publisher pub_arm_;

    ros::Publisher pub_kobuki_velocity_;
    ros::Publisher pub_kobuki_power_;

    Object object_[3];

    double move_fb = 0;
    double move_rl = 0;

    double target_fb = 0;
    double target_rl = 0;

    double comm_linear_vel_ = 0;
    double comm_angul_vel_ = 0;

    double kp_ = 0.5;  // P Gain (Sesuaikan nilainya)
    double ki_ = 0.01; // I Gain (Sesuaikan nilainya)
    double kd_ = 0.1;  // D Gain (Sesuaikan nilainya)

    double prev_error_ = 0.0;
    double integral_ = 0.0;
    double target_distance_ = 1.2; // Jarak ideal dalam meter
    
    void FollowHuman();
    void ManualComm();
    void AutomaticSearchCup();
    void ApplyAcceleration();
    double GetFilteredDistance(double);
    
    void PublishMessage();

    double PIDControl(double error);

    int mode_communication_ = 0; //0: home&disconnect, 1: follow human, 2: manual, 3: take cup, 4: direction by hand
    int manual_comm_ = 0; // 0: stop, 1: forward, 2: backward, 3: right, 4: left
    
    bool arm_moving_ = false;
    int yaw_deg_;
    
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
