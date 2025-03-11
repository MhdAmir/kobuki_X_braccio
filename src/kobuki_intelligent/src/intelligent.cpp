#include "kobuki_intelligent/intelligent.h"

Intelligent::Intelligent() {
    sub_realsense_object_ = node_handle_.subscribe("/camera1_postprocess", 1, &Intelligent::ObjectCallback, this);
    sub_realsense_human_ = node_handle_.subscribe("/camera2_postprocess", 1, &Intelligent::HumanTrackCallback, this);
    sub_arm_ = node_handle_.subscribe("/braccio_status", 1, &Intelligent::ArmCallback, this);
    sub_arm_ = node_handle_.subscribe("/mobile_base/events/cliff", 1, &Intelligent::CliffCallback, this);
    sub_bump_ = node_handle_.subscribe("/mobile_base/events/bumper", 1, &Intelligent::BumperCallback, this);
    sub_communication_ = node_handle_.subscribe("/communication_hp", 1, &Intelligent::CommunicationCallback, this);
    
    pub_kobuki_velocity_ = node_handle_.advertise<geometry_msgs::Twist>("/keyop/cmd_vel", 1);
    pub_arm_ = node_handle_.advertise<std_msgs::UInt8MultiArray>("/joint_array",1);
    
    // pub_kobuki_power_ = node_handle_.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
}

void Intelligent::ArmCallback(const std_msgs::Bool::ConstPtr& msg){
    // ROS_INFO("Arm callback triggered. Value: %s", msg->data ? "true" : "false");
    arm_moving_ = !msg->data;
}

void Intelligent::CommunicationCallback(const custom_msgs::Comm::ConstPtr& msg){
    mode_communication_ = msg->mode;
    manual_comm_ = msg->manualComm;
}
void Intelligent::BumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){

}

void Intelligent::CliffCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    
}

void Intelligent::HumanTrackCallback(const custom_msgs::Realsense::ConstPtr& msg){
    object_[kPerson].exist = msg->yolo.size() > 0 &&
                !(msg->yolo[0].x == 0 && msg->yolo[0].y == 0);

    if (object_[kPerson].exist)
    {
        const auto &obj0 = msg->yolo[0];

        object_[kPerson].name = obj0.name.c_str();
        object_[kPerson].frame.x = obj0.x - kHalfFrameWidth;
        object_[kPerson].frame.y = obj0.y - kHalfFrameHeight;
        object_[kPerson].distance = obj0.distance;
        object_[kPerson].x_real = obj0.real_x;
        object_[kPerson].y_real = obj0.real_y;
        object_[kPerson].z_real = obj0.real_z;
    }

    // fprintf(stderr, "person dist >> %g\n", object_[kPerson].distance);
    // fprintf(stderr, "person exist >> %d\n\n", object_[kPerson].exist);
}

void Intelligent::ObjectCallback(const custom_msgs::Realsense::ConstPtr& msg){
    object_[kCup].exist = msg->yolo.size() > 0 &&
                !(msg->yolo[0].x == 0 && msg->yolo[0].y == 0);

    if (object_[kCup].exist)
    {
        const auto &obj0 = msg->yolo[0];
        
        object_[kCup].name = obj0.name.c_str();
        object_[kCup].frame.x = obj0.x - kHalfFrameWidth;
        object_[kCup].frame.y = obj0.y - kHalfFrameHeight;
        object_[kCup].distance = obj0.distance;
        object_[kCup].x_real = obj0.real_x;
        object_[kCup].y_real = obj0.real_y;
        object_[kCup].z_real = obj0.real_z;
    }

    fprintf(stderr, "cup dist >> %g\n", object_[kCup].distance);
    fprintf(stderr, "cup exist >> %d\n\n", object_[kCup].exist);
}
