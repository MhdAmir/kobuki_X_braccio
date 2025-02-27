#include "kobuki_yolo/subscribe_frame.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_subscriber");
    ros::NodeHandle nh;
    
    RealsenseSubscriber subscriber(nh);
    
    ros::spin();
    
    return 0;
}