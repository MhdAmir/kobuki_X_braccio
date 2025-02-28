#include <ros/ros.h>
#include "camera/realsense.h"
#include "frame_process.h"
// #include "ros/ros_publisher.h"
#include "object_detection/yolo/object_detection.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_vision_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    ModelConfig model_config("/home/eros/parameters/realsense/learning/model_config.yaml");
    RealSenseCamera camera;
    YOLO yolo(model_config);
    FrameProcessor frame_processor(camera, yolo, model_config);

    frame_processor.Start();

    // ROSPublish ros_publish(frame_processor, model_config, nh);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        // ros_publish.PublishMessage();
    }
}
