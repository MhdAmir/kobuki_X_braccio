#include <ros/ros.h>
#include "camera/realsense.h"
#include "frame_process.h"
#include "ros/ros_publisher.h"
#include "object_detection/yolo/object_detection.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_vision_node");
    ros::NodeHandle nh("~");  // Namespace untuk parameter

    std::string model_config_path, camera_name;
    if (!nh.getParam("model_config", model_config_path))
    {
        ROS_ERROR("Parameter 'model_config' not set, using default.");
        model_config_path = "/home/eros/parameters/realsense/learning/model_config1.yaml";
    }

    if (!nh.getParam("camera_name", camera_name))
    {
        ROS_ERROR("Parameter 'camera_name' not set, using default.");
        camera_name = "camera";
    }

    ModelConfig model_config(model_config_path);
    RealSenseCamera camera(camera_name);
    YOLO yolo(model_config);
    FrameProcessor frame_processor(camera, yolo, model_config);

    frame_processor.Start();
    ROSPublish ros_publish(frame_processor, model_config, camera_name, nh);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        ros_publish.PublishMessage();
    }
}
