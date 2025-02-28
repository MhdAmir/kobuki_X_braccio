#include "ros/ros_publisher.h"

ROSPublish::ROSPublish(FrameProcessor &frame_processor, ModelConfig &model_config, ros::NodeHandle &nh) : frame_processor_(frame_processor), model_config_(model_config)
{
    realsense_pub_ = nh.advertise<msgs::Realsense>("/realsense", 0);
}

void ROSPublish::PublishMessage()
{
    std::vector<std::string> name_object = model_config_.GetObjectName();
    std::vector<DetectedObject> detected_object = frame_processor_.PubDetectedObject();

    msgs::Realsense realsense_message;
    std::map<std::string, bool> object_detected;
    for (const auto &name : name_object)
    {
        object_detected[name] = false;
    }

    for (const auto &detection : detected_object)
    {
        msgs::Object object;
        object.x = detection.centroid_.x;
        object.y = detection.centroid_.y;
        object.distance = detection.distance_;
        object.name = name_object[detection.class_id_];
        realsense_message.yolo.push_back(object);

        object_detected[object.name] = true;
    }

    for (const auto &name : name_object)
    {
        if (!object_detected[name])
        {
            msgs::Object object;
            object.x = 0;
            object.y = 0;
            object.distance = 0;
            object.name = name;
            realsense_message.yolo.push_back(object);
        }
    }

    realsense_pub_.publish(realsense_message);
}