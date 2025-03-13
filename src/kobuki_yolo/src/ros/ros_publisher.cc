#include "ros/ros_publisher.h"

ROSPublish::ROSPublish(FrameProcessor &frame_processor, ModelConfig &model_config, std::string camera_name,ros::NodeHandle &nh) : frame_processor_(frame_processor), model_config_(model_config)
{
    realsense_pub_ = nh.advertise<custom_msgs::Realsense>("/"+camera_name+"_postprocess", 0);
}

void ROSPublish::PublishMessage()
{
    // fprintf(stderr, "pub\n");
    std::lock_guard<std::mutex> lock(publish_mutex);

    std::vector<std::string> name_object = model_config_.GetObjectName();
    
    std::vector<DetectedObject> detected_object;
    detected_object.clear();
    detected_object = frame_processor_.DrawFrame();

    if (name_object.empty()) {
        ROS_ERROR("ModelConfig object names are empty!");
        return;
    }

    custom_msgs::Realsense realsense_message;

    if (!detected_object.empty()) {
        const DetectedObject &detection = detected_object[0];

        if (detection.class_id_ >= 0 && detection.class_id_ < name_object.size()) {
            custom_msgs::Object object;
            object.x = detection.centroid_.x;
            object.y = detection.centroid_.y;
            object.distance = detection.distance_;
            object.real_x = detection.xyz_values_.x;
            object.real_y = detection.xyz_values_.y;
            object.real_z = detection.xyz_values_.z;
            object.name = name_object[detection.class_id_];

            realsense_message.yolo.push_back(object);
        } else {
            ROS_WARN("Invalid class_id_: %d, skipping object.", detection.class_id_);
        }
    } else {

        custom_msgs::Object object;
        object.x = 0;
        object.y = 0;
        object.distance = 0;
        object.name = "none";
        realsense_message.yolo.push_back(object);
    }

    
    realsense_pub_.publish(realsense_message);
    // fprintf(stderr, "pub success\n");
}
