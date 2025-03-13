    #ifndef DETECTOR_ROSPUB_H_
    #define DETECTOR_ROSPUB_H_

    #include "frame_process.h"
    #include "utility/model_config.h"
    #include "ros/ros.h"

    #include "custom_msgs/Object.h"
    #include "custom_msgs/Realsense.h"

    

    class ROSPublish
    {
    public:
        ROSPublish(FrameProcessor &frame_processor, ModelConfig &model_config, std::string camera_name, ros::NodeHandle &nh);

        void PublishMessage();

    private:
        FrameProcessor &frame_processor_;
        ModelConfig &model_config_;
        ros::Publisher realsense_pub_;
        std::mutex publish_mutex;
    };

    #endif // DETECTOR_ROSPUB_H_
