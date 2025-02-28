    #ifndef DETECTOR_ROSPUB_H_
    #define DETECTOR_ROSPUB_H_

    #include "frame_process.h"
    #include "utility/model_config.h"
    #include "ros/ros.h"
    #include <msgs/Object.h>
    #include <msgs/Realsense.h>

    class ROSPublish
    {
    public:
        ROSPublish(FrameProcessor &frame_processor, ModelConfig &model_config, ros::NodeHandle &nh);

        void PublishMessage();

    private:
        FrameProcessor &frame_processor_;
        ModelConfig &model_config_;
        ros::Publisher realsense_pub_;
    };

    #endif // DETECTOR_ROSPUB_H_
