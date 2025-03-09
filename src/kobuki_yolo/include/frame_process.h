#ifndef FRAME_PROCESS_H
#define FRAME_PROCESS_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "camera/realsense.h"
#include "object_detection/yolo/object_detection.h"
#include "utility/model_config.h"
#include <thread>

enum VisionThread : uint8_t {
	kCapture = 0,
	kObjectDetection = 1,
	kColorProcess = 2,
};

class FrameProcessor {
public:
    FrameProcessor(RealSenseCamera& camera, YOLO& yolo, ModelConfig& model_config);
    ~FrameProcessor();

    void Start();
    void DrawFrame();
    std::vector<DetectedObject> PubDetectedObject();

private:
    RealSenseCamera& camera_;
    YOLO& yolo_;
    std::thread thread_[3];
    ModelConfig& model_config_;

    std::vector<std::string> config_object_name_;
    std::vector<DetectedObject> detected_object_;


    void EstimateDepth(std::vector<DetectedObject> &detected_object);
    void EstimateRealXYZ(std::vector<DetectedObject> &detected_object);

    cv::Mat color_frame_;
    cv::Mat depth_frame_;
    bool use_gui_ = true;
    bool running_ = true;
};

#endif // FRAME_PROCESS_H
