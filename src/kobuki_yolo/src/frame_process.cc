#include "frame_process.h"

FrameProcessor::FrameProcessor(RealSenseCamera& camera, YOLO& yolo, ModelConfig& model_config) : camera_(camera), yolo_(yolo), model_config_(model_config) {    
    use_gui_ = true;
    running_ = true;
    config_object_name_ = model_config_.GetObjectName();
}
FrameProcessor::~FrameProcessor() {
    running_ = false;
    camera_.Stop();
    if(use_gui_)
        cv::destroyAllWindows();

    const uint8_t kTotalThread = 3;
    for (uint8_t i = 0; i < kTotalThread; ++i) {
      if (thread_[i].joinable())
        thread_[i].join();
    }

    fprintf(stderr, "[realsense] successfully stopped. See ya!\n");
}

void FrameProcessor::Start() {
    thread_[kCapture] = std::thread(&RealSenseCamera::Start, &camera_);
    thread_[kObjectDetection] = std::thread(&YOLO::Start, &yolo_, std::ref(camera_));
    thread_[kColorProcess] = std::thread(&FrameProcessor::DrawFrame, this);
}


void FrameProcessor::DrawFrame() {
    while (running_ )
    {
        if(camera_.Device_connected_.load() == false){
            cv::destroyAllWindows();
            continue;
        }
        color_frame_ = camera_.GetColorFrame();
        depth_frame_ = camera_.GetDepthFrame();

        detected_object_ = yolo_.GetDetectedObject();
        EstimateDepth(detected_object_);
        EstimateRealXYZ(detected_object_);

        if (color_frame_.empty() || depth_frame_.empty()) {
            continue;
        }
        if(use_gui_){
            for(auto& object : detected_object_){
                cv::rectangle(color_frame_, object.rectangle_, cv::Scalar(0, 255, 0), 2);
                cv::putText(color_frame_, std::to_string(object.confidence_), object.centroid_, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

            }
            cv::imshow("Color Frame", color_frame_);
            cv::imshow("Depth Frame", depth_frame_);    
            cv::waitKey(1);
        }
    }
}

void FrameProcessor::EstimateDepth(std::vector<DetectedObject> &detected_object) {
    for(auto& object : detected_object){
        object.distance_ = camera_.GetDepthValue(object.centroid_, 6);
    }
}

void FrameProcessor::EstimateRealXYZ(std::vector<DetectedObject> &detected_object){
    for(auto& object : detected_object){
        object.xyz_values_ = camera_.GetXYZDepthValues(object.centroid_, 6);
    }
}

std::vector<DetectedObject> FrameProcessor::PubDetectedObject(){
    return detected_object_;
}
