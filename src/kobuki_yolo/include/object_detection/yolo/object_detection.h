#ifndef DETECTOR_OBJECT_DETECTION_H_
#define DETECTOR_OBJECT_DETECTION_H_

#include "object_detection/yolo/inference/inference.h"
#include "camera/realsense.h"
#include "utility/model_config.h"


#include <cstdint>
#include <vector>
#include <algorithm>
#include <mutex>
#include <condition_variable>

#include <opencv2/core/types.hpp>

class DetectedObject {
 public:
  cv::Point centroid_;
  cv::Point bottom_centroid_;
  cv::Rect rectangle_;
  float distance_;
  uint8_t class_id_;
  rs2_vector xyz_values_;
  float confidence_;
};

class YOLO {
 public:
  YOLO(ModelConfig& model_config);
  ~YOLO();

  void Start(RealSenseCamera& camera);
  void Stop();

  void EnableYOLO(bool enable);

  std::vector<DetectedObject> GetDetectedObject();
  std::vector<DetectedObject> Predict(const cv::Mat &frame);
  bool SortByConfidence(const Detection &right, const Detection &left);

 private:
  ModelConfig model_config_;
  void Run(RealSenseCamera &camera);
  cv::Point GetCentroid(const cv::Rect &rect);
  cv::Point GetBottomCentroid(const cv::Rect &rect);
  Inference *inference_;
  cv::Mat color_frame_;
  cv::Mat depth_frame_;

  std::vector<DetectedObject> detected_object_;

  std::mutex mutex_object;
  std::atomic<bool> running_;       
  std::atomic<bool> yolo_enabled_;  
  std::thread yolo_thread_;          
  std::mutex mutex_yolo_;           
  std::condition_variable cv_yolo_;  
};

#endif // DETECTOR_OBJECT_DETECTION_H_