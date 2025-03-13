#include "object_detection/yolo/object_detection.h"

YOLO::YOLO(ModelConfig &model_config)
    : model_config_(model_config), running_(false), yolo_enabled_(false)
{
  inference_ = new Inference(model_config);
}

YOLO::~YOLO()
{
  Stop();
  delete inference_;
}

void YOLO::Start(RealSenseCamera &camera)
{
  if (running_)
    return; 
  running_ = true;
  yolo_thread_ = std::thread(&YOLO::Run, this, std::ref(camera));
}

void YOLO::Stop()
{
  if (!running_)
    return;              
  running_ = false;      
  cv_yolo_.notify_all(); 
  if (yolo_thread_.joinable())
  {
    yolo_thread_.join(); 
  }
}

void YOLO::EnableYOLO(bool enable)
{
  {
    std::lock_guard<std::mutex> lock(mutex_yolo_);
    yolo_enabled_ = enable;
  }
  cv_yolo_.notify_all(); 
}

void YOLO::Run(RealSenseCamera &camera) {
  while (running_) {
      std::unique_lock<std::mutex> lock(mutex_yolo_);
      cv_yolo_.wait(lock, [this]() { return yolo_enabled_ || !running_; });

      if (!running_) break; // Keluar jika thread diminta berhenti

      if (!yolo_enabled_) continue; // Lewati inferensi jika YOLO dinonaktifkan

      if (!camera.Device_connected_.load())
          continue;

      cv::Mat color_frame = camera.GetColorFrame();
      if (color_frame.empty()) {
          continue;
      }

      detected_object_ = Predict(color_frame);
  }
}

std::vector<DetectedObject> YOLO::GetDetectedObject()
{
  std::lock_guard<std::mutex> lock(mutex_object);
  return detected_object_;
}

cv::Point YOLO::GetCentroid(const cv::Rect &rect)
{
  cv::Point centroid;

  centroid.x = rect.x + (rect.width / 2);
  centroid.y = rect.y + (rect.height / 2);

  return centroid;
}

cv::Point YOLO::GetBottomCentroid(const cv::Rect &rect)
{
  cv::Point bottom_centroid;

  bottom_centroid.x = rect.x + (rect.width / 2);
  bottom_centroid.y = rect.y + (rect.height);

  return bottom_centroid;
}

bool YOLO::SortByConfidence(const Detection &right, const Detection &left)
{
  return right.confidence > left.confidence;
}

std::vector<DetectedObject> YOLO::Predict(const cv::Mat &frame)
{
  // fprintf(stderr, "predict\n");

  const float kConfidenceThreshold = 0.05;

  std::vector<Detection> detection = inference_->RunInference(frame);
  std::vector<DetectedObject> detected_object;

  for (uint16_t i = 0; i < detection.size(); ++i)
  {
    if (detection[i].confidence < kConfidenceThreshold)
      continue;

    DetectedObject detected;

    detected.class_id_ = detection[i].class_id;
    detected.bottom_centroid_ = GetBottomCentroid(detection[i].box);
    detected.centroid_ = GetCentroid(detection[i].box);
    detected.rectangle_ = detection[i].box;
    detected.confidence_ = detection[i].confidence;

    detected_object.push_back(detected);
  }

  // fprintf(stderr, "success predict\n");


  return detected_object;
}
