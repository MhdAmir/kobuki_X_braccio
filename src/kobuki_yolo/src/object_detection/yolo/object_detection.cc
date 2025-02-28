#include "object_detection/yolo/object_detection.h"

YOLO::YOLO(ModelConfig &model_config) : model_config_(model_config)
{
  inference_ = new Inference(model_config);
}

YOLO::~YOLO()
{
  if (running_)
    running_ = false;

  delete inference_;
}

void YOLO::Start(RealSenseCamera &camera)
{
  running_ = true;
  while (running_)
  {
    if (!camera.Device_connected_.load())
      continue;

    color_frame_ = camera.GetColorFrame();

    if (color_frame_.empty())
    {
      continue;
    }

    detected_object_ = Predict(color_frame_);
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

  return detected_object;
}
