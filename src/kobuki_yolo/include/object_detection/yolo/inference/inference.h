#ifndef YOLOV8_INFERENCE_H_
#define YOLOV8_INFERENCE_H_

#include <iostream>
#include <string>
#include <vector>

#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include "utility/model_config.h"

struct Detection {
	int class_id;
	float confidence;
	cv::Rect box;
};

class Inference {
public:
	Inference(ModelConfig& model_config);
	std::vector<Detection> RunInference(const cv::Mat &frame);

private:
	void InitialModel(const std::string &model_path, const int &int8_size);
	void Preprocessing(const cv::Mat &frame);
	void PostProcessing();
	cv::Rect GetBoundingBox(const cv::Rect &src);

	ov::CompiledModel compiled_model_;
	ov::InferRequest inference_request_;
	ov::Tensor input_tensor_;

	cv::Mat resized_frame_;
	cv::Size2f model_input_shape_;
	cv::Size model_output_shape_;
	cv::Point2f factor_;

	std::vector<float> model_score_threshold_;
	std::vector<float> model_NMS_threshold_;
	std::vector<Detection> detections_;
	std::vector<int> target_class_ids_; // Added to store target class IDs
};

#endif // YOLOV8_INFERENCE_H_
