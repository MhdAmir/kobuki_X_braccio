#include "object_detection/yolo/inference/inference.h"

#include <memory>

Inference::Inference(ModelConfig& model_config) {
    model_score_threshold_ = model_config.GetScoreThreshold();
    model_NMS_threshold_ = model_config.GetScoreThreshold(); 
    
    std::string model_path = model_config.GetModelPath();
    int int8_size = model_config.GetInputSize();

    std::cerr << model_path << std::endl;

    InitialModel(model_path, int8_size);
    
    // Store the target class IDs (0 and 41)
    target_class_ids_ = {0, 41};
}

void Inference::InitialModel(const std::string &model_path, const int &int8_size) {
    ov::Core core;   
    std::shared_ptr<ov::Model> model = core.read_model(model_path);
    ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);

    if (model->is_dynamic()) {
        model->reshape({1, 3, static_cast<long int>(int8_size), static_cast<long int>(int8_size)});
    }

    ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
    ppp.input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale({ 255, 255, 255 });
    ppp.input().model().set_layout("NCHW");
    ppp.output().tensor().set_element_type(ov::element::f32);

    model = ppp.build();
    compiled_model_ = core.compile_model(model, "CPU");
    inference_request_ = compiled_model_.create_infer_request();

    const std::vector<ov::Output<ov::Node>> inputs = model->inputs();
    const ov::Shape input_shape = inputs[0].get_shape();

    short height = input_shape[1];
    short width = input_shape[2];
    model_input_shape_ = cv::Size2f(width, height);

    const std::vector<ov::Output<ov::Node>> outputs = model->outputs();
    const ov::Shape output_shape = outputs[0].get_shape();

    height = output_shape[1];
    width = output_shape[2];
    model_output_shape_ = cv::Size(width, height);
}

std::vector<Detection> Inference::RunInference(const cv::Mat &frame) {
    Preprocessing(frame);
    inference_request_.infer();
    PostProcessing();

    return detections_;
}

void Inference::Preprocessing(const cv::Mat &frame) {
    // Use a fixed size output to avoid memory reallocations
    cv::resize(frame, resized_frame_, model_input_shape_, 0, 0, cv::INTER_AREA);

    factor_.x = static_cast<float>(frame.cols / model_input_shape_.width);
    factor_.y = static_cast<float>(frame.rows / model_input_shape_.height);

    float *input_data = (float *)resized_frame_.data;
    input_tensor_ = ov::Tensor(compiled_model_.input().get_element_type(), compiled_model_.input().get_shape(), input_data);
    inference_request_.set_input_tensor(input_tensor_);
}

void Inference::PostProcessing() {
    std::vector<int> class_list;
    std::vector<float> confidence_list;
    std::vector<cv::Rect> box_list;

    float *detections = inference_request_.get_output_tensor().data<float>();
    const cv::Mat detection_outputs(model_output_shape_, CV_32F, (float *)detections);

    if(detection_outputs.rows - 4 != model_score_threshold_.size()) {
        std::cerr << "Amount of parameter model score threshold is not correct" << std::endl;
        std::cerr << "Check it on netron.app / metadata.yaml" << std::endl;
    }

    // Pre-allocate memory for faster processing
    class_list.reserve(detection_outputs.cols);
    confidence_list.reserve(detection_outputs.cols);
    box_list.reserve(detection_outputs.cols);

    // Process only target classes (0 and 41)
    for (int i = 0; i < detection_outputs.cols; ++i) {
        const cv::Mat classes_scores = detection_outputs.col(i).rowRange(4, detection_outputs.rows);
        
        // Only check scores for classes 0 and 41 instead of finding max across all classes
        for (int target_class : target_class_ids_) {
            float score = classes_scores.at<float>(target_class, 0);
            
            if (score > model_score_threshold_[target_class]) {
                class_list.push_back(target_class);
                confidence_list.push_back(score);

                const float x = detection_outputs.at<float>(0, i);
                const float y = detection_outputs.at<float>(1, i);
                const float w = detection_outputs.at<float>(2, i);
                const float h = detection_outputs.at<float>(3, i);

                cv::Rect box;
                box.x = static_cast<int>(x);
                box.y = static_cast<int>(y);
                box.width = static_cast<int>(w);
                box.height = static_cast<int>(h);

                box_list.push_back(box);
                break; // Only add this detection once if it matches multiple target classes
            }
        }
    }

    // Apply NMS separately for each class for better performance
    detections_.clear();
    
    // Process class 0
    std::vector<int> indices_class0;
    std::vector<cv::Rect> boxes_class0;
    std::vector<float> scores_class0;
    
    for (int i = 0; i < class_list.size(); i++) {
        if (class_list[i] == 0) {
            indices_class0.push_back(i);
            boxes_class0.push_back(box_list[i]);
            scores_class0.push_back(confidence_list[i]);
        }
    }
    
    std::vector<int> NMS_result0;
    if (!boxes_class0.empty()) {
        cv::dnn::NMSBoxes(boxes_class0, scores_class0, model_score_threshold_[0], model_NMS_threshold_[0], NMS_result0);
        
        for (int idx : NMS_result0) {
            Detection result;
            int original_idx = indices_class0[idx];
            
            result.class_id = 0;
            result.confidence = confidence_list[original_idx];
            result.box = GetBoundingBox(box_list[original_idx]);
            
            detections_.push_back(result);
        }
    }
    
    // Process class 41
    std::vector<int> indices_class41;
    std::vector<cv::Rect> boxes_class41;
    std::vector<float> scores_class41;
    
    for (int i = 0; i < class_list.size(); i++) {
        if (class_list[i] == 41) {
            indices_class41.push_back(i);
            boxes_class41.push_back(box_list[i]);
            scores_class41.push_back(confidence_list[i]);
        }
    }
    
    std::vector<int> NMS_result41;
    if (!boxes_class41.empty()) {
        cv::dnn::NMSBoxes(boxes_class41, scores_class41, model_score_threshold_[41], model_NMS_threshold_[41], NMS_result41);
        
        for (int idx : NMS_result41) {
            Detection result;
            int original_idx = indices_class41[idx];
            
            result.class_id = 41;
            result.confidence = confidence_list[original_idx];
            result.box = GetBoundingBox(box_list[original_idx]);
            
            detections_.push_back(result);
        }
    }
}

cv::Rect Inference::GetBoundingBox(const cv::Rect &src) {
    cv::Rect box = src;

    box.x = (box.x - 0.5 * box.width) * factor_.x;
    box.y = (box.y - 0.5 * box.height) * factor_.y;
    box.width *= factor_.x;
    box.height *= factor_.y;
    
    return box;
}