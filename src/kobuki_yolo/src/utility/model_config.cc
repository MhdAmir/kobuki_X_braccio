#include "utility/model_config.h"

ModelConfig::ModelConfig(const std::string &model_config_path) {
    const YAML::Node model_config=YAML::LoadFile(model_config_path);
    
    model_input_size_ = model_config["model_input_size"].as<int>();
    model_path_ = model_config["model_path"].as<std::string>();

    for (const auto& data : model_config["class"]) {
        const auto& value = data.second; // Corresponding value (map with "label" and "confidence")

        float confidence = value["confidence"].as<float>();
        std::string label = value["name"].as<std::string>();

        config_object_name_.push_back(label);
        min_score_threshold_.push_back(confidence);
    }
}