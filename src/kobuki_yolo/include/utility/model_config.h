#ifndef MODEL_CONFIG_H_
#define MODEL_CONFIG_H_
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>


class ModelConfig{
    public:
        ModelConfig(const std::string &model_config_path);

        std::vector<std::string> GetObjectName() { return config_object_name_; }
        std::vector<float> GetScoreThreshold() { return min_score_threshold_; }
        int GetInputSize() { return model_input_size_; }
        std::string GetModelPath() { return model_path_; }

    private:
        std::vector<std::string> config_object_name_;
	    std::vector<float> min_score_threshold_;

        int model_input_size_;
        std::string model_path_;
};

#endif // MODEL_CONFIG_H_
