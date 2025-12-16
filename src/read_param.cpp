// Copyright 2025 perception
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ai_seg_mask_pointcloud_roi_extractor/ai_seg_mask_pointcloud_roi_extractor.hpp"

namespace seg_mask_roi_extractor
{
    bool AISegMaskPointCloudROIExtractor::parserYamlParam()
    {
        std::string model_classes_config;
        model_classes_config = ament_index_cpp::get_package_share_directory("ai_seg_mask_pointcloud_roi_extractor") + "/config/" +
                                    params_->confidence_threshold_file_path;
        RCLCPP_INFO(get_logger(), "Parser data, model_classes_config = %s",  model_classes_config.c_str());
        
        try
        {
            YAML::Node config = YAML::LoadFile( model_classes_config);

            for (const auto& node : config) 
            {
                if (node.second.IsScalar()) 
                {
                    std::string class_name = node.first.as<std::string>();
                    double confidence = node.second.as<double>();
                    target_names_confindece_[class_name] = confidence;
                }
            }

            for (const auto& pair : target_names_confindece_) 
            {
                RCLCPP_INFO(get_logger(), "Target_Class_Info : ");
                RCLCPP_INFO(get_logger(), "----------Name: %s  --> Confidence: %f", pair.first.c_str(), pair.second);
            }
        }
        catch (const YAML::Exception &e)
        {
            throw std::runtime_error( model_classes_config + ":load yaml error: " + std::string(e.what()));
            return false;
        }

        return true;
    }
} // namespace seg_mask_roi_extractor