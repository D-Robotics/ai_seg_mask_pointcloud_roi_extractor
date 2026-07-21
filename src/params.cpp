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

#include <algorithm>
#include <map>
#include <unordered_map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "mask_pc_roi_extractor/mask_pc_roi_extractor.hpp"

namespace robot::mask_pc_roi_extractor
{

// ==================== Dynamic Parameter Callbacks ====================

bool MaskPcRoiExtractor::dynamicParaCallback()
{
    param_callback_ = this->add_on_set_parameters_callback(
        std::bind(&MaskPcRoiExtractor::onParameterChange, this, std::placeholders::_1));

    if (!param_callback_)
    {
        param_callback_.reset();
        RCLCPP_ERROR(get_logger(),
                     "Failed to register dynamic parameter callback");
        return false;
    }
    return true;
}

rclcpp::Logger::Level MaskPcRoiExtractor::stringToLogLevel(const std::string& level_str)
{
    // O(1) lookup instead of O(n) if-else chain.
    // switch() cannot be used on std::string (C++ only allows integral/enum types).
    static const std::unordered_map<std::string, rclcpp::Logger::Level> kLevelMap = {
        {"debug",    rclcpp::Logger::Level::Debug},
        {"info",     rclcpp::Logger::Level::Info},
        {"warn",     rclcpp::Logger::Level::Warn},
        {"error",    rclcpp::Logger::Level::Error},
        {"critical", rclcpp::Logger::Level::Fatal},
    };

    // Normalize to lowercase.
    std::string key = level_str;
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    auto it = kLevelMap.find(key);
    if (it != kLevelMap.end())
    {
        RCLCPP_INFO(get_logger(), "Log level updated to: %s", key.c_str());
        return it->second;
    }

    RCLCPP_WARN(get_logger(), "Unknown log level '%s', falling back to INFO",
                level_str.c_str());
    return rclcpp::Logger::Level::Info;
}

rcl_interfaces::msg::SetParametersResult MaskPcRoiExtractor::onParameterChange(
    const std::vector<rclcpp::Parameter> &params)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (const auto &param : params)
    {
        const std::string& name = param.get_name();
        if (name == "log_level")
        {
            auto level = stringToLogLevel(param.as_string());
            get_logger().set_level(level);
            rclcpp::get_logger("rcl").set_level(level);
            rclcpp::get_logger("rclcpp").set_level(level);
        }
        else if (name == "debug")
        {
            params_->debug = param.as_bool();
            RCLCPP_INFO(get_logger(), "debug updated to: %s", params_->debug ? "true" : "false");
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Unknown parameter '%s' — ignored", name.c_str());
        }
    }
    return result;
}

// ==================== Class Name Parameter Parsing ====================

void MaskPcRoiExtractor::rebuildTargetClassIdSet()
{
    // Copy the (immutable) class-name dictionary under the lock, then build both ID sets
    // (depth filtering + ROI extraction) without holding the lock.
    // name:id
    std::shared_ptr<const std::unordered_map<std::string, size_t>> class_names;
    {
        std::lock_guard<std::mutex> lock(class_info_mutex_);
        class_names = class_names_id_;
    }
    if (!class_names) return;  // class-info not yet received; rebuilt later in classInfoCallback

    // Filter out categories that do not exist in the model
    auto build_set = [&](const std::unordered_map<std::string, double>& conf_map)
    {
        auto s = std::make_shared<std::unordered_set<int>>();
        for (const auto& pair : conf_map)
        {
            auto it = class_names->find(pair.first);
            if (it != class_names->end())
            {
                // id+1 (matching class-ID mask encoding)
                s->insert(static_cast<int>(it->second) + 1);
            }
        }
        return s;
    };

    {
        std::lock_guard<std::mutex> lock(class_info_mutex_);
        target_class_ids_     = build_set(target_names_confidence_);      // depth filtering
        target_roi_class_ids_ = build_set(target_names_roi_confidence_);  // ROI extraction

        // Merged set = union of both, used by the parsing stage to retain
        // all classes that either pipeline may need.
        auto merged = std::make_shared<std::unordered_set<int>>(*target_class_ids_);
        if (target_roi_class_ids_)
        {
            merged->insert(target_roi_class_ids_->begin(), target_roi_class_ids_->end());
        }
        target_class_ids_merged_ = merged;
    }
}

bool MaskPcRoiExtractor::parserClassnamesParam()
{
    // Helper: load a YAML config and populate a name→confidence map.
    auto load_config = [this](const std::string& file_path,
                              std::unordered_map<std::string, double>& out_map,
                              const char* label)
    {
        out_map.clear();  // defensive: prevent stale entries on re-configuration
        std::string full_path = ament_index_cpp::get_package_share_directory("mask_pc_roi_extractor")
                                + "/config/" + file_path;
        RCLCPP_INFO(get_logger(), "Loading %s config: %s", label, full_path.c_str());
        try
        {
            YAML::Node config = YAML::LoadFile(full_path);
            for (const auto& node : config)
            {
                if (node.second.IsScalar())
                {
                    out_map[node.first.as<std::string>()] = node.second.as<double>();
                }
            }
            for (const auto& pair : out_map)
            {
                RCLCPP_INFO(get_logger(), "  [%s] %s -> conf=%.2f",
                            label, pair.first.c_str(), pair.second);
            }
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to load %s config %s: %s",
                         label, full_path.c_str(), e.what());
            RCLCPP_WARN(get_logger(), "%s thresholds are empty — "
                        "corresponding detections will be skipped.", label);
        }
    };

    // Confidence level threshold for the filtered or extracted depth mask region
    load_config(params_->confidence_threshold_file_path,
                target_names_confidence_, "depth");
    
    // Confidence level threshold for the extracted ROI region
    load_config(params_->confidence_threshold_roi_file_path,
                target_names_roi_confidence_, "ROI");

    // Merge both configs into a single map for the parsing stage.
    // Union of keys; when a key appears in both, take the lower confidence
    // to avoid filtering out detections that either pipeline might need.
    target_names_merged_conf_ = target_names_confidence_;
    for (const auto& [name, conf] : target_names_roi_confidence_)
    {
        auto it = target_names_merged_conf_.find(name);
        if (it == target_names_merged_conf_.end())
        {
            target_names_merged_conf_[name] = conf;
        }
        else
        {
            it->second = std::min(it->second, conf);
        }
    }

    rebuildTargetClassIdSet();
    return true;
}

} // namespace robot::mask_pc_roi_extractor
