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
#include <cctype>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "ai_seg_mask_pointcloud_roi_extractor/ai_seg_mask_pointcloud_roi_extractor.hpp"

namespace seg_mask_roi_extractor
{
    bool AISegMaskPointCloudROIExtractor::dynamicParaCallback()
    {
        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&AISegMaskPointCloudROIExtractor::onParameterChange, this, std::placeholders::_1));

        if (!param_callback_)
        {
            param_callback_.reset();
            RCLCPP_ERROR(get_logger(),
                         "Failed to register dynamic parameter callback");
            return false;
        }
        return true;
    }

    rclcpp::Logger::Level AISegMaskPointCloudROIExtractor::stringToLogLevel(const std::string& level_str)
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

    rcl_interfaces::msg::SetParametersResult AISegMaskPointCloudROIExtractor::onParameterChange(
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
} // namespace seg_mask_roi_extractor
