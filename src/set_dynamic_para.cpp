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

#include <map>
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
            param_callback_ = nullptr;
            return false;
        }
        return true;
    }

    rclcpp::Logger::Level AISegMaskPointCloudROIExtractor::stringToLogLevel(const std::string& level_str) 
    {
        std::string lower_str = level_str;
        std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);

        if (lower_str == "debug") 
        {
            RCLCPP_INFO(get_logger(), "log level need updated to debug");
            return rclcpp::Logger::Level::Debug;
        } 
        else if (lower_str == "info") 
        {
            RCLCPP_INFO(get_logger(), "log level need updated to info");
            return rclcpp::Logger::Level::Info;
        } 
        else if (lower_str == "warn") 
        {
            RCLCPP_INFO(get_logger(), "log level need updated to warn");
            return rclcpp::Logger::Level::Warn;
        } 
        else if (lower_str == "error") 
        {
            RCLCPP_INFO(get_logger(), "log level need updated to error");
            return rclcpp::Logger::Level::Error;
        } 
        else if (lower_str == "critical") 
        {
            RCLCPP_INFO(get_logger(), "log level need updated to critical");
            return rclcpp::Logger::Level::Fatal;
        } 
        else 
        {
            RCLCPP_INFO(get_logger(), "Invalid log level: %s, default use info", level_str.c_str());
            return rclcpp::Logger::Level::Info;
        }
    }

    rcl_interfaces::msg::SetParametersResult AISegMaskPointCloudROIExtractor::onParameterChange(
        const std::vector<rclcpp::Parameter> &params)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &param : params)
        {
            if (param.get_name() == "log_level")
            {
                auto level = stringToLogLevel(param.as_string());
                get_logger().set_level(level);
                rclcpp::get_logger("rcl").set_level(level);
                rclcpp::get_logger("rclcpp").set_level(level);

                RCLCPP_INFO(get_logger(), "log level updated to: %s", param.as_string().c_str());
            }
            else if (param.get_name() == "debug")
            {
                RCLCPP_INFO(get_logger(), "Update debug mode");
                params_->debug = param.as_bool();
                RCLCPP_INFO(get_logger(), "debug updated to: %s", params_->debug ? "true" : "false");
            }
            else if (param.get_name() == "topic_check")
            {
                RCLCPP_INFO(get_logger(), "Start check topic");
                if (!checkRequiredTopic())
                {
                    RCLCPP_ERROR(get_logger(), "Topic detection exception !");
                }
            }
            else if (param.get_name() == "time_debug")
            {
                RCLCPP_INFO(get_logger(), "Update time debug mode");
                params_->time_debug = param.as_bool();
                RCLCPP_INFO(get_logger(), "time debug updated to: %s", params_->time_debug ? "true" : "false");
            }
            else
            {
                
            }
        }

        return result;
    }
} // namespace seg_mask_roi_extractor
