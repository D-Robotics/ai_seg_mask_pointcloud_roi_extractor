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

#include "ai_seg_mask_pointcloud_roi_extractor/ai_seg_mask_pointcloud_roi_extractor.hpp"

namespace seg_mask_roi_extractor
{
    rclcpp::Time AISegMaskPointCloudROIExtractor::timeStampDoubleToTime(double time_stamp)
    {
        auto secs = static_cast<int32_t>(time_stamp);
        auto n_secs = static_cast<uint32_t>((time_stamp - floor(time_stamp)) * 1e9);
        return {secs, n_secs};
    }

    double AISegMaskPointCloudROIExtractor::timeStampTimeToDoubleSec(const rclcpp::Time &time_in)
    {
        return time_in.seconds();
    }

    double AISegMaskPointCloudROIExtractor::timeStampTimeToDoubleNanoSec(const rclcpp::Time &time_in)
    {
        return time_in.nanoseconds();
    }

    double AISegMaskPointCloudROIExtractor::timeStampTimeToDoubleMilliSec(const rclcpp::Time &time_in)
    {
        return time_in.nanoseconds() / 1e6;
    }

    std::string AISegMaskPointCloudROIExtractor::timeStampTimeToString(const rclcpp::Time &time_in)
    {
        std::time_t t = time_in.seconds();
        return std::ctime(&t);
    }

    double AISegMaskPointCloudROIExtractor::headerTimeStampTimeToDoubleSec(std_msgs::msg::Header header)
    {
        return header.stamp.sec + header.stamp.nanosec * 1e-9;
    }

    double AISegMaskPointCloudROIExtractor::headerTimeStampTimeToDoubleMilliSec(std_msgs::msg::Header header)
    {
        return header.stamp.sec * 1e3 + header.stamp.nanosec * 1e-6;
    }

    double AISegMaskPointCloudROIExtractor::headerTimeStampTimeToDoubleNanoSec(std_msgs::msg::Header header)
    {
        return header.stamp.sec * 1e9 + header.stamp.nanosec;
    }

    rclcpp::Time AISegMaskPointCloudROIExtractor::headerTimeStampToTime(std_msgs::msg::Header header)
    {
        return timeStampDoubleToTime(headerTimeStampTimeToDoubleSec(header));
    }

} // namespace seg_mask_roi_extractor