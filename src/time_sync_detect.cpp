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

namespace robot::ai_seg_mask_pointcloud_roi_extractor
{
    void AISegMaskPointCloudROIExtractor::timeSyncStatus()
    {
        rclcpp::Time now = this->get_clock()->now();

        if (last_time_ == -std::numeric_limits<double>::infinity())
        {
            RCLCPP_WARN(get_logger(), "========= No synchronized data received ==========");
        }
        else
        {
            rclcpp::Duration time_delay = now - last_receive_time_;

            if (std::abs(time_delay.seconds()) > sync_time_delta_)
            {
                RCLCPP_WARN(get_logger(),"Sync ata timeout! No synchronized data for %.2fs (threshold: %.2fs)",
                                std::abs(time_delay.seconds()), sync_time_delta_);
            }
        }
        return;
    }

    bool AISegMaskPointCloudROIExtractor::timeSyncDetect()
    {
        timeSyncTimer_ = create_wall_timer(std::chrono::seconds(1), std::bind(& AISegMaskPointCloudROIExtractor::timeSyncStatus, this));
        return true;
    }
} 
