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
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ai_seg_mask_pointcloud_roi_extractor/ai_seg_mask_pointcloud_roi_extractor.hpp"

namespace seg_mask_roi_extractor
{
    void AISegMaskPointCloudROIExtractor::rebuildTargetClassIdSet()
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

    bool AISegMaskPointCloudROIExtractor::parserYamlParam()
    {
        // Helper: load a YAML config and populate a name→confidence map.
        auto load_config = [this](const std::string& file_path,
                                  std::unordered_map<std::string, double>& out_map,
                                  const char* label)
        {
            out_map.clear();  // defensive: prevent stale entries on re-configuration
            std::string full_path = ament_index_cpp::get_package_share_directory("ai_seg_mask_pointcloud_roi_extractor")
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
} // namespace seg_mask_roi_extractor
