// Copyright (c) 2025，D-Robotics.
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
                        const std::string cls_name = node.first.as<std::string>();
                        out_map[cls_name] = node.second.as<double>();
                        if (label == std::string("ROI"))
                        {
                            // File order preserved: drives per-point palette
                            // assignment (cell_colors.list line N -> class N).
                            roi_class_names_.push_back(cls_name);
                        }
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

    void AISegMaskPointCloudROIExtractor::loadRoiClassColors()
    {
        // 1. Base scheme for every class id — identical to the semantic_map
        //    visualizer fallback: 20 hue groups x 4 brightness levels.
        roi_color_base_.clear();
        roi_color_base_.reserve(80);
        for (int id = 0; id < 80; ++id)
        {
            const float h = static_cast<float>(id / 4) * 360.0f / 20.0f;
            const float s = 1.0f;
            const float v = 0.6f + 0.4f * (static_cast<float>(id % 4) / 3.0f);
            const float c = v * s;
            const float x = c * (1.0f - std::fabs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
            const float m = v - c;
            float r = 0.0f, g = 0.0f, b = 0.0f;
            if (h < 60.0f)       { r = c; g = x; }
            else if (h < 120.0f) { r = x; g = c; }
            else if (h < 180.0f) { g = c; b = x; }
            else if (h < 240.0f) { g = x; b = c; }
            else if (h < 300.0f) { r = x; b = c; }
            else                 { r = c; b = x; }
            const auto ch = [m](float f) -> uint32_t
            { return static_cast<uint32_t>(std::lround((f + m) * 255.0f)) & 0xFFu; };
            roi_color_base_.push_back((ch(r) << 16) | (ch(g) << 8) | ch(b));
        }

        // 2. Palette: semantic_map's cell_colors.list — the same file the
        //    semantic_map visualizer uses for object-id markers.
        roi_color_palette_.clear();
        try
        {
            const std::string path =
                ament_index_cpp::get_package_share_directory("semantic_map")
                + "/config/cell_colors.list";
            std::ifstream fi(path);
            if (fi)
            {
                std::string line;
                while (std::getline(fi, line))
                {
                    const auto b = line.find_first_not_of(" \t\r\n");
                    if (b == std::string::npos || line[b] == '#') continue;
                    const auto e = line.find_last_not_of(" \t\r\n");
                    const std::string hex = line.substr(b, e - b + 1);
                    if (hex.size() != 6) continue;
                    roi_color_palette_.push_back(
                        static_cast<uint32_t>(std::stoul(hex, nullptr, 16)));
                }
            }
        }
        catch (const std::exception& e)
        {
            // semantic_map not installed (standalone extractor) or unreadable
            // palette file: HSV base colors only.
            RCLCPP_WARN(get_logger(), "cell_colors.list unavailable (%s) — "
                        "non-whitelist HSV fallback covers all ids", e.what());
        }
        RCLCPP_INFO(get_logger(),
                    "ROI point colors: %zu-class HSV fallback + %zu palette entries "
                    "(source: semantic_map config/cell_colors.list)",
                    roi_color_base_.size(), roi_color_palette_.size());
    }

    void AISegMaskPointCloudROIExtractor::rebuildRoiColorOverlay()
    {
        auto cn = getClassNamesInfo();
        if (!cn || roi_color_palette_.empty())
        {
            return;
        }
        auto overlay = std::make_shared<std::unordered_map<int32_t, uint32_t>>();
        for (size_t i = 0; i < roi_class_names_.size() && i < roi_color_palette_.size(); ++i)
        {
            auto it = cn->find(roi_class_names_[i]);
            if (it != cn->end())
            {
                (*overlay)[static_cast<int32_t>(it->second)] = roi_color_palette_[i];
            }
        }
        const size_t assigned = overlay->size();
        std::lock_guard<std::mutex> lock(roi_color_mutex_);
        roi_color_overlay_ = std::move(overlay);
        RCLCPP_INFO(get_logger(),
                    "ROI point color overlay rebuilt: %zu whitelist classes use "
                    "cell_colors.list, remaining ids use the HSV fallback",
                    assigned);
    }
} // namespace seg_mask_roi_extractor
