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
#include "ai_seg_mask_pointcloud_roi_extractor/depth_continuity.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <vector>

namespace seg_mask_roi_extractor
{
    void AISegMaskPointCloudROIExtractor::processFilteredPublish(cv::Mat& depth_img,
                                                const cv::Mat& mask_img,
                                                double time_stamp,
                                                const std::string& frame_id,
                                                bool use_extractor,
                                                float min_depth,
                                                float max_depth,
                                                bool debug,
                                                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_pub,
                                                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_mask_pub,
                                                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub)
    {
        // Set the depth map region corresponding to the mask to 0 (modifies depth_img in place)
        setDepthZeroByMask(mask_img, depth_img, use_extractor);

        // Publish the processed depth map
        pubImage(time_stamp, frame_id, depth_img, filtered_depth_pub);

        // Obtain the mask of the depth map (reuse a pooled full-res buffer instead of
        // allocating a fresh cv::Mat every frame).
        cv::Mat depth_mask = mask_pool_.acquire(depth_img.rows, depth_img.cols, CV_8UC1);
        if (!generateMaskByDepth(depth_img, depth_mask))
        {
            RCLCPP_ERROR(get_logger(), "Failed to obtain the mask of the depth map!");
            mask_pool_.release(depth_mask);
            return;
        }

        // Published processed depth map mask
        pubImage(time_stamp, frame_id, depth_mask, filtered_depth_mask_pub);
        mask_pool_.release(depth_mask);

        if (debug)
        {
            // Depth Map to Point Cloud (empty mask = no masking; avoids a full-res zero alloc)
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cv::Mat mask_label;  // empty on purpose
            if (!convertDepthToPointcloud(depth_img, mask_label, cloud, min_depth, max_depth))
            {
                RCLCPP_ERROR(get_logger(), "Depth map to point cloud conversion failed!");
                return;
            }
            pubPointCloud(*cloud, frame_id, time_stamp, filtered_cloud_pub);
        }

        return;
    }

    void AISegMaskPointCloudROIExtractor::processROIPointClouds(const cv::Mat& depth_img,
                                                const cv::Mat& seg_mask_class_id,
                                                const std::vector<BoxInfo>& filtered_all_box_info,
                                                double time_stamp,
                                                const std::string& frame_id,
                                                rclcpp::Publisher<::ai_seg_mask_pointcloud_roi_extractor::msg::ROIPointClouds>::SharedPtr roi_cloud_pub,
                                                int img_width,
                                                int img_height,
                                                float min_depth,
                                                float max_depth,
                                                const std::unordered_map<std::string, double>& roi_conf)
    {
        if (filtered_all_box_info.empty())
        {
            return;
        }
        // Validate depth encoding: ptr<uint16_t> below assumes CV_16UC1.
        // CV_32FC1 would produce garbage values (cast of 4-byte float to 2-byte int).
        if (depth_img.type() != CV_16UC1)
        {
            RCLCPP_ERROR(get_logger(),
                         "Depth must be CV_16UC1 for ROI extraction, got type=%d",
                         depth_img.type());
            return;
        }
        // Get camera intrinsics once
        auto camera_info = getCameraInfo();
        if (!camera_info)
        {
            RCLCPP_WARN(get_logger(), "Camera info not available, cannot extract ROI point clouds");
            return;
        }
        const double fx = camera_info->k[0];
        const double fy = camera_info->k[4];
        const double cx = camera_info->k[2];
        const double cy = camera_info->k[5];
        // Precompute reciprocals: replace division in the inner loop with multiplication
        // (cheaper, and lets the compiler emit NEON multiply instructions).
        const float inv_fx = (fx != 0.0) ? static_cast<float>(1.0 / fx) : 0.0f;
        const float inv_fy = (fy != 0.0) ? static_cast<float>(1.0 / fy) : 0.0f;

        const bool has_class_mask = !seg_mask_class_id.empty() &&
                                     seg_mask_class_id.rows == img_height &&
                                     seg_mask_class_id.cols == img_width;

        // Compute the header stamp once (constant for all sub-clouds of this frame).
        const rclcpp::Time stamp = timeStampDoubleToTime(time_stamp);

        ::ai_seg_mask_pointcloud_roi_extractor::msg::ROIPointClouds roi_clouds_msg;
        roi_clouds_msg.header.frame_id = frame_id;
        roi_clouds_msg.header.stamp = stamp;

        for (const auto& box_info : filtered_all_box_info)
        {
            // Secondary confidence filter: the parsing stage uses a merged
            // (lower) threshold to avoid dropping detections.  Now apply the
            // ROI-specific threshold, which may be stricter.
            // Classes not in roi_conf are skipped entirely — only configured
            // classes are published as ROI point clouds.
            auto conf_it = roi_conf.find(box_info.name);
            if (conf_it == roi_conf.end()) continue;
            if (box_info.confidence < conf_it->second) continue;
            // Clamp rect to image bounds
            int x_start = std::max(0, box_info.x_offset);
            int y_start = std::max(0, box_info.y_offset);
            int x_end = std::min(img_width, box_info.x_offset + box_info.width);
            int y_end = std::min(img_height, box_info.y_offset + box_info.height);

            if (x_start >= x_end || y_start >= y_end)
            {
                continue;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            const int target_pixel_value = static_cast<int>(box_info.id) + 1;

            // Reserve the maximum possible number of points (rect area) up front so that
            // push_back never triggers vector reallocation / buffer re-copy.
            const int rect_area = (x_end - x_start) * (y_end - y_start);
            if (rect_area > 0) cloud->points.reserve(static_cast<size_t>(rect_area));

            // Iterate only over the rect area
            if (has_class_mask)
            {
                // Use class-ID mask for precise instance segmentation
                for (int v = y_start; v < y_end; ++v)
                {
                    const uchar* class_row = seg_mask_class_id.ptr<uchar>(v);
                    const uint16_t* depth_row = depth_img.ptr<uint16_t>(v);

                    for (int u = x_start; u < x_end; ++u)
                    {
                        if (class_row[u] != target_pixel_value) continue;

                        float depth_value = depth_row[u] * DEPTH_SCALE;
                        if (depth_value <= min_depth || depth_value > max_depth) continue;

                        // Depth continuity check: filter out background pixels
                        // at object boundaries (depth discontinuity).
                        if (params_->depth_continuity_check &&
                            !isDepthConsistent(depth_img, seg_mask_class_id,
                                               u, v, target_pixel_value,
                                               depth_value,
                                               static_cast<float>(params_->max_depth_diff),
                                               DEPTH_SCALE))
                        {
                            continue;
                        }

                        float x = (static_cast<float>(u) - cx) * depth_value * inv_fx;
                        float y = (static_cast<float>(v) - cy) * depth_value * inv_fy;

                        cloud->push_back(pcl::PointXYZ(x, y, depth_value));
                    }
                }
            }
            else
            {
                // No class mask available: use entire bbox rect as mask
                for (int v = y_start; v < y_end; ++v)
                {
                    const uint16_t* depth_row = depth_img.ptr<uint16_t>(v);

                    for (int u = x_start; u < x_end; ++u)
                    {
                        float depth_value = depth_row[u] * DEPTH_SCALE;
                        if (depth_value <= min_depth || depth_value > max_depth) continue;

                        float x = (static_cast<float>(u) - cx) * depth_value * inv_fx;
                        float y = (static_cast<float>(v) - cy) * depth_value * inv_fy;

                        cloud->push_back(pcl::PointXYZ(x, y, depth_value));
                    }
                }
            }

            if (cloud->empty()) continue;

            // Instance-level adaptive percentile depth gate (2026-09-14).
            // Why: radial "tail" streaks from occlusion boundaries are generated
            // as one coherent run in a single frame — every streak pixel's 3x3
            // neighborhood is depth-consistent, so the local continuity check
            // above cannot see them (user-tested). But the streak sits at
            // background depth, far outside the object's own depth span.
            // Adaptive (not fixed tau): keep-band = [p10 - m, p90 + m] with
            // m = max(tau_min, p90 - p10). The band scales with the instance's
            // depth extent, so large objects (sofa/table, if whitelisted later)
            // keep their far edges, while tails — background bleed typically
            // >= 0.8 m beyond the body — are cut entirely. Percentiles tolerate
            // up to ~10%/90% contamination by construction.
            // tau_min <= 0 disables. Cost: O(N) copy + 2x nth_element per box.
            const float gate_tau = static_cast<float>(params_->instance_depth_gate_tau);
            if (gate_tau > 0.0f && cloud->size() >= 8)
            {
                std::vector<float> depths;
                depths.reserve(cloud->size());
                for (const auto& p : cloud->points) depths.push_back(p.z);
                const size_t n = depths.size();
                const size_t i10 = n / 10;
                const size_t i90 = std::min(n - 1, (n * 9) / 10);
                std::nth_element(depths.begin(), depths.begin() + i10, depths.end());
                const float p10 = depths[i10];
                std::nth_element(depths.begin() + i10, depths.begin() + i90, depths.end());
                const float p90 = depths[i90];

                const float margin = std::max(gate_tau, p90 - p10);
                const float lo = p10 - margin;
                const float hi = p90 + margin;

                size_t w = 0;
                for (size_t r = 0; r < cloud->size(); ++r)
                {
                    const float z = cloud->points[r].z;
                    if (z >= lo && z <= hi)
                        cloud->points[w++] = cloud->points[r];
                }
                cloud->points.resize(w);
                if (cloud->empty()) continue;
            }

            cloud->width = cloud->size();
            cloud->height = 1;
            cloud->is_dense = false;
            // Convert to ROS PointCloud2, then MOVE it into the ROI wrapper (no message copy).
            ::ai_seg_mask_pointcloud_roi_extractor::msg::ROIPointCloud roi_cloud;
            pcl::toROSMsg(*cloud, roi_cloud.cloud);
            roi_cloud.cloud.header.frame_id = frame_id;
            roi_cloud.cloud.header.stamp = stamp;
            roi_cloud.confidence = box_info.confidence;
            roi_cloud.id = static_cast<int32_t>(box_info.id);

            // MOVE the wrapper into the aggregate (avoids copying the whole PointCloud2 buffer).
            roi_clouds_msg.clouds.push_back(std::move(roi_cloud));
        }
        
        if (!roi_clouds_msg.clouds.empty())
        {
            // Log the classes present in this frame (deduplicated, one log per frame).
            {
                std::set<int> seen_ids;
                for (const auto& c : roi_clouds_msg.clouds)
                {
                    seen_ids.insert(c.id);
                }
                // Build a compact log line: "id:name, id:name, ..."
                std::string line;
                for (int id : seen_ids)
                {
                    if (!line.empty()) line += ", ";
                    line += std::to_string(id) + ":";
                    // Look up class name from the stored map (name→id, need reverse lookup).
                    // class_names_id_ is a shared_ptr<const unordered_map<string,size_t>>.
                    bool found = false;
                    if (auto cn = getClassNamesInfo())
                    {
                        for (const auto& kv : *cn)
                        {
                            if (static_cast<int>(kv.second) == id)
                            {
                                line += kv.first;
                                found = true;
                                break;
                            }
                        }
                    }
                    if (!found) line += "?";
                }
                RCLCPP_INFO(get_logger(), "[roi] frame classes: %s", line.c_str());
            }
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "[roi] frame classes: empty (no detections)");
        }

        // Always publish — even empty messages trigger semantic_map's callback
        // so decay continues when there are no detections.
        auto out = std::make_unique<::ai_seg_mask_pointcloud_roi_extractor::msg::ROIPointClouds>(std::move(roi_clouds_msg));
        roi_cloud_pub->publish(std::move(out));
        return;
    }

    void AISegMaskPointCloudROIExtractor::processROIVisual(const cv::Mat& depth_img,
                                              const cv::Mat& seg_mask_class_id,
                                              const std::vector<BoxInfo>& filtered_all_box_info,
                                              double timestamp,
                                              const std::string& frame_id,
                                              rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr roi_visual_pub,
                                              int img_width,
                                              int img_height,
                                              float min_depth,
                                              float max_depth,
                                              const std::unordered_map<std::string, double>& roi_conf)
    {
        (void)filtered_all_box_info;
        (void)roi_conf;
        (void)img_width;
        (void)img_height;
        (void)min_depth;
        (void)max_depth;

        if (!roi_visual_pub)
        {
            return;
        }
        if (depth_img.empty() || depth_img.type() != CV_16UC1)
        {
            return;
        }

        const int h = depth_img.rows;
        const int w = depth_img.cols;

        // ── 1. JET-colorize the depth map into a bgr8 base image ─────────────
        cv::Mat gray(h, w, CV_8UC1);
        {
            const float scale = (max_depth > min_depth) ? (255.0f / (max_depth - min_depth)) : 1.0f;
            for (int v = 0; v < h; ++v)
            {
                const uint16_t* drow = depth_img.ptr<uint16_t>(v);
                uchar* gptr = gray.ptr<uchar>(v);
                for (int u = 0; u < w; ++u)
                {
                    float d = static_cast<float>(drow[u]) * DEPTH_SCALE;
                    if (d < min_depth || d > max_depth || drow[u] == 0)
                    {
                        gptr[u] = 0;
                    }
                    else
                    {
                        gptr[u] = static_cast<uchar>(
                            std::min(255.0f, (d - min_depth) * scale));
                    }
                }
            }
        }
        cv::Mat base(h, w, CV_8UC3);
        cv::applyColorMap(gray, base, cv::COLORMAP_JET);

        // ── 2. Overlay segmentation mask (class-ID mask) on depth ────────────
        // Every pixel with a non-zero class-ID mask value is blended 50/50
        // with the per-class color.  No confidence filtering, no bounding boxes
        // — the raw segmentation mask drives the overlay.
        //
        // Per-class color palette (BGR).  Indexed by ((pixel_value - 1) % kNumClassColors).
        static const cv::Scalar kClassColors[] = {
            cv::Scalar(0, 0, 255),     // 0: red
            cv::Scalar(0, 255, 0),     // 1: green
            cv::Scalar(255, 0, 0),     // 2: blue
            cv::Scalar(0, 255, 255),   // 3: yellow
            cv::Scalar(255, 255, 0),   // 4: cyan
            cv::Scalar(255, 0, 255),   // 5: magenta
            cv::Scalar(0, 128, 255),   // 6: orange
            cv::Scalar(128, 255, 0),   // 7: lime
        };
        static constexpr int kNumClassColors = static_cast<int>(
            sizeof(kClassColors) / sizeof(kClassColors[0]));

        if (!seg_mask_class_id.empty() &&
            seg_mask_class_id.rows == h && seg_mask_class_id.cols == w)
        {
            for (int v = 0; v < h; ++v)
            {
                const uchar* class_row = seg_mask_class_id.ptr<uchar>(v);
                cv::Vec3b*   brow      = base.ptr<cv::Vec3b>(v);
                for (int u = 0; u < w; ++u)
                {
                    uchar pv = class_row[u];
                    if (pv == 0) continue;  // background

                    int cls_id = static_cast<int>(pv) - 1;
                    const cv::Scalar& c = kClassColors[cls_id % kNumClassColors];

                    // 50/50 blend: seg mask color + JET depth
                    brow[u][0] = static_cast<uchar>((brow[u][0] + c[0]) / 2);
                    brow[u][1] = static_cast<uchar>((brow[u][1] + c[1]) / 2);
                    brow[u][2] = static_cast<uchar>((brow[u][2] + c[2]) / 2);
                }
            }
        }

        pubImage(timestamp, frame_id, base, roi_visual_pub);
        return;
    }

} // namespace seg_mask_roi_extractor
