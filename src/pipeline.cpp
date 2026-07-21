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

#include "mask_pc_roi_extractor/mask_pc_roi_extractor.hpp"

namespace robot::mask_pc_roi_extractor
{
    void MaskPcRoiExtractor::processFilteredPublish(cv::Mat& depth_img,
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

    void MaskPcRoiExtractor::processROIPointClouds(const cv::Mat& depth_img,
                                                const cv::Mat& seg_mask_class_id,
                                                const std::vector<BoxInfo>& filtered_all_box_info,
                                                double time_stamp,
                                                const std::string& frame_id,
                                                rclcpp::Publisher<::mask_pc_roi_extractor::msg::ROIPointClouds>::SharedPtr roi_cloud_pub,
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

        ::mask_pc_roi_extractor::msg::ROIPointClouds roi_clouds_msg;
        roi_clouds_msg.header.frame_id = frame_id;
        roi_clouds_msg.header.stamp = stamp;

        for (const auto& box_info : filtered_all_box_info)
        {
            // Secondary confidence filter: the parsing stage uses a merged
            // (lower) threshold to avoid dropping detections.  Now apply the
            // ROI-specific threshold, which may be stricter.
            auto conf_it = roi_conf.find(box_info.name);
            if (conf_it != roi_conf.end() && box_info.confidence < conf_it->second)
                continue;
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

            cloud->width = cloud->size();
            cloud->height = 1;
            cloud->is_dense = false;
            // Convert to ROS PointCloud2, then MOVE it into the ROI wrapper (no message copy).
            ::mask_pc_roi_extractor::msg::ROIPointCloud roi_cloud;
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
            // Publish by unique_ptr: move the aggregate into the middleware (zero-copy under
            // intra-process comms; avoids the publish(const&) copy otherwise).
            auto out = std::make_unique<::mask_pc_roi_extractor::msg::ROIPointClouds>(std::move(roi_clouds_msg));
            roi_cloud_pub->publish(std::move(out));
        }
        return;
    }

} // namespace robot::mask_pc_roi_extractor
