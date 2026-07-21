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
    bool MaskPcRoiExtractor::convertDepthToPointcloud(const cv::Mat& depth,
                                        const cv::Mat &mask,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                        float min_depth,
                                        float max_depth)
    {
        // An empty mask means "no masking" (keep all valid depth pixels). Only enforce the
        // size match when a mask is actually provided. This lets the debug path skip allocating
        // and scanning an all-zero full-res mask.
        const bool use_mask = !mask.empty();
        if (use_mask && depth.size() != mask.size())
        {
            RCLCPP_ERROR(get_logger(), "Depth and mask images have different sizes! Skipping processing.");
            return false;
        }
        // Validate depth encoding: ptr<uint16_t> assumes CV_16UC1.
        if (depth.type() != CV_16UC1)
        {
            RCLCPP_ERROR(get_logger(),
                         "Depth must be CV_16UC1, got type=%d", depth.type());
            return false;
        }

        // Check if camera info is available
        auto camera_info = getCameraInfo();
        if (!camera_info)
        {
            RCLCPP_WARN(get_logger(), "Camera info not available, cannot convert to point cloud");
            return false;
        }
        if (!cloud)
        {
            cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        }
        cloud->clear();
        try
        {
            // Get camera intrinsic parameters
            double fx = camera_info->k[0];  // focal length x
            double fy = camera_info->k[4];  // focal length y
            double cx = camera_info->k[2];  // principal point x
            double cy = camera_info->k[5];  // principal point y
            const float inv_fx = (fx != 0.0) ? static_cast<float>(1.0 / fx) : 0.0f;
            const float inv_fy = (fy != 0.0) ? static_cast<float>(1.0 / fy) : 0.0f;

            // Reserve up front (debug path): worst case is the whole image.
            cloud->points.reserve(static_cast<size_t>(depth.rows * depth.cols));

            // Convert depth image to 3D points (direct pointer access, no bounds-checked at<>())
            for (int v = 0; v < depth.rows; ++v)
            {
                const uchar* mrow = use_mask ? mask.ptr<uchar>(v) : nullptr;
                const uint16_t* drow = depth.ptr<uint16_t>(v);
                for (int u = 0; u < depth.cols; ++u)
                {
                    if (mrow && mrow[u] == 255) continue;

                    float depth_value = drow[u] * DEPTH_SCALE;

                    // Skip invalid depth values
                    if (depth_value <= min_depth || depth_value > max_depth || std::isnan(depth_value) || std::isinf(depth_value))
                    {
                        continue;
                    }
                    // Calculate 3D point (multiplication instead of division)
                    float x = (static_cast<float>(u) - cx) * depth_value * inv_fx;
                    float y = (static_cast<float>(v) - cy) * depth_value * inv_fy;
                    pcl::PointXYZ point;
                    // point.z = depth_value;
                    // point.y = y;
                    // point.x = x;

                    point.x = depth_value;
                    point.y = -x;
                    point.z = -y;

                    cloud->push_back(point);
                }
            }
            // Set point cloud metadata
            {
                cloud->width = cloud->size();
                cloud->height = 1;
                cloud->is_dense = false;
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Error converting depth to point cloud: %s", e.what());
            return false;
        }
        return true;
    }

    bool MaskPcRoiExtractor::setDepthZeroByMask(const cv::Mat &mask, 
                                                cv::Mat &depth_img, 
                                                bool use_extractor)
    {
        if (depth_img.empty())
        {
            RCLCPP_ERROR(get_logger(), "depth_img is empty!");
            return false;
        }
        // An empty mask means "no region to filter this frame" (e.g. no detections).
        // Treat as a no-op rather than an error to avoid log spam on every idle frame.
        if (mask.empty())
        {
            return true;
        }
        if (mask.size() != depth_img.size())
        {
            RCLCPP_ERROR(get_logger(), "Mask and depth_img size mismatch! mask: %dx%d, depth: %dx%d",
                        mask.cols, mask.rows, depth_img.cols, depth_img.rows);
            return false;
        }
        if (mask.type() != CV_8UC1)
        {
            RCLCPP_ERROR(get_logger(), "Mask must be single-channel CV_8UC1 (type=0)! Current type: %d", mask.type());
            return false;
        }
        const int depth_type = depth_img.type();
        if (depth_type != CV_32FC1 && depth_type != CV_16UC1)
        {
            RCLCPP_ERROR(get_logger(), "Depth_img only supports CV_32FC1(5)/CV_16UC1(2)! Current type: %d", depth_type);
            return false;
        }
        // Ensure mask is continuous for direct pointer access inside forEach lambda.
        // (forEach iterates in row-major order, so a non-continuous sub-matrix would
        // break manual data[] indexing.)
        cv::Mat mask_cont = mask.isContinuous() ? mask : mask.clone();
        try
        {
            const uchar* mask_data = mask_cont.data;
            const size_t mask_step = mask_cont.step;
            auto set_depth_zero = [&](auto& depth_pixel, const int* pos)
            {
                const size_t row = static_cast<size_t>(pos[0]);
                const size_t col = static_cast<size_t>(pos[1]);
                const uchar mask_pixel = mask_data[row * mask_step + col];
                const bool need_set_zero = use_extractor ? (mask_pixel != 255) : (mask_pixel == 255);
                if (need_set_zero)
                {
                    depth_pixel = 0;
                }
            };
            if (depth_type == CV_32FC1)
            {
                depth_img.forEach<float>(set_depth_zero);
            }
            else
            {
                depth_img.forEach<ushort>(set_depth_zero);
            }
        }
        catch (const cv::Exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Exception in setDepthZeroByMask: %s", e.what());
            return false;
        }
        RCLCPP_DEBUG(get_logger(), "setDepthZeroByMask success! Depth type: %s, use_extractor: %s",
                    (depth_type == CV_32FC1 ? "CV_32FC1" : "CV_16UC1"),
                    (use_extractor ? "true" : "false"));
        return true;
    }

    bool MaskPcRoiExtractor::generateMaskByDepth(const cv::Mat& depth_img, cv::Mat& mask)
    {
        if (depth_img.empty())
        {
            RCLCPP_ERROR(get_logger(), "Input depth_img is empty!");
            return false;
        }
        int depth_type = depth_img.type();
        if (depth_type != CV_32FC1 && depth_type != CV_16UC1)
        {
            RCLCPP_ERROR(get_logger(), "Depth_img only support CV_32FC1(5)/CV_16UC1(2)! Current type: %d", depth_type);
            return false;
        }
        // Reuse the caller-provided buffer when it already matches (pool-friendly): only
        // (re)allocate on a size/type mismatch, then zero it. Avoids a full-res heap alloc
        // every frame on the always-on filter path.
        if (mask.rows != depth_img.rows || mask.cols != depth_img.cols || mask.type() != CV_8UC1)
        {
            mask.create(depth_img.size(), CV_8UC1);
        }
        mask.setTo(0);
        try
        {
            uchar* mask_ptr = mask.data;
            int mask_step = mask.isContinuous() ? mask.cols : mask.step;
            auto genMaskFunc = [&](auto& depth_pixel, const int* pos)
            {
                bool is_valid_depth = (depth_pixel > 0);
                if (is_valid_depth)
                {
                    mask_ptr[pos[0] * mask_step + pos[1]] = 255;
                }
            };
            if (depth_type == CV_32FC1)
            {
                depth_img.forEach<float>(genMaskFunc);
            }
            else if (depth_type == CV_16UC1)
            {
                depth_img.forEach<ushort>(genMaskFunc);
            }
        }
        catch (const cv::Exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Exception in generateMaskByDepth: %s", e.what());
            mask.release();
            return false;
        }
        RCLCPP_DEBUG(get_logger(), "Generate mask by depth success! Mask size: %dx%d", mask.cols, mask.rows);
        return true;
    }

} // namespace robot::mask_pc_roi_extractor
