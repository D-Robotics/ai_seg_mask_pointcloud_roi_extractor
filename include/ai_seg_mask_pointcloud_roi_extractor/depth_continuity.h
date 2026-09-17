// Copyright 2026 perception
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

#pragma once

#include <opencv2/core.hpp>

namespace robot::ai_seg_mask_pointcloud_roi_extractor
{

/**
 * @brief Check if a pixel's depth is consistent with same-class neighbors.
 *
 * Collects depth values of same-class pixels in a 3×3 neighbourhood,
 * computes the median (requires >= 3 valid neighbours), and compares
 * the current pixel's depth against it.  Returns false if the depth
 * deviates more than max_depth_diff from the median — this typically
 * means the pixel belongs to the background (depth discontinuity at
 * object boundaries) and should be filtered out.
 *
 * @param depth_img           Full depth image (CV_16UC1, mm).
 * @param seg_mask_class_id   Full class-ID mask (CV_8UC1, pixel = class_id + 1).
 * @param u, v                Pixel coordinate to check.
 * @param target_pixel_value  Expected class pixel value (class_id + 1).
 * @param depth_value         Current pixel depth in meters.
 * @param max_depth_diff      Maximum allowed depth difference from neighbour median (meters).
 * @param depth_scale         Scale factor to convert raw depth to meters (e.g. 0.001f for mm).
 * @return true if depth is consistent, false if it should be filtered out.
 */
bool isDepthConsistent(
    const cv::Mat& depth_img,
    const cv::Mat& seg_mask_class_id,
    int u, int v,
    int target_pixel_value,
    float depth_value,
    float max_depth_diff,
    float depth_scale);

}  // namespace robot::ai_seg_mask_pointcloud_roi_extractor