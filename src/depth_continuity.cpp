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

#include "ai_seg_mask_pointcloud_roi_extractor/depth_continuity.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace seg_mask_roi_extractor
{

bool isDepthConsistent(
    const cv::Mat& depth_img,
    const cv::Mat& seg_mask_class_id,
    int u, int v,
    int target_pixel_value,
    float depth_value,
    float max_depth_diff,
    float depth_scale)
{
    const int img_h = depth_img.rows;
    const int img_w = depth_img.cols;

    // Collect depth values of same-class pixels in a 3×3 neighbourhood.
    // Fixed-size array on the stack — no heap allocation.
    std::array<float, 9> neighbour_depths{};
    int count = 0;

    for (int dv = -1; dv <= 1; ++dv)
    {
        int nv = v + dv;
        if (nv < 0 || nv >= img_h) continue;

        const uint16_t* depth_row = depth_img.ptr<uint16_t>(nv);
        const uchar* class_row = seg_mask_class_id.ptr<uchar>(nv);

        for (int du = -1; du <= 1; ++du)
        {
            int nu = u + du;
            if (nu < 0 || nu >= img_w) continue;

            if (class_row[nu] != target_pixel_value) continue;
            if (du == 0 && dv == 0) continue;  // exclude self

            float nd = static_cast<float>(depth_row[nu]) * depth_scale;
            neighbour_depths[count++] = nd;
        }
    }

    // Need at least 3 valid neighbours for a reliable median.
    if (count < 3) return true;  // insufficient data — keep the pixel

    // Quick median: sort the filled portion and take the middle element.
    // For 3-9 elements the sort is essentially free.
    auto end = neighbour_depths.begin() + count;
    std::sort(neighbour_depths.begin(), end);
    float median = neighbour_depths[count / 2];

    return std::abs(depth_value - median) <= max_depth_diff;
}

}  // namespace seg_mask_roi_extractor