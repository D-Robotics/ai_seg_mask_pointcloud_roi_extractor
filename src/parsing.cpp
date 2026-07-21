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
// ══════════════════════════════════════════════════════════════════════
// parserImage
// ══════════════════════════════════════════════════════════════════════

bool MaskPcRoiExtractor::parserImage(
    const sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    cv_bridge::CvImageConstPtr &image_cv)
{
    if (!image_msg)
    {
        RCLCPP_ERROR(get_logger(), "Null image message received");
        return false;
    }
    try
    {
        // cv_bridge handles all common encodings (MONO8/16, BGR8, 32FC1, etc.)
        // and throws on unsupported ones — no manual whitelist needed.
        image_cv = cv_bridge::toCvShare(image_msg, image_msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return false;
    }
    const cv::Mat& img = image_cv->image;
    if (img.rows != static_cast<int>(params_->camera_height) ||
        img.cols != static_cast<int>(params_->camera_width))
    {
        RCLCPP_ERROR(get_logger(),
                     "Image size mismatch: got %dx%d, expected %dx%d",
                     img.cols, img.rows,
                     static_cast<int>(params_->camera_width),
                     static_cast<int>(params_->camera_height));
        return false;
    }
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// parserImage (compressed)
// ══════════════════════════════════════════════════════════════════════

bool MaskPcRoiExtractor::parserImage(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr image_msg,
    cv_bridge::CvImageConstPtr &image_cv)
{
    if (!image_msg)
    {
        RCLCPP_ERROR(get_logger(), "Null compressed image message received");
        return false;
    }
    try
    {
        // cv_bridge decodes JPEG/PNG transparently.
        image_cv = cv_bridge::toCvCopy(image_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception (compressed): %s", e.what());
        return false;
    }
    const cv::Mat& img = image_cv->image;
    if (img.rows != static_cast<int>(params_->camera_height) ||
        img.cols != static_cast<int>(params_->camera_width))
    {
        RCLCPP_ERROR(get_logger(),
                     "Image size mismatch: got %dx%d, expected %dx%d",
                     img.cols, img.rows,
                     static_cast<int>(params_->camera_width),
                     static_cast<int>(params_->camera_height));
        return false;
    }
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// Helper: parse segmentation mask → target-class binary mask
// ══════════════════════════════════════════════════════════════════════

/**
 * @brief Parse the semantic segmentation mask and filter by target class IDs.
 *
 * The mask arrives as a 4×-downsampled CV_32FC1 buffer inside the @p mask_label
 * capture.  It is upsampled to full resolution and then filtered in a single
 * pass: only pixels whose (+1-encoded) class ID belongs to @p target_class_ids
 * are set to 255 in @p mask_valid.
 *
 * Also runs a one-time self-check diagnostic to validate the encoding convention.
 *
 * @param[out] seg_mask_class_id  Full-resolution class-ID mask (for per-ROI extraction).
 * @param[out] mask_valid         Binary mask filtered to target classes only.
 * @return true if a valid mask was produced, false otherwise (not an error).
 */
static bool parseSegmentationMask(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& msg,
    const std::string& mask_label,
    int img_w,
    int img_h,
    const std::shared_ptr<const std::unordered_set<int>>& target_class_ids,
    size_t class_count,
    MatPool& mask_pool,
    cv::Mat& seg_mask_class_id,
    cv::Mat& mask_valid)
{
    for (const auto& target : msg->targets)
    {
        if (target.type != mask_label) continue;
        if (target.captures.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("parserDetectInfo"),
                        "Seg mask target has no captures, skipping");
            continue;
        }
        const auto& captures = target.captures[0];

        cv::Mat seg_down(captures.img.height, captures.img.width, CV_32FC1);
        if (seg_down.total() != captures.features.size())
        {
            RCLCPP_ERROR(rclcpp::get_logger("parserDetectInfo"),
                         "Seg mask features size(%zu) != image size(%ld)",
                         captures.features.size(), seg_down.total());
            return false;
        }

        std::memcpy(seg_down.data, captures.features.data(),
                    captures.features.size() * sizeof(float));
        seg_down.convertTo(seg_down, CV_8UC1);

        // Upsample to full resolution (INTER_NEAREST preserves integer class IDs).
        seg_mask_class_id = mask_pool.acquire(img_h, img_w, CV_8UC1);
        cv::resize(seg_down, 
                seg_mask_class_id, 
                cv::Size(img_w, img_h),
                0, 0, cv::INTER_NEAREST);

        // ── Single-pass target filtering ────────────────────────
        mask_valid = mask_pool.acquire(img_h, img_w, CV_8UC1);
        mask_valid.setTo(0);
        if (target_class_ids && !target_class_ids->empty())
        {
            for (int v = 0; v < img_h; ++v)
            {
                const uchar* srow = seg_mask_class_id.ptr<uchar>(v);
                uchar*       mrow = mask_valid.ptr<uchar>(v);
                for (int u = 0; u < img_w; ++u)
                {
                    if (target_class_ids->count(static_cast<int>(srow[u])) > 0)
                        mrow[u] = 255;
                }
            }
        }
        // ── One-time self-check diagnostic ──────────────────────
        static bool diag_done = false;
        if (!diag_done)
        {
            diag_done = true;
            double mn = 0.0, mx = 0.0;
            cv::minMaxLoc(seg_mask_class_id, &mn, &mx);
            double mmx = 0.0;
            cv::minMaxLoc(mask_valid, nullptr, &mmx);
            RCLCPP_INFO(rclcpp::get_logger("parserDetectInfo"),
                        "[seg-mask] pixel range=[%.0f, %.0f], class_count=%zu, "
                        "targets matched=%s",
                        mn, mx, class_count, mmx > 0.0 ? "YES" : "NO");
        }

        return true;
    }
    return false;
}

// ══════════════════════════════════════════════════════════════════════
// Helper: parse bounding-box detections from PerceptionTargets
// ══════════════════════════════════════════════════════════════════════

/**
 * @brief Extract detection boxes (non-mask targets) with confidence filtering.
 *
 * Either or both of the output pointers may be nullptr — the corresponding
 * operation is skipped.  This lets the caller use the same function for
 * depth-filtering mask generation (depth config, mask output only) and for
 * ROI point-cloud extraction (ROI config, box list output only).
 *
 * When @p detect_valid_area_mask is non-null the caller must provide a
 * correctly-sized CV_8UC1 mat (validated inline).
 */
static void parseDetectionBoxes(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& msg,
    const std::string& mask_label,
    const std::unordered_map<std::string, size_t>& class_names,  // name:id (The ID starts from 0)
    const std::unordered_map<std::string, double>& target_conf,  // name: confidence
    int img_w,
    int img_h,
    cv::Mat* detect_valid_area_mask,   // nullable — when non-null, fill mask rects
    std::vector<BoxInfo>* filtered_boxes)  // nullable — when non-null, collect boxes
{
    if (detect_valid_area_mask)
    {
        if (detect_valid_area_mask->empty() ||
            detect_valid_area_mask->type() != CV_8UC1 ||
            detect_valid_area_mask->cols != img_w ||
            detect_valid_area_mask->rows != img_h)
        {
            RCLCPP_ERROR(rclcpp::get_logger("parserDetectInfo"),
                         "detect_valid_area_mask invalid: %dx%d type=%d (expected %dx%d CV_8UC1)",
                         detect_valid_area_mask->cols, detect_valid_area_mask->rows,
                         detect_valid_area_mask->type(), img_w, img_h);
            return;
        }
    }
    for (const auto& target : msg->targets)
    {
        if (target.type == mask_label) continue;
        // Determine whether this class is in the target class list; if not, skip it
        auto id_it = class_names.find(target.type);
        if (id_it == class_names.end())
        {
            RCLCPP_WARN(rclcpp::get_logger("parserDetectInfo"),
                        "Class '%s' not found in class_names, skipping",
                        target.type.c_str());
            continue;
        }
        size_t id = id_it->second;
        for (const auto& roi : target.rois)
        {
            double conf = roi.confidence;
            auto conf_it = target_conf.find(target.type);
            if (conf_it != target_conf.end() && conf < conf_it->second)
                continue;
            BoxInfo box;
            box.name       = target.type;
            box.id         = id;
            box.confidence = conf;
            box.x_offset   = roi.rect.x_offset;
            box.y_offset   = roi.rect.y_offset;
            box.width      = roi.rect.width;
            box.height     = roi.rect.height;

            // Clamp to image bounds — covers negative offsets and overflows.
            cv::Rect rect(box.x_offset, box.y_offset, box.width, box.height);
            rect &= cv::Rect(0, 0, img_w, img_h);
            if (rect.area() <= 0) continue;
            if (detect_valid_area_mask) (*detect_valid_area_mask)(rect).setTo(255);
            if (filtered_boxes)         filtered_boxes->push_back(box);
        }
    }
}

// ══════════════════════════════════════════════════════════════════════
// parserDetectInfo (orchestrator)
// ══════════════════════════════════════════════════════════════════════

bool MaskPcRoiExtractor::parserDetectInfo(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &detect_info_msg,
    cv::Mat &mask_img,
    std::vector<BoxInfo>& filtered_all_box_info,
    cv::Mat& seg_mask_class_id)
{
    filtered_all_box_info.clear();
    const int img_h = static_cast<int>(params_->camera_height);
    const int img_w = static_cast<int>(params_->camera_width);
    // name:id (The ID starts from 0)
    auto class_names = getClassNamesInfo();
    if (!class_names || class_names->empty())
    {
        RCLCPP_ERROR(get_logger(), "class_names_id is empty, no class info loaded!");
        return false;
    }
    if (detect_info_msg->targets.empty())
    {
        RCLCPP_WARN(get_logger(), "detect_info_msg->targets is empty!");
        return true;
    }
    // ── 1. Parse seg mask → target-class binary mask (merged config) ──
    cv::Mat mask_valid;
    bool has_seg_mask = parseSegmentationMask(
        detect_info_msg,
        Semantic_Segmentation_Mask_Label,
        img_w, img_h,
        target_class_ids_merged_,
        class_names ? class_names->size() : 0,
        mask_pool_,
        seg_mask_class_id,
        mask_valid);

    // ── 2. Parse detection boxes (merged config) → mask + boxes ──────
    cv::Mat detect_valid_area_mask = mask_pool_.acquire(img_h, img_w, CV_8UC1);
    detect_valid_area_mask.setTo(0);
    parseDetectionBoxes(detect_info_msg,
                        Semantic_Segmentation_Mask_Label,
                        *class_names,
                        target_names_merged_conf_,
                        img_w, img_h,
                        &detect_valid_area_mask,      // write mask for depth filtering
                        &filtered_all_box_info);       // collect boxes for ROI extraction

    // ── 3. Intersect target mask ∩ bbox union ────────────────────────
    if (has_seg_mask && !mask_valid.empty())
    {
        mask_img = mask_pool_.acquire(img_h, img_w, CV_8UC1);
        cv::bitwise_and(mask_valid, detect_valid_area_mask, mask_img);
        mask_pool_.release(mask_valid);
        mask_pool_.release(detect_valid_area_mask);
    }
    else
    {
        mask_img = std::move(detect_valid_area_mask);
        if (!mask_valid.empty()) mask_pool_.release(mask_valid);
    }
    return true;
}
} // namespace robot::mask_pc_roi_extractor
