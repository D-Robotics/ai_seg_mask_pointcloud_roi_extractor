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

#include <sstream>

namespace seg_mask_roi_extractor
{
    void AISegMaskPointCloudROIExtractor::parserDepthMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                                                    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &detect_info_msg)
    {
        // Check if messages are valid
        if (!depth_msg || !detect_info_msg)
        {
            RCLCPP_ERROR(get_logger(), "Invalid depth or detect info message received!");
            return;
        }
        std::chrono::high_resolution_clock::time_point start, end;
        start = std::chrono::high_resolution_clock::now();
        if (params_->debug)
        {
            rclcpp::Time now = this->get_clock()->now();
            rclcpp::Time depth_time(depth_msg->header.stamp);
            rclcpp::Time detect_time(detect_info_msg->header.stamp);
            rclcpp::Duration depth_delay = now - depth_time;
            rclcpp::Duration detect_info_delay = now - detect_time;
            rclcpp::Duration msg_time_diff = depth_time - detect_time;
            RCLCPP_DEBUG(get_logger(), "Received synchronized depth and mask messages");
            RCLCPP_DEBUG(get_logger(), "Depth Msg timestamp: %d.%09u",
                                        depth_msg->header.stamp.sec, 
                                        depth_msg->header.stamp.nanosec);
            RCLCPP_DEBUG(get_logger(), "Detect-Info Msg timestamp: %d.%09u",
                                        detect_info_msg->header.stamp.sec, 
                                        detect_info_msg->header.stamp.nanosec);
            RCLCPP_DEBUG(get_logger(), "Depth Msg publish-subscribe delay: %.3f ms (%.6f s)",
                                        depth_delay.nanoseconds() / 1000000.0, 
                                        depth_delay.seconds());
            RCLCPP_DEBUG(get_logger(), "Detect-Info Msg publish-subscribe delay: %.3f ms (%.6f s)",
                                        detect_info_delay.nanoseconds() / 1000000.0, 
                                        detect_info_delay.seconds());
            RCLCPP_DEBUG(get_logger(), "Depth-Mask Msg time difference: %.3f ms (%.6f s)",
                                        std::llabs(msg_time_diff.nanoseconds()) / 1000000.0,
                                        std::abs(msg_time_diff.seconds()));
        }

        double time_stamp = headerTimeStampTimeToDoubleSec(depth_msg->header);
        std::string frame_id = depth_msg->header.frame_id;

        // Per-frame time-sync bookkeeping (drives timeSyncStatus() stall warnings).
        if (last_time_ == -std::numeric_limits<double>::infinity())
        {
            last_time_ = time_stamp;
        }
        else
        {
            double time_diff = std::abs(time_stamp - last_time_);
            if (time_diff > sync_time_delta_)
            {
                RCLCPP_WARN(get_logger(), "The delay of time synchronization between the depth map and mask exceeds the set threshold, current_time=%f(s), last_time=%f(s), delta=%f(s)",
                            time_stamp, last_time_, time_diff);
            }
            last_time_ = time_stamp;
            last_receive_time_ = rclcpp::Time(depth_msg->header.stamp);
        }

        // 0. Guard: class info and camera info are received asynchronously via
        // separate subscribers and may not have arrived on the first few frames.
        // Skip silently until they are ready — avoids log spam and pool leaks.
        if (!getClassNamesInfo() || getClassNamesInfo()->empty())
            return;
        if (!getCameraInfo())
            return;

        // 1. Parse Depth Map (zero-copy share of the message buffer)
        cv_bridge::CvImageConstPtr depth_cv;
        if (!parserImage(depth_msg, depth_cv))
        {
            RCLCPP_ERROR(get_logger(), "Depth map parsing failed!");
            return;
        }

        // 2. Parse detection information: generates binary mask_img + class-ID seg_mask + box infos
        //    mask_img / seg_mask_class_id are borrowed from the reusable Mat pool.
        cv::Mat mask_img;
        cv::Mat seg_mask_class_id;
        std::vector<BoxInfo> filtered_all_box_info;
        if (!parserDetectInfo(detect_info_msg,
                            mask_img,
                            filtered_all_box_info,
                            seg_mask_class_id))
        {
            RCLCPP_ERROR(get_logger(), "Detect info parsing failed!");
            return;
        }

        // 2.5 Erode the class-ID segmentation mask to shrink object boundaries
        //     inward.  A 21x21 elliptical kernel removes ~10 px of edge
        //     artifacts in a single pass.
        if (!seg_mask_class_id.empty())
        {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(21, 21));
            cv::erode(seg_mask_class_id, seg_mask_class_id, kernel, cv::Point(-1, -1), 1);
        }

        // ==================== Dispatch to thread pool (non-blocking) ====================
        // Build a self-contained job whose data outlives this callback, then enqueue two
        // independent tasks (filter stage / ROI stage) that truly run in parallel on
        // persistent worker threads. The callback returns immediately so the executor
        // keeps receiving frames -> higher effective frame rate.
        auto job = std::make_shared<FrameJob>();
        job->depth_cv = depth_cv;
        job->mask_img = std::move(mask_img);
        job->seg_mask_class_id = std::move(seg_mask_class_id);
        job->boxes = std::move(filtered_all_box_info);
        job->frame_id = frame_id;
        job->timestamp = time_stamp;
        job->stamp = depth_msg->header.stamp;
        job->mask_pool = &mask_pool_;

        // Snapshot the parameters needed by the worker tasks HERE, on the (serialized)
        // callback thread, so the worker threads read immutable values instead of racing
        // with cameraInfoCallback / onParameterChange on the executor thread.
        job->cfg.debug = params_->debug;
        job->cfg.cam_w = static_cast<int>(params_->camera_width);
        job->cfg.cam_h = static_cast<int>(params_->camera_height);
        job->cfg.min_depth = static_cast<float>(params_->min_depth);
        job->cfg.max_depth = static_cast<float>(params_->max_depth);
        job->cfg.use_extractor = params_->use_extractor;

        // Capture publisher shared_ptrs to keep them alive during async processing,
        // preventing use-after-free if lifecycle transitions reset member publishers concurrently.
        auto fp  = filtered_depth_pub_;
        auto fmp = filtered_depth_mask_pub_;
        auto fcp = filtered_cloud_pub_;
        auto rcp = roi_cloud_pub_;
        auto rvp = roi_visual_pub_;

        // Task 1: filtered depth/mask/cloud publish (operates on its own depth clone)
        auto task_filter = [this, job, fp, fmp, fcp]()
        {
            ScopedTimer t(timing_filter_.acc_us, timing_filter_.count, job->cfg.debug);
            // If seg mask is available, further restrict mask_img to depth-specific
            // classes (parsing stage keeps all merged classes).  This is the depth
            // pipeline's own per-pixel class filter.
            if (!job->seg_mask_class_id.empty() && target_class_ids_ && !target_class_ids_->empty())
            {
                cv::Mat& mask = job->mask_img;
                const cv::Mat& seg = job->seg_mask_class_id;
                for (int v = 0; v < mask.rows; ++v)
                {
                    uchar* mrow = mask.ptr<uchar>(v);
                    const uchar* srow = seg.ptr<uchar>(v);
                    for (int u = 0; u < mask.cols; ++u)
                    {
                        if (mrow[u] == 255 && target_class_ids_->count(srow[u]) == 0)
                            mrow[u] = 0;
                    }
                }
            }
            // Guard: when no detection targets exist (e.g. empty frame), mask_img
            // is empty.  Skip morphology to avoid OpenCV dereferencing a null
            // data pointer, but continue to processFilteredPublish which handles
            // empty masks gracefully (no-op setDepthZeroByMask).
            if (!job->mask_img.empty())
            {
                // Depth pipeline mask strategy:
                //   filtering (use_extractor=false): dilate (expand mask to cover more context)
                //   extraction (use_extractor=true):  erode  (shrink mask for precise extraction)
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
                if (!job->cfg.use_extractor)
                {
                    cv::dilate(job->mask_img, job->mask_img, kernel, cv::Point(-1, -1),
                               params_->dilate_iter_num);
                }
                else
                {
                    cv::erode(job->mask_img, job->mask_img, kernel, cv::Point(-1, -1),
                              params_->dilate_iter_num);
                }
            }
            const int h = job->depth_cv->image.rows;
            const int w = job->depth_cv->image.cols;
            cv::Mat depth_clone = depth_pool_.acquire(h, w, CV_16UC1);
            job->depth_cv->image.copyTo(depth_clone);
            this->processFilteredPublish(depth_clone, job->mask_img, job->timestamp, job->frame_id,
                                         job->cfg.use_extractor, job->cfg.min_depth, job->cfg.max_depth, job->cfg.debug,
                                         fp, fmp, fcp);
            depth_pool_.release(depth_clone);
        };

        // Task 2: per-ROI point cloud extraction with conf and id (read-only depth)
        auto task_roi = [this, job, rcp]()
        {
            ScopedTimer t(timing_roi_.acc_us, timing_roi_.count, job->cfg.debug);
            this->processROIPointClouds(job->depth_cv->image, job->seg_mask_class_id,
                                        job->boxes, job->timestamp, job->frame_id, rcp,
                                        job->cfg.cam_w, job->cfg.cam_h,
                                        job->cfg.min_depth, job->cfg.max_depth,
                                        target_names_roi_confidence_);
        };

        // Task 3: ROI visual overlay render (bgr8 depth + segment). Only when a
        // publisher is configured. Uses the same ROI filtering as task_roi so the
        // overlay matches the semantic info written to the map.
        auto task_visual = [this, job, rvp]()
        {
            if (!rvp) return;
            this->processROIVisual(job->depth_cv->image, job->seg_mask_class_id,
                                   job->boxes, job->timestamp, job->frame_id, rvp,
                                   job->cfg.cam_w, job->cfg.cam_h,
                                   job->cfg.min_depth, job->cfg.max_depth,
                                   target_names_roi_confidence_);
        };

        if (thread_pool_)
        {
            // Add tasks to the thread pool and limit the maximum number of pending frames to avoid excessive memory growth
            const size_t max_pending = static_cast<size_t>(std::max(1, params_->max_pending_frames));
            thread_pool_->enqueue(task_filter, max_pending);
            thread_pool_->enqueue(task_roi, max_pending);
            thread_pool_->enqueue(task_visual, max_pending);
        }
        else
        {
            // Fallback (should not happen post-activate): run sequentially to stay correct.
            task_filter();
            task_roi();
            task_visual();
        }

        end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        RCLCPP_DEBUG(get_logger(), "[parserDepthMaskCallback()] Parse+Dispatch Time = %.3f ms",
                    duration.count() / 1000.0);

        // Periodic timing report for worker tasks (every N frames, debug only).
        if (params_->debug && ++frame_counter_ >= timing_report_interval_)
        {
            RCLCPP_INFO(get_logger(),
                        "[timing] last %d frames avg: filter=%.3f ms (%d samples), roi=%.3f ms (%d samples)",
                        frame_counter_,
                        timing_filter_.avg_ms(), timing_filter_.count.load(),
                        timing_roi_.avg_ms(),    timing_roi_.count.load());
            timing_filter_.reset();
            timing_roi_.reset();
            frame_counter_ = 0;
        }

        return;
    }

    void AISegMaskPointCloudROIExtractor::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        if(!msg)
        {
            RCLCPP_ERROR(get_logger(), "Invalid camera info message received");
            return;
        }
        camera_info_ = msg;
        params_->camera_height = static_cast<int>(msg->height);
        params_->camera_width = static_cast<int>(msg->width);

        // Log camera info received
        RCLCPP_INFO(get_logger(), "Camera info received with frame_id: %s", camera_info_->header.frame_id.c_str());
        RCLCPP_INFO(get_logger(), "Camera Height = %d, Width = %d", params_->camera_height, params_->camera_width);
        RCLCPP_INFO(get_logger(), "Camera intrinsic matrix:");
        RCLCPP_INFO(get_logger(), "  fx: %.4f, fy: %.4f, cx: %.4f, cy: %.4f",
                                    camera_info_->k[0],
                                    camera_info_->k[4],
                                    camera_info_->k[2],
                                    camera_info_->k[5]);

        // Unsubscribe from camera info topic after receiving the first message
        camera_info_sub_.reset();

        return;
    }

    sensor_msgs::msg::CameraInfo::SharedPtr AISegMaskPointCloudROIExtractor::getCameraInfo()
    {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        return camera_info_;
    }


    void AISegMaskPointCloudROIExtractor::classInfoCallback(const ai_msgs::msg::PerceptionInfo::SharedPtr msg)
    {
        if(!msg)
        {
            RCLCPP_ERROR(get_logger(), "Invalid class names message received");
            return;
        }
        RCLCPP_INFO(get_logger(), "Class_Info_Callback W:%d H:%d Class_Size:%ld", msg->width, msg->height, msg->class_names.size());

        // Build the immutable dictionary once, then publish it as a shared_ptr snapshot
        // so readers take a cheap pointer copy (no per-frame deep copy of the strings).
        // name: id
        auto local_map = std::make_shared<std::unordered_map<std::string, size_t>>();
        for (size_t idx = 0; idx < msg->class_names.size(); idx++)
        {
            // name: id
            (*local_map)[msg->class_names[idx]] = idx;
        }
        if (local_map->empty())
        {
            RCLCPP_WARN(get_logger(), "class_names_id_ is empty!");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(class_info_mutex_);
            class_names_id_ = std::move(local_map);
        }
        RCLCPP_INFO(get_logger(), "Model_Class_Info : ");
        for (const auto& pair : *class_names_id_)
        {
            RCLCPP_INFO(get_logger(), "----------Name: %s  --> ID: %zu", pair.first.c_str(), pair.second);
        }
        // Rebuild the precomputed target class-ID set now that both dictionaries exist.
        rebuildTargetClassIdSet();
        // Unsubscribe from class names info topic after receiving the first message
        class_info_sub_.reset();
        return;

    }

    std::shared_ptr<const std::unordered_map<std::string, size_t>> AISegMaskPointCloudROIExtractor::getClassNamesInfo()
    {
        std::lock_guard<std::mutex> lock(class_info_mutex_);
        // name: id
        return class_names_id_;
    }


    // ══════════════════════════════════════════════════════════════
    // Detection-message parsing (merged back from parsing.cpp to match
    // the upstream single-file callback.cpp layout)
    // ══════════════════════════════════════════════════════════════
// ══════════════════════════════════════════════════════════════════════
// parserImage
// ══════════════════════════════════════════════════════════════════════

bool AISegMaskPointCloudROIExtractor::parserImage(
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

bool AISegMaskPointCloudROIExtractor::parserImage(
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

bool AISegMaskPointCloudROIExtractor::parserDetectInfo(
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
                        &detect_valid_area_mask,       // write mask for depth filtering
                        &filtered_all_box_info);       // collect boxes for ROI extraction

    // ── 3. Intersect target mask ∩ bbox union ────────────────────────
    if (has_seg_mask && !mask_valid.empty())
    {
        mask_img = mask_pool_.acquire(img_h, img_w, CV_8UC1);
        cv::bitwise_and(mask_valid, detect_valid_area_mask, mask_img);

        // ── 4. Erode mask_img → write back to seg_mask_class_id ──
        // Shrink the bbox-constrained mask boundary to remove unreliable
        // edge pixels before ROI point cloud extraction.  Eroding in-place
        // on mask_img (1 iteration = 1 pixel) has negligible effect on
        // task_filter's subsequent dilate/erode.
        const int erode_n = params_->erode_iter_num_roi;
        if (erode_n > 0)
        {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::erode(mask_img, mask_img, kernel, cv::Point(-1, -1), erode_n);

            // Write back: keep seg_mask_class_id only where erode output is 255
            for (int v = 0; v < img_h; ++v)
            {
                const uchar* mrow = mask_img.ptr<uchar>(v);
                uchar* srow = seg_mask_class_id.ptr<uchar>(v);
                for (int u = 0; u < img_w; ++u)
                {
                    if (mrow[u] != 255) srow[u] = 0;
                }
            }
        }

        // ── 4b. Per-class EXTRA erosion on seg_mask_class_id ─────────
        // Tightens only the configured classes (default: chair) to fight
        // semantic-map blob inflation.  mask_img is NOT touched, so the
        // depth-filter/obstacle pipeline is unaffected — safe as long as the
        // listed classes are absent from the depth-filter config
        // (model_classes_config.yaml, currently person-only).  Raising the
        // GLOBAL erode_iter_num_roi instead caused navigation jank
        // (BUG-20260914); this is the class-differentiated replacement.
        const int extra_n = params_->erode_extra_iter_num;
        if (extra_n > 0 && !params_->erode_extra_class_names.empty())
        {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::Mat sub = mask_pool_.acquire(img_h, img_w, CV_8UC1);
            // Comma-separated list (plain string param — the descriptions-yaml
            // launch tooling only supports bool/int/double/string types).
            std::stringstream ss(params_->erode_extra_class_names);
            std::string name;
            while (std::getline(ss, name, ','))
            {
                const size_t b = name.find_first_not_of(" \t");
                if (b == std::string::npos) continue;
                name = name.substr(b, name.find_last_not_of(" \t") - b + 1);
                auto it = class_names->find(name);
                if (it == class_names->end())
                {
                    RCLCPP_WARN_ONCE(get_logger(),
                                     "erode_extra_class_names: '%s' not in model class list, skipped",
                                     name.c_str());
                    continue;
                }
                // Pixel values in seg_mask_class_id are class-id + 1 (0 = background)
                const uchar cid = static_cast<uchar>(it->second + 1);
                cv::compare(seg_mask_class_id, cid, sub, cv::CMP_EQ);
                if (cv::countNonZero(sub) == 0) continue;  // class absent this frame
                cv::erode(sub, sub, kernel, cv::Point(-1, -1), extra_n);
                // Drop this class's pixels that did not survive the extra erosion
                for (int v = 0; v < img_h; ++v)
                {
                    const uchar* srow = sub.ptr<uchar>(v);
                    uchar* grow = seg_mask_class_id.ptr<uchar>(v);
                    for (int u = 0; u < img_w; ++u)
                    {
                        if (grow[u] == cid && srow[u] == 0) grow[u] = 0;
                    }
                }
            }
            mask_pool_.release(sub);
        }

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

    // ══════════════════════════════════════════════════════════════
    // Depth projection utilities (merged back from depth_utils.cpp)
    // ══════════════════════════════════════════════════════════════
    bool AISegMaskPointCloudROIExtractor::convertDepthToPointcloud(const cv::Mat& depth,
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

    bool AISegMaskPointCloudROIExtractor::setDepthZeroByMask(const cv::Mat &mask, 
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

    bool AISegMaskPointCloudROIExtractor::generateMaskByDepth(const cv::Mat& depth_img, cv::Mat& mask)
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


} // namespace seg_mask_roi_extractor
