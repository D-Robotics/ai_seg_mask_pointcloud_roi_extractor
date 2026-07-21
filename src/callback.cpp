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
    void MaskPcRoiExtractor::parserDepthMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
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
                                        job->cfg.cam_w, job->cfg.cam_h, job->cfg.min_depth, job->cfg.max_depth,
                                        target_names_roi_confidence_);
        };

        if (thread_pool_)
        {
            // Add tasks to the thread pool and limit the maximum number of pending frames to avoid excessive memory growth
            const size_t max_pending = static_cast<size_t>(std::max(1, params_->max_pending_frames));
            thread_pool_->enqueue(task_filter, max_pending);
            thread_pool_->enqueue(task_roi, max_pending);
        }
        else
        {
            // Fallback (should not happen post-activate): run sequentially to stay correct.
            task_filter();
            task_roi();
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

    void MaskPcRoiExtractor::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
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

    sensor_msgs::msg::CameraInfo::SharedPtr MaskPcRoiExtractor::getCameraInfo()
    {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        return camera_info_;
    }


    void MaskPcRoiExtractor::classInfoCallback(const ai_msgs::msg::PerceptionInfo::SharedPtr msg)
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

    std::shared_ptr<const std::unordered_map<std::string, size_t>> MaskPcRoiExtractor::getClassNamesInfo()
    {
        std::lock_guard<std::mutex> lock(class_info_mutex_);
        // name: id
        return class_names_id_;
    }

} // namespace robot::mask_pc_roi_extractor
