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

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace
{
/**
 * @brief Declare a ROS2 parameter if it has not already been declared.
 */
void declare_parameter_if_not_declared(
    rclcpp::Node* node,
    const std::string& name,
    const rclcpp::ParameterValue& default_value)
{
    if (!node->has_parameter(name))
    {
        node->declare_parameter(name, default_value);
    }
    // RCLCPP_INFO(node->get_logger(), "  %s: %s",
    //             name.c_str(),
    //             node->get_parameter(name).value_to_string().c_str());
}

/**
 * @brief Get a ROS2 parameter value if it has been declared.
 *
 * Logs the value on success, or an error with a troubleshooting command
 * if the parameter has not been declared. The output value is left
 * unchanged on failure (the caller's default is preserved).
 */
template <typename T>
void get_parameter_if_declared(
    rclcpp::Node* node,
    const std::string& name,
    T& value)
{
    if (node->has_parameter(name))
    {
        node->get_parameter(name, value);
        RCLCPP_INFO(node->get_logger(), "  %s: %s",
                    name.c_str(),
                    node->get_parameter(name).value_to_string().c_str());
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(),
                     "  Parameter '%s' not declared! "
                     "Check with: ros2 param list | grep %s",
                     name.c_str(), name.c_str());
    }
}

}  // anonymous namespace


namespace robot::ai_seg_mask_pointcloud_roi_extractor
{

/**
 * @brief Parameters for depth mask extraction algorithm
 * Contains configuration parameters for depth-based mask extraction,
 * including confidence thresholds and target classes for segmenting
 * depth images into different regions of interest.
 */
struct MaskPcRoiExtractorPara
{
    /**
     * @brief Shared pointer type for MaskPcRoiExtractorPara
     */
    using Ptr = std::shared_ptr<MaskPcRoiExtractorPara>;

    bool        debug{false};                           /**< Enable debug mode */
    std::string depth_image_topic{""};                /**< Depth image topic name */
    std::string detect_info_topic{""};                /**< Detection info topic name */
    std::string class_info_topic{""};                 /**< Class info topic name */
    std::string camera_info_topic{""};                /**< Camera info topic name */
    std::string color_image_topic{""};                /**< Color image topic name */
    std::string filtered_mask_topic{""};              /**< Filtered mask topic name */
    std::string filtered_depth_topic{""};             /**< Filtered depth topic name */
    std::string filtered_cloud_topic{""};             /**< Filtered point cloud topic name */
    std::string roi_cloud_topic{""};                  /**< ROI point cloud topic name */
    std::string roi_visual_topic{""};                 /**< ROI overlay visual image topic name (bgr8 depth+segment overlay) */
    std::string confidence_threshold_file_path{""};   /**< Confidence threshold config file path (depth filtering) */
    std::string confidence_threshold_roi_file_path{""};  /**< Confidence threshold config file path (ROI extraction) */
    int         queue_size{10};                       /**< Message subscription queue size */
    double      allow_timestamp_deviation{1.0};       /**< Allowed timestamp deviation for message sync (seconds) */
    double      min_depth{0.0};                       /**< Minimum depth value for filtering (meters) */
    double      max_depth{3.0};                       /**< Maximum depth value for filtering (meters) */
    int         dilate_iter_num{1};                   /**< Mask dilation iterations */
    int         erode_iter_num_roi{1};              /**< ROI mask erosion iterations (0=disabled). KEEP LOW: the eroded mask is written back to seg_mask_class_id and feeds the depth-filter/obstacle pipeline; >=3 made costmap obstacles flicker and navigation choppy (2026-09-14 regression). For per-class tightening use erode_extra_class_names instead (ROI/semantic-map path only). */
    std::string erode_extra_class_names{"chair"};   /**< Comma-separated class names receiving EXTRA erosion on seg_mask_class_id only (mask_img untouched → depth-filter/obstacle pipeline unaffected). String (not array): the descriptions-yaml launch tooling only supports bool/int/double/string types. Only list classes NOT in the depth-filter config (model_classes_config.yaml, currently person-only); otherwise the obstacle mask would shrink too (BUG-20260914). */
    int         erode_extra_iter_num{2};            /**< Extra erosion iterations for erode_extra_class_names (0=disabled). 2 = chair total boundary shrink matches the empirically tuned global erode_iter_num_roi=3 that fixed semantic-map blob inflation. */
    bool        depth_continuity_check{false};      /**< Enable depth continuity check (3x3 same-class neighborhood depth median) for ROI point cloud. Default OFF (reverted 2026-09-14 after field test): the local check cannot see coherent single-frame tail streaks (internally depth-consistent), and isolated flying pixels are already covered by the instance-level percentile gate (instance_depth_gate_tau) — keeping it ON only added 2~9 ms/frame. Retained as a debug switch. ROI/semantic-map chain only. */
    double      max_depth_diff{0.3};               /**< Max allowed depth difference from neighbor median (meters) */
    double      instance_depth_gate_tau{0.3};      /**< Per-instance ADAPTIVE percentile depth gate minimum margin (meters, 0=disabled). Keep-band = [p10 - m, p90 + m], m = max(tau, p90-p10): band scales with the instance's own depth span, so large objects keep their far edges while coherent radial tail streaks (background bleed, internally depth-consistent → invisible to the local 3x3 continuity check, user-tested 2026-09-14) are cut entirely. ROI/semantic-map chain only. */
    int         camera_width{640};                    /**< Camera output image width (pixels) */
    int         camera_height{352};                   /**< Camera output image height (pixels) */
    std::string log_level{"info"};                    /**< Log level (debug/info/warn/error) */
    bool        use_extractor{false};                 /**< true=extract ROI, false=filter ROI */
    int         worker_threads{3};                    /**< Persistent worker thread count (max 3) */
    int         max_pending_frames{2};                /**< Max pending frames in processing queue */

    /** @brief Default constructor */
    MaskPcRoiExtractorPara() = default;

    /** @brief Create a completely independent deep copy (value semantics) */
    MaskPcRoiExtractorPara clone() const { return *this; }

    /** @brief Create a shared_ptr deep copy (shared ownership) */
    Ptr cloneShared() const { return std::make_shared<MaskPcRoiExtractorPara>(*this); }

    /** @brief Parameterized constructor with all config fields */  
    MaskPcRoiExtractorPara(
        bool debug,
        std::string depth_image_topic,
        std::string detect_info_topic,
        std::string class_info_topic,
        std::string camera_info_topic,
        std::string color_image_topic,
        std::string filtered_mask_topic,
        std::string filtered_depth_topic,
        std::string filtered_cloud_topic,
        std::string roi_cloud_topic,
        std::string roi_visual_topic,
        std::string confidence_threshold_file_path,
        int queue_size,
        double allow_timestamp_deviation,
        double min_depth,
        double max_depth,
        int dilate_iter_num,
        int erode_iter_num_roi,
        bool depth_continuity_check,
        double max_depth_diff,
        int camera_width,
        int camera_height,
        std::string log_level,
        bool use_extractor,
        int worker_threads,
        int max_pending_frames
    ) :
        debug(debug),
        depth_image_topic(depth_image_topic),
        detect_info_topic(detect_info_topic),
        class_info_topic(class_info_topic),
        camera_info_topic(camera_info_topic),
        color_image_topic(color_image_topic),
        filtered_mask_topic(filtered_mask_topic),
        filtered_depth_topic(filtered_depth_topic),
        filtered_cloud_topic(filtered_cloud_topic),
        roi_cloud_topic(roi_cloud_topic),
        roi_visual_topic(roi_visual_topic),
        confidence_threshold_file_path(confidence_threshold_file_path),
        queue_size(queue_size),
        allow_timestamp_deviation(allow_timestamp_deviation),
        min_depth(min_depth),
        max_depth(max_depth),
        dilate_iter_num(dilate_iter_num),
        erode_iter_num_roi(erode_iter_num_roi),
        depth_continuity_check(depth_continuity_check),
        max_depth_diff(max_depth_diff),
        camera_width(camera_width),
        camera_height(camera_height),
        log_level(log_level),
        use_extractor(use_extractor),
        worker_threads(worker_threads),
        max_pending_frames(max_pending_frames) {}

    /** @brief Declare all parameters on the given ROS2 lifecycle node */
    void declare_parameters(rclcpp::Node* node)
    {
        declare_parameter_if_not_declared(node, "debug", rclcpp::ParameterValue(false));
        declare_parameter_if_not_declared(node, "depth_image_topic", rclcpp::ParameterValue("image_raw"));
        declare_parameter_if_not_declared(node, "detect_info_topic", rclcpp::ParameterValue("hobot_dnn_detection"));
        declare_parameter_if_not_declared(node, "class_info_topic", rclcpp::ParameterValue("hobot_dnn_detection_info"));
        declare_parameter_if_not_declared(node, "camera_info_topic", rclcpp::ParameterValue("camera_info"));
        declare_parameter_if_not_declared(node, "color_image_topic", rclcpp::ParameterValue("image_raw"));
        declare_parameter_if_not_declared(node, "filtered_mask_topic", rclcpp::ParameterValue("filtered_mask"));
        declare_parameter_if_not_declared(node, "filtered_depth_topic", rclcpp::ParameterValue("filtered_depth_img"));
        declare_parameter_if_not_declared(node, "filtered_cloud_topic", rclcpp::ParameterValue("filtered_depth_cloud"));
        declare_parameter_if_not_declared(node, "roi_cloud_topic", rclcpp::ParameterValue("roi_pointclouds"));
        declare_parameter_if_not_declared(node, "roi_visual_topic", rclcpp::ParameterValue(""));
        declare_parameter_if_not_declared(node, "confidence_threshold_file_path", rclcpp::ParameterValue("model_classes_config.yaml"));
        declare_parameter_if_not_declared(node, "confidence_threshold_roi_file_path", rclcpp::ParameterValue("model_classes_roi_config.yaml"));
        declare_parameter_if_not_declared(node, "queue_size", rclcpp::ParameterValue(10));
        declare_parameter_if_not_declared(node, "allow_timestamp_deviation", rclcpp::ParameterValue(1.0));
        declare_parameter_if_not_declared(node, "min_depth", rclcpp::ParameterValue(0.0));
        declare_parameter_if_not_declared(node, "max_depth", rclcpp::ParameterValue(5.0));
        declare_parameter_if_not_declared(node, "dilate_iter_num", rclcpp::ParameterValue(1));
        declare_parameter_if_not_declared(node, "erode_iter_num_roi", rclcpp::ParameterValue(1));
        declare_parameter_if_not_declared(node, "erode_extra_class_names", rclcpp::ParameterValue(std::string("chair")));
        declare_parameter_if_not_declared(node, "erode_extra_iter_num", rclcpp::ParameterValue(2));
        declare_parameter_if_not_declared(node, "depth_continuity_check", rclcpp::ParameterValue(false));
        declare_parameter_if_not_declared(node, "max_depth_diff", rclcpp::ParameterValue(0.3));
        declare_parameter_if_not_declared(node, "instance_depth_gate_tau", rclcpp::ParameterValue(0.3));
        declare_parameter_if_not_declared(node, "camera_width", rclcpp::ParameterValue(640));
        declare_parameter_if_not_declared(node, "camera_height", rclcpp::ParameterValue(352));
        declare_parameter_if_not_declared(node, "log_level", rclcpp::ParameterValue("info"));
        declare_parameter_if_not_declared(node, "use_extractor", rclcpp::ParameterValue(false));
        declare_parameter_if_not_declared(node, "worker_threads", rclcpp::ParameterValue(3));
        declare_parameter_if_not_declared(node, "max_pending_frames", rclcpp::ParameterValue(2));
    }

    void get_parameters(rclcpp::Node* node)
    {
        get_parameter_if_declared(node, "debug", this->debug);
        get_parameter_if_declared(node, "depth_image_topic", this->depth_image_topic);
        get_parameter_if_declared(node, "detect_info_topic", this->detect_info_topic);
        get_parameter_if_declared(node, "class_info_topic", this->class_info_topic);
        get_parameter_if_declared(node, "camera_info_topic", this->camera_info_topic);
        get_parameter_if_declared(node, "color_image_topic", this->color_image_topic);
        get_parameter_if_declared(node, "filtered_mask_topic", this->filtered_mask_topic);
        get_parameter_if_declared(node, "filtered_depth_topic", this->filtered_depth_topic);
        get_parameter_if_declared(node, "filtered_cloud_topic", this->filtered_cloud_topic);
        get_parameter_if_declared(node, "roi_cloud_topic", this->roi_cloud_topic);
        get_parameter_if_declared(node, "roi_visual_topic", this->roi_visual_topic);
        get_parameter_if_declared(node, "confidence_threshold_file_path", this->confidence_threshold_file_path);
        get_parameter_if_declared(node, "confidence_threshold_roi_file_path", this->confidence_threshold_roi_file_path);
        get_parameter_if_declared(node, "queue_size", this->queue_size);
        get_parameter_if_declared(node, "allow_timestamp_deviation", this->allow_timestamp_deviation);
        get_parameter_if_declared(node, "min_depth", this->min_depth);
        get_parameter_if_declared(node, "max_depth", this->max_depth);
        get_parameter_if_declared(node, "dilate_iter_num", this->dilate_iter_num);
        get_parameter_if_declared(node, "erode_iter_num_roi", this->erode_iter_num_roi);
        get_parameter_if_declared(node, "erode_extra_class_names", this->erode_extra_class_names);
        get_parameter_if_declared(node, "erode_extra_iter_num", this->erode_extra_iter_num);
        get_parameter_if_declared(node, "depth_continuity_check", this->depth_continuity_check);
        get_parameter_if_declared(node, "max_depth_diff", this->max_depth_diff);
        get_parameter_if_declared(node, "instance_depth_gate_tau", this->instance_depth_gate_tau);
        get_parameter_if_declared(node, "camera_width", this->camera_width);
        get_parameter_if_declared(node, "camera_height", this->camera_height);
        get_parameter_if_declared(node, "log_level", this->log_level);
        get_parameter_if_declared(node, "use_extractor", this->use_extractor);
        get_parameter_if_declared(node, "worker_threads", this->worker_threads);
        get_parameter_if_declared(node, "max_pending_frames", this->max_pending_frames);
    }
};

}  // namespace robot::ai_seg_mask_pointcloud_roi_extractor
