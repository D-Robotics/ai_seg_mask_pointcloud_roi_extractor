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
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ai_msgs/msg/perception_targets.hpp"
#include "ai_msgs/msg/perception_info.hpp"
#include "mask_pc_roi_extractor/msg/roi_point_cloud.hpp"
#include "mask_pc_roi_extractor/msg/roi_point_clouds.hpp"
#include "mask_pc_roi_extractor/params.hpp"
#include "mask_pc_roi_extractor/internal_types.hpp"

namespace robot::mask_pc_roi_extractor
{

    /**
     * @class MaskPcRoiExtractor
     * @brief A node that extracts depth masks from segmentation masks and publishes ROI point clouds
     *
     * This node subscribes to both depth images and segmentation masks, aligns them by timestamp,
     * and extracts depth masks based on the mask information. Additionally, it extracts per-instance
     * ROI point clouds with confidence and ID information.
     */
    class MaskPcRoiExtractor : public rclcpp::Node
    {
        public:
            /**
             * @brief Constructor.  All initialisation (params, publishers, subscribers,
             *        message_filters synchronizer, thread pool) is done here.
             * @param options Node options
             */
            explicit MaskPcRoiExtractor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

            /**
             * @brief Destructor
             */
            virtual ~MaskPcRoiExtractor() = default;

            /**
             * @brief Initialize paramter
             * @return True if parsing is successful, false otherwise
             */
            bool initParam();

            /**
             * @brief Initialize the subscribers
             */
            void initializeSubscriber();

            /**
             * @brief Initialize the publishers
             */
            void initializePublisher();

            /**
             * @brief Parser class names parameter
             */
            bool parserClassnamesParam();

            /**
             * Check if single topics that need to be subscribed to exist
             * @param topic_list Topic name
             * @return bool Exist → true, otherwise → false
             */
            bool check_single_topic(const std::string &topic_name);

            /**
             * Check if all topics that need to be subscribed to exist
             * @param topic_list Topic List Vector
             * @return bool All Exist → true, otherwise → false
             */
            bool check_topic_list(const std::vector<std::string> &topic_list);

            /**
             * Check if all topics exist (silent — no per-topic error log).
             * Used for polling retries to avoid log spam.
             * @param topic_list Topic List Vector
             * @return bool All Exist → true, otherwise → false
             */
            bool check_topic_list_quiet(const std::vector<std::string> &topic_list);

            /**
             * @brief Get camera intrinsic parameters
             * @return Shared pointer to camera info, or nullptr if not available
             */
            sensor_msgs::msg::CameraInfo::SharedPtr getCameraInfo();

            /**
             * @brief Get class names and their corresponding IDs (lock-free snapshot)
             * @return Shared pointer to an immutable map (no per-frame deep copy)
             */
            std::shared_ptr<const std::unordered_map<std::string, size_t>> getClassNamesInfo();

            /**
             * @brief Convert double timestamp to rclcpp::Time
             * @param time_stamp Double timestamp
             * @return Corresponding rclcpp::Time
             */
            static rclcpp::Time timeStampDoubleToTime(double time_stamp);

            /**
             * @brief Convert rclcpp::Time to double timestamp in seconds
             * @param time_in rclcpp::Time to convert
             * @return Corresponding double timestamp in seconds
             */
            static double timeStampTimeToDoubleSec(const rclcpp::Time &time_in);

            /**
             * @brief Convert rclcpp::Time to double timestamp in nanoseconds
             * @param time_in rclcpp::Time to convert
             * @return Corresponding double timestamp in nanoseconds
             */
            static double timeStampTimeToDoubleNanoSec(const rclcpp::Time &time_in);

            /**
             * @brief Convert rclcpp::Time to double timestamp in milliseconds
             * @param time_in rclcpp::Time to convert
             * @return Corresponding double timestamp in milliseconds
             */
            static double timeStampTimeToDoubleMilliSec(const rclcpp::Time &time_in);

            /**
             * @brief Convert rclcpp::Time to string timestamp
             * @param time_in rclcpp::Time to convert
             * @return Corresponding string timestamp
             */
            static std::string timeStampTimeToString(const rclcpp::Time &time_in);

            /**
             * @brief Convert header timestamp to double timestamp in seconds
             * @param header Header message containing timestamp
             * @return Corresponding double timestamp in seconds
             */
            static double headerTimeStampTimeToDoubleSec(const std_msgs::msg::Header& header);

            /**
             * @brief Convert header timestamp to double timestamp in milliseconds
             * @param header Header message containing timestamp
             * @return Corresponding double timestamp in milliseconds
             */
            static double headerTimeStampTimeToDoubleMilliSec(const std_msgs::msg::Header& header);

            /**
             * @brief Convert header timestamp to double timestamp in nanoseconds
             * @param header Header message containing timestamp
             * @return Corresponding double timestamp in nanoseconds
             */
            static double headerTimeStampTimeToDoubleNanoSec(const std_msgs::msg::Header& header);

            /**
             * @brief Convert header timestamp to rclcpp::Time
             * @param header Header message containing timestamp
             * @return Corresponding rclcpp::Time
             */
            static rclcpp::Time headerTimeStampToTime(const std_msgs::msg::Header& header);

            /**
             * @brief Publish an image using ImageInfo structure
             * @param timestamp Timestamp for the message header
             * @param frame_id Coordinate frame ID for the message header
             * @param image Shared pointer to ImageInfo structure containing image data
             * @param publisher Shared pointer to the ROS2 image publisher
             */
            void pubImage(double timestamp,
                        const std::string& frame_id,
                        cv::Mat &image,
                        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher);

            /**
             * @brief Compression mode for published images.
             */
            enum class ImageCompression : uint8_t
            {
                kNone = 0,  /**< No compression (raw pixels) */
                kJpeg = 1,  /**< JPEG compression */
                kPng  = 2,  /**< PNG compression */
            };

            /** @brief Default JPEG quality (0–100). */
            static constexpr int kDefaultJpegQuality = 85;
            /** @brief Default PNG compression level (0–9). */
            static constexpr int kDefaultPngCompression = 3;

            /**
             * @brief Publish an image with optional compression.
             *
             * When @p compression is kNone (default), the image is published
             * as a raw sensor_msgs::Image.  When set to kJpeg or kPng, the
             * image is encoded in-memory and the resulting byte buffer is
             * published with the corresponding encoding string ("jpeg"/"png").
             *
             * @param timestamp   ROS timestamp for the header.
             * @param frame_id    Coordinate frame ID.
             * @param image       OpenCV image to publish.
             * @param publisher   ROS2 image publisher.
             * @param compression Compression mode (default: kNone).
             * @param quality     JPEG quality (1–100) or PNG level (0–9).
             *                    ≤0 means "use default".
             */
            void pubImageCompressed(
                double timestamp,
                const std::string& frame_id,
                cv::Mat &image,
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher,
                ImageCompression compression = ImageCompression::kNone,
                int quality = 0);

            /**
             * @brief Infer the ROS sensor_msgs encoding string from an OpenCV Mat type.
             * @param image  Input image.
             * @return Encoding string (e.g. "mono8", "bgr8", "mono16").
             */
            static std::string inferImageEncoding(const cv::Mat& image);

            /**
             * @brief Publish a PCL point cloud of any point type.
             *
             * Template implementation (see below).  Supports all standard
             * PCL point types: PointXYZ, PointXYZI, PointXYZRGB, etc.
             *
             * @param cloud     Shared pointer to the PCL cloud.
             * @param frame_id  Coordinate frame ID.
             * @param timestamp ROS timestamp.
             * @param publisher ROS2 publisher.
             */
            template <typename PointT>
            void pubPointCloud(
                const pcl::PointCloud<PointT>& cloud,
                const std::string& frame_id,
                double timestamp,
                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);

        private:
            /**
             * @brief Callback function for synchronized depth and mask messages
             * @param depth_msg Shared pointer to the depth image message
             * @param mask_msg Shared pointer to the mask image message
             */
            void parserDepthMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                                        const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &detect_info_msg);

            /**
             * @brief Process and publish filtered depth/mask/pointcloud (original functionality)
             * @param depth_img Depth image (will be modified in-place)
             * @param mask_img Binary mask image
             * @param timestamp Timestamp for publishing
             * @param frame_id Frame ID for publishing
             */
            void processFilteredPublish(cv::Mat& depth_img,
                                       const cv::Mat& mask_img,
                                       double timestamp,
                                       const std::string& frame_id,
                                       bool use_extractor,
                                       float min_depth,
                                       float max_depth,
                                       bool debug,
                                       rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_pub,
                                       rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_mask_pub,
                                       rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub);

            /**
             * @brief Process and publish per-ROI point clouds with confidence and ID
             * @param depth_img Original depth image (read-only)
             * @param seg_mask_class_id Full-resolution class-ID mask (CV_8UC1, pixel = class_id + 1)
             * @param filtered_all_box_info Vector of filtered box infos
             * @param timestamp Timestamp for publishing
             * @param frame_id Frame ID for publishing
             */
            void processROIPointClouds(const cv::Mat& depth_img,
                                      const cv::Mat& seg_mask_class_id,
                                      const std::vector<BoxInfo>& filtered_all_box_info,
                                      double timestamp,
                                      const std::string& frame_id,
                                      rclcpp::Publisher<::mask_pc_roi_extractor::msg::ROIPointClouds>::SharedPtr roi_cloud_pub,
                                      int img_width,
                                      int img_height,
                                      float min_depth,
                                      float max_depth,
                                      const std::unordered_map<std::string, double>& roi_conf);

            /**
             * @brief Parse depth image message into a cv::Mat (zero-copy share of message buffer)
             * @param depth_msg Shared pointer to the depth image message
             * @param depth_cv Output CvImageConstPtr sharing the message buffer (read-only)
             * @return True if parsing is successful, false otherwise
             */
            bool parserImage(const sensor_msgs::msg::Image::ConstSharedPtr image_msg,
                            cv_bridge::CvImageConstPtr &image_cv);

            bool parserImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr image_msg,
                            cv_bridge::CvImageConstPtr &image_cv);

            /**
             * @brief Parse detection info message into a cv::Mat mask and extract box infos
             * @param detect_info_msg Shared pointer to the detection info message
             * @param mask_img Output pooled cv::Mat for the binary intersection mask
             * @param filtered_all_box_info Output vector of filtered box infos (name, id, conf, rect)
             * @param seg_mask_class_id Output pooled full-resolution class-ID mask (CV_8UC1, pixel = class_id + 1, 0 = background)
             * @return True if parsing is successful, false otherwise
             */
            bool parserDetectInfo(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &detect_info_msg,
                            cv::Mat &mask_img,
                            std::vector<BoxInfo>& filtered_all_box_info,
                            cv::Mat& seg_mask_class_id);

            /**
             * @brief Camera info callback function
             * @param msg Shared pointer to the camera info message
             */
            void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

            /**
             * @brief Class info callback function
             * @param msg Shared pointer to the class info message
             */
            void classInfoCallback(const ai_msgs::msg::PerceptionInfo::SharedPtr msg);

            /**
             * @brief Convert depth image to point cloud using camera intrinsics
             * @param depth Mat of depth image
             * @param mask Mat of depth image roi mask
             * @param cloud Point cloud to be populated
             * @return True if conversion is successful, false otherwise
             */
            bool convertDepthToPointcloud(const cv::Mat& depth,
                                        const cv::Mat& mask,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                        float min_depth,
                                        float max_depth);

            /**
             * @brief Callback function for dynamic parameters
             * @return True if callback succeeded, false otherwise
             */
            bool dynamicParaCallback();

            /**
             * @brief Callback function for parameter changes
             * @param params Vector of rclcpp::Parameter objects
             * @return SetParametersResult object indicating success or failure
             */
            rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &params);

            /**
             * @brief Convert log level string to rclcpp::Logger::Level enum
             * @param level_str Log level string (e.g., "INFO", "DEBUG")
             * @return Corresponding rclcpp::Logger::Level enum value
             */
            rclcpp::Logger::Level stringToLogLevel(const std::string& level_str);

            /**
             * @brief Set depth values to zero where mask is non-zero
             * @param mask Mat of mask image
             * @param depth_img Mat of depth image
             * @return True if operation is successful, false otherwise
             */
             bool setDepthZeroByMask(const cv::Mat &mask, cv::Mat &depth_img, bool use_extractor);

            /**
             * @brief Generate mask image by depth image
             * @param depth_img Mat of depth image
             * @param mask Mat of mask image
             * @return True if operation is successful, false otherwise
             */
             bool generateMaskByDepth(const cv::Mat& depth_img, cv::Mat& mask);

            /**
             * @brief Build the set of target class IDs once (for single-pass mask generation)
             */
            void rebuildTargetClassIdSet();

        private:
            rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_{nullptr};  ///< Camera info subscriber
            rclcpp::Subscription<ai_msgs::msg::PerceptionInfo>::SharedPtr class_info_sub_{nullptr};   ///< Class info subscriber

            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_mask_pub_;  ///< Filtered depth mask image publisher
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;  ///< Filtered point cloud publisher
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_pub_;  ///< Filtered depth image publisher
            rclcpp::Publisher<::mask_pc_roi_extractor::msg::ROIPointClouds>::SharedPtr roi_cloud_pub_;  ///< ROI point clouds publisher

            message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;  ///< Depth image subscriber
            message_filters::Subscriber<ai_msgs::msg::PerceptionTargets> detect_info_sub_;   ///< Segmentation mask subscriber

            // ApproximateTime: pairs depth + detection even when timestamps drift slightly,
            // using allow_timestamp_deviation as the maximum interval. Avoids silent frame loss.
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                            ai_msgs::msg::PerceptionTargets> SyncPolicy;
            std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_{nullptr};  ///< Time synchronizer

            rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_{nullptr}; ///< Parameter callback handle

            std::shared_ptr<MaskPcRoiExtractorPara> params_{nullptr};  ///< Shared pointer to extractor parameters

            sensor_msgs::msg::CameraInfo::SharedPtr camera_info_{nullptr};  ///< Camera intrinsic parameters
            std::mutex camera_info_mutex_;  ///< Mutex for camera info access

            // Class-name dictionary: built once from the class-info topic, then immutable.
            // Stored as a shared_ptr to an immutable map so readers take a cheap pointer
            // snapshot (no per-frame deep copy of strings) under a short lock.
            std::shared_ptr<const std::unordered_map<std::string, size_t>> class_names_id_;
            std::mutex class_info_mutex_;

            std::unordered_map<std::string, double> target_names_confidence_; ///< Target class name and confidence (depth filtering)
            std::unordered_map<std::string, double> target_names_roi_confidence_; ///< Target class name and confidence (ROI extraction)
            std::unordered_map<std::string, double> target_names_merged_conf_; ///< Merged name→confidence (union of depth + ROI, min on conflict)
            // Precomputed target class IDs (+1, matching the class-ID mask encoding) for
            // single-pass mask generation. Stored as an immutable shared_ptr snapshot so the
            // detection callback can read it lock-free (written once when class info arrives).
            std::shared_ptr<const std::unordered_set<int>> target_class_ids_;         ///< Depth filtering class IDs
            std::shared_ptr<const std::unordered_set<int>> target_roi_class_ids_;     ///< ROI extraction class IDs
            std::shared_ptr<const std::unordered_set<int>> target_class_ids_merged_;  ///< Merged class IDs (union of depth + ROI)

            std::string Semantic_Segmentation_Mask_Label = "parking_space"; ///< Mask info of Detect rsult

            int MASK_DOWNSAMPLING_RATIO = 4;
            float DEPTH_SCALE = 0.001f;

            // ===== Optimization primitives =====
            std::shared_ptr<ThreadPool> thread_pool_;  ///< Persistent worker threads
            MatPool depth_pool_;  ///< Reusable full-res CV_16UC1 buffers (depth clone for filter stage)
            MatPool mask_pool_;   ///< Reusable full-res CV_8UC1 buffers (mask / seg mask)

            // ===== Debug timing =====
            TaskTiming timing_filter_;       ///< Accumulated execution time for filter task
            TaskTiming timing_roi_;          ///< Accumulated execution time for ROI task
            int        frame_counter_{0};    ///< Frame count for periodic timing report
            int        timing_report_interval_{100};  ///< Print timing averages every N frames
    };

}  // namespace robot::mask_pc_roi_extractor

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(robot::mask_pc_roi_extractor::MaskPcRoiExtractor)
