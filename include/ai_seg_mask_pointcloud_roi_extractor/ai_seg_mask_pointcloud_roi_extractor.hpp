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

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <map>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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

/**
 * @brief Macro to run a callback function and measure its execution time.
 * @param prefix Prefix for the log message
 * @param debug Whether to enable debug mode
 * @param ... Callback function invocation
 */
#define RUN_AND_TIMING_PREFIX(prefix, debug, ...)                                                     \
    do                                                                                                \
    {                                                                                                 \
        if (debug)                                                                                    \
        {                                                                                             \
            struct timespec stCurTime0;                                                               \
            clock_gettime(CLOCK_REALTIME, &stCurTime0);                                               \
            __VA_ARGS__;                                                                              \
            struct timespec stCurTime1;                                                               \
            clock_gettime(CLOCK_REALTIME, &stCurTime1);                                               \
            double delta_time =                                                                       \
                (stCurTime1.tv_sec - stCurTime0.tv_sec) * 1000.0 +                                    \
                (stCurTime1.tv_nsec - stCurTime0.tv_nsec) * 0.000001;                                 \
            std::cout << "[" << prefix << "] : cost time is = " << delta_time << "(ms)" << std::endl; \
        }                                                                                             \
        else                                                                                          \
        {                                                                                             \
            __VA_ARGS__;                                                                              \
        }                                                                                             \
    } while (0);


namespace seg_mask_roi_extractor
{
    /**
    * @struct BoxInfo
    * @brief Information about a detected object box
    */
    struct BoxInfo
    {
        std::string name;  ///< Class name
        size_t id;         ///< Class ID
        double confidence; ///< Confidence score
        int x_offset;      ///< Width offset of box
        int y_offset;      ///< Height offset of box
        int width;         ///< Width offset of box
        int height;        ///< Height offset of box

        /**
        * @brief Default constructor for box info
        */
        BoxInfo() = default;
    };

        /**
    * @brief Parameters for depth edge extraction algorithm
    * Contains configuration parameters for depth-based mask extraction,
    * including confidence thresholds and target classes for segmenting
    * depth images into different regions of interest.
    */
    struct AISegMaskPointCloudROIExtractorPara
    {
        /**
         * @brief Shared pointer alias.
         */
        using Ptr = std::shared_ptr<AISegMaskPointCloudROIExtractorPara>;

        bool debug{false};                             ///< Debug mode
        bool topic_check{false};                       ///< Topic check
        std::string  depth_image_topic{""};            ///< Depth image topic name
        std::string  detect_info_topic{""} ;           ///< Segmentation detect topic name
        std::string  class_info_topic{""} ;            ///< Class topic name
        std::string  camera_info_topic{""};            ///< Camera info topic name
        std::string  color_image_topic{""};             ///< Color image topic name
        std::string  filtered_mask_topic{""};          ///< Filtered mask image topic name
        std::string  filtered_depth_topic{""};         ///< Filtered depth image topic name
        std::string  filtered_cloud_topic{""};         ///< Filtered point cloud topic name
        std::string  confidence_threshold_file_path{""}; ///< File path of confidence threshold for each class
        int queue_size{10};                            ///< Queue size for message synchronization
        double allow_timestamp_deviation{1.0};          ///< Allowable timestamp deviation for synchronization
        double min_depth{0.0};                         ///< Minimum depth value for filtering
        double max_depth{5.0};                         ///< Maximum depth value for filtering
        int dilate_iter_num{1};                        ///< Number of iterations for dilation
        size_t camera_width{640};                      ///< Camera width
        size_t camera_height{352};                     ///< Camera height
        std::string log_level{"info"};                 ///< Log level
        bool use_extractor{true};                      ///< Whether to extract the region of interest, true := extract, false := filter

        /**
        * @brief Constructor for depth mask extractor parameters
        * @param debug Debug mode
        * @param topic_check Topic check
        * @param depth_image_topic Depth image topic name
        * @param detect_info_topic Segmentation detect topic name
        * @param class_info_topic Class topic name
        * @param camera_info_topic Camera info topic name
        * @param color_image_topic Color image topic name
        * @param filtered_mask_topic Filtered mask image topic name
        * @param filtered_depth_topic Filtered depth image topic name
        * @param filtered_cloud_topic Filtered point cloud topic name
        * @param confidence_threshold_file_path File path of confidence threshold for each class
        * @param queue_size Queue size for message synchronization
        * @param allow_timestamp_deviation Allowable timestamp deviation for synchronization
        * @param min_depth Minimum depth value for filtering
        * @param max_depth Maximum depth value for filtering
        * @param dilate_iter_num  Number of iterations for dilation
        * @param camera_width Camera width
        * @param camera_height Camera height
        * @param log_level Log level
        * @param use_extractor Whether to extract the region of interest, true := extract, false := filter
        */
        AISegMaskPointCloudROIExtractorPara(
            bool debug,
            bool topic_check,
            std::string  depth_image_topic,      
            std::string  detect_info_topic,  
            std::string  class_info_topic,  
            std::string  camera_info_topic,  
            std::string  color_image_topic,            
            std::string  filtered_mask_topic,          
            std::string  filtered_depth_topic,     
            std::string  filtered_cloud_topic,     
            std::string confidence_threshold_file_path,
            int queue_size,
            double allow_timestamp_deviation,
            double min_depth,
            double max_depth,
            int dilate_iter_num,
            size_t camera_width,
            size_t camera_height,
            std::string log_level,
            bool use_extractor
        ) : 
        debug(debug),
        topic_check(topic_check),
        depth_image_topic(depth_image_topic),
        detect_info_topic(detect_info_topic),
        class_info_topic(class_info_topic),
        camera_info_topic(camera_info_topic),
        color_image_topic(color_image_topic),             
        filtered_mask_topic(filtered_mask_topic), 
        filtered_depth_topic(filtered_depth_topic),
        filtered_cloud_topic(filtered_cloud_topic),
        confidence_threshold_file_path(confidence_threshold_file_path),
        queue_size(queue_size),
        allow_timestamp_deviation(allow_timestamp_deviation),
        min_depth(min_depth),
        max_depth(max_depth),
        dilate_iter_num(dilate_iter_num),
        camera_width(camera_width),
        camera_height(camera_height),
        log_level(log_level),
        use_extractor(use_extractor) {}

        /**
        * @brief Default constructor for depth mask extractor parameters
        */
        AISegMaskPointCloudROIExtractorPara() = default;

        /**
         * @brief Print the configuration parameters to the console
         * @param logger Logger to use for printing
         */
        void print(const rclcpp::Logger &logger) const
        {
            RCLCPP_INFO(logger, "DepthMaskExtractor-Config-Param:");
            RCLCPP_INFO(logger, "  debug: %s", this->debug ? "true" : "false");
            RCLCPP_INFO(logger, "  topic_check: %s", this->topic_check ? "true" : "false");
            RCLCPP_INFO(logger, "  depth_image_topic: %s", this->depth_image_topic.c_str());
            RCLCPP_INFO(logger, "  detect_info_topic: %s", this->detect_info_topic.c_str());
            RCLCPP_INFO(logger, "  class_info_topic: %s", this->class_info_topic.c_str());
            RCLCPP_INFO(logger, "  camera_info_topic: %s", this->camera_info_topic.c_str());
            RCLCPP_INFO(logger, "  color_image_topic: %s", this->color_image_topic.c_str());
            RCLCPP_INFO(logger, "  filtered_mask_topic: %s", this->filtered_mask_topic.c_str());
            RCLCPP_INFO(logger, "  filtered_depth_topic: %s", this->filtered_depth_topic.c_str());
            RCLCPP_INFO(logger, "  filtered_cloud_topic: %s", this->filtered_cloud_topic.c_str());
            RCLCPP_INFO(logger, "  confidence_threshold_file_path: %s", this->confidence_threshold_file_path.c_str());
            RCLCPP_INFO(logger, "  queue_size: %d", this->queue_size);
            RCLCPP_INFO(logger, "  allow_timestamp_deviation: %.2f", this->allow_timestamp_deviation);
            RCLCPP_INFO(logger, "  min_depth: %.2f", this->min_depth);
            RCLCPP_INFO(logger, "  max_depth: %.2f", this->max_depth);
            RCLCPP_INFO(logger, "  dilate_iter_num: %d", this->dilate_iter_num);
            RCLCPP_INFO(logger, "  camera_width: %ld", this->camera_width);
            RCLCPP_INFO(logger, "  camera_height: %ld", this->camera_height);
            RCLCPP_INFO(logger, "  log_level: %s", this->log_level.c_str());
            RCLCPP_INFO(logger, "  use_extractor: %s", this->use_extractor ? "true" : "false");
        }   

        /**
         * @brief Declare parameters for the depth mask extractor
         * @param node Pointer to the node interface for parameter declaration
         */
        void declare_parameters(rclcpp::Node* node)
        {
            node->declare_parameter("debug", false);
            node->declare_parameter("topic_check", false);
            node->declare_parameter("depth_image_topic", "image_raw");
            node->declare_parameter("detect_info_topic", "hobot_dnn_detection");
            node->declare_parameter("class_info_topic", "hobot_dnn_detection_info");
            node->declare_parameter("camera_info_topic", "camera_info");
            node->declare_parameter("color_image_topic", "image_raw");
            node->declare_parameter("filtered_mask_topic", "filtered_mask");
            node->declare_parameter("filtered_depth_topic", "filtered_depth_img");
            node->declare_parameter("filtered_cloud_topic", "filtered_depth_cloud");
            node->declare_parameter("confidence_threshold_file_path", "confidence_threshold_file_path");
            node->declare_parameter("queue_size", 10);
            node->declare_parameter("allow_timestamp_deviation", 1.0); 
            node->declare_parameter("min_depth", 0.0);
            node->declare_parameter("max_depth", 5.0); 
            node->declare_parameter("dilate_iter_num", 1); 
            node->declare_parameter("camera_width", 640);
            node->declare_parameter("camera_height", 352);
            node->declare_parameter("log_level", "info");
            node->declare_parameter("use_extractor", false);
        }

        /**
         * @brief Get parameters for the depth mask extractor
         * @param node Pointer to the node interface for parameter retrieval
         */
        void get_parameters(rclcpp::Node* node)
        {
            node->get_parameter("debug", this->debug);
            node->get_parameter("topic_check", this->topic_check);
            node->get_parameter("depth_image_topic", this->depth_image_topic);
            node->get_parameter("detect_info_topic", this->detect_info_topic);
            node->get_parameter("class_info_topic", this->class_info_topic);
            node->get_parameter("camera_info_topic", this->camera_info_topic);
            node->get_parameter("color_image_topic", this->color_image_topic);
            node->get_parameter("filtered_mask_topic", this->filtered_mask_topic);
            node->get_parameter("filtered_depth_topic", this->filtered_depth_topic);
            node->get_parameter("filtered_cloud_topic", this->filtered_cloud_topic);
            node->get_parameter("confidence_threshold_file_path", this->confidence_threshold_file_path);
            node->get_parameter("queue_size", this->queue_size);
            node->get_parameter("allow_timestamp_deviation", this->allow_timestamp_deviation);
            node->get_parameter("min_depth", this->min_depth);
            node->get_parameter("max_depth", this->max_depth);
            node->get_parameter("dilate_iter_num", this->dilate_iter_num);
            node->get_parameter("camera_width", this->camera_width);
            node->get_parameter("camera_height", this->camera_height);
            node->get_parameter("log_level", this->log_level);
            node->get_parameter("use_extractor", this->use_extractor);
        }
    };

    constexpr size_t DEFAULT_QOS_DEPTH = 10;
    constexpr size_t DEFAULT_QOS_PUB = 1;

    /**
    * @class AISegMaskPointCloudROIExtractor
    * @brief A lifecycle node that extracts depth masks from segmentation masks
    * 
    * This node subscribes to both depth images and segmentation masks, aligns them by timestamp,
    * and extracts depth masks based on the mask information.
    */
    class AISegMaskPointCloudROIExtractor : public rclcpp::Node
    {
        public:
            /**
            * @brief Constructor
            * @param options Node options
            */
            explicit AISegMaskPointCloudROIExtractor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

            /**
            * @brief Destructor
            */
            virtual ~AISegMaskPointCloudROIExtractor() = default;

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
            * @brief Parser yaml paramter
            */
            bool parserYamlParam();

            /**
            * @brief Periodically print time synchronization abnormal status information
            */
            void timeSyncStatus();

            /**
            * @brief Time synchronization periodic detection
            */
            bool timeSyncDetect();

            /**
            * Check if all topics that need to be subscribed to exist
            * @return bool All Exist → true, otherwise → false
            */
            bool checkRequiredTopic();

            /**
            * @brief Get camera intrinsic parameters
            * @return Shared pointer to camera info, or nullptr if not available
            */
            sensor_msgs::msg::CameraInfo::SharedPtr getCameraInfo();

            /**
            * @brief Get class names and their corresponding IDs
            * @return Map of class names to their IDs
            */
            std::map<std::string, size_t> getClassNamesInfo();

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
            static double headerTimeStampTimeToDoubleSec(std_msgs::msg::Header header);

            /**
            * @brief Convert header timestamp to double timestamp in milliseconds
            * @param header Header message containing timestamp
            * @return Corresponding double timestamp in milliseconds
            */
            static double headerTimeStampTimeToDoubleMilliSec(std_msgs::msg::Header header);

            /**
            * @brief Convert header timestamp to double timestamp in nanoseconds
            * @param header Header message containing timestamp
            * @return Corresponding double timestamp in nanoseconds
            */
            static double headerTimeStampTimeToDoubleNanoSec(std_msgs::msg::Header header);

            /**
            * @brief Convert header timestamp to rclcpp::Time
            * @param header Header message containing timestamp
            * @return Corresponding rclcpp::Time
            */
            static rclcpp::Time headerTimeStampToTime(std_msgs::msg::Header header);

            /**
            * @brief Publish an image using ImageInfo structure
            * @param timestamp Timestamp for the message header
            * @param frame_id Coordinate frame ID for the message header
            * @param image Shared pointer to ImageInfo structure containing image data
            * @param publisher Shared pointer to the ROS2 image publisher
            */
            void pubImage(double timestamp,
                        std::string frame_id,
                        cv::Mat &image,
                        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher);

            /**
            * @brief Publish a point cloud to a ROS2 topic
            * @param cloud Shared pointer to the PCL point cloud to publish
            * @param frame_id Coordinate frame ID for the message header
            * @param timestamp Timestamp for the message header
            * @param publisher Shared pointer to the ROS2 publisher
            */
           void pubPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                            std::string frame_id,
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
            * @brief Parse depth image message into a cv::Mat
            * @param depth_msg Shared pointer to the depth image message
            * @param depth_img Output cv::Mat to store the depth image
            * @return True if parsing is successful, false otherwise
            */
            bool parserDepth(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
                            cv::Mat &depth_img);


            /**
            * @brief Parse detection info message into a cv::Mat mask
            * @param detect_info_msg Shared pointer to the detection info message
            * @param mask_img Output cv::Mat to store the mask image
            * @return True if parsing is successful, false otherwise
            */
            bool parserDetectInfo(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &detect_info_msg,
                            cv::Mat &mask_img);

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
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

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
             bool setDepthZeroByMask(const cv::Mat &mask, cv::Mat &depth_img);


            /**
             * @brief Generate mask image by depth image
             * @param depth_img Mat of depth image
             * @param mask Mat of mask image
             * @return True if operation is successful, false otherwise
             */
             bool generateMaskByDepth(const cv::Mat& depth_img, cv::Mat& mask);

            /**
            * Check if single topics that need to be subscribed to exist
            * @param topic_list Topic name
            * @return bool Exist → true, otherwise → false
            */
            bool checkSingleTopic(const std::string &topic_name);

            /**
            * Check if all topics that need to be subscribed to exist
            * @param topic_list Topic List Vector
            * @return bool All Exist → true, otherwise → false
            */
            bool checkTopicList(const std::vector<std::string> &topic_list);

        private:         
            rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_{nullptr};  ///< Camera info subscriber
            rclcpp::Subscription<ai_msgs::msg::PerceptionInfo>::SharedPtr class_info_sub_{nullptr};   ///< Class info subscriber
            rclcpp::Subscription<ai_msgs::msg::PerceptionInfo>::SharedPtr color_image_sub_{nullptr};   ///< color image subscriber

            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_mask_pub_;  ///< Filtered depth mask image publisher
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;  ///< Filtered point cloud publisher
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_pub_;  ///< Filtered depth image publisher

            message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;  ///< Depth image subscriber
            message_filters::Subscriber<ai_msgs::msg::PerceptionTargets> detect_info_sub_;   ///< Segmentation mask subscriber

            typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, 
                                                            ai_msgs::msg::PerceptionTargets> SyncPolicy;
            std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_{nullptr};  ///< Time synchronizer

            rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_{nullptr}; ///< Parameter callback handle

            std::shared_ptr<AISegMaskPointCloudROIExtractorPara> params_{nullptr};  ///< Shared pointer to extractor parameters
            
            sensor_msgs::msg::CameraInfo::SharedPtr camera_info_{nullptr};  ///< Camera intrinsic parameters
            std::mutex camera_info_mutex_;  ///< Mutex for camera info access

            std::map<std::string, size_t> class_names_id_; ///< Class name and id
            std::mutex class_info_mutex_;

            std::map<std::string, double> target_names_confindece_; ///< Target class name and confidence

            std::string Semantic_Segmentation_Mask_Label = "parking_space"; ///< Mask info of Detect rsult 
            
            int MASK_DOWNSAMPLING_RATIO = 4;
            float DEPTH_SACLAE = 0.001f;

            double last_time_ = -std::numeric_limits<double>::infinity();  ///< Record the time of the previous frame data
            double sync_time_delta_{0.5}; ///< Time synchronization time interval
            rclcpp::TimerBase::SharedPtr timeSyncTimer_{nullptr};
            rclcpp::Time last_receive_time_;

    };

} 

// // Register the component with class_loader
// RCLCPP_COMPONENTS_REGISTER_NODE(seg_mask_roi_extractor::AISegMaskPointCloudROIExtractor)
