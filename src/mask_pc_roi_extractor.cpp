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

    MaskPcRoiExtractor::MaskPcRoiExtractor(const rclcpp::NodeOptions &options)
        : rclcpp::Node("seg_mask", options)
    {
        RCLCPP_INFO(get_logger(), "MaskPcRoiExtractor constructing...");

        // ---- init params ----
        params_ = std::make_shared<MaskPcRoiExtractorPara>();
        if (!initParam())
        {
            RCLCPP_ERROR(get_logger(), "initParam() failed!");
            return;
        }

        // ---- thread pool ----
        size_t n = static_cast<size_t>(params_->worker_threads);
        if (n < 1) n = 1;
        if (n > 3) n = 3;
        thread_pool_ = std::make_shared<ThreadPool>(n);
        RCLCPP_INFO(get_logger(), "Worker thread pool started with %zu threads", n);
        RCLCPP_INFO(get_logger(), "MaskPcRoiExtractor ready");
    }

    bool MaskPcRoiExtractor::initParam()
    {
        RCLCPP_INFO(get_logger(), "Initialize Params Start");

        // Declare parameters using the parameter management utility
        params_->declare_parameters(this);

        // Get parameters using the parameter management utility
        params_->get_parameters(this);

        if (!parserClassnamesParam())
        {
            RCLCPP_ERROR(get_logger(), "Failed to convert class info to maps!");
            return false;
        }
        if (!dynamicParaCallback())
        {
            RCLCPP_INFO(get_logger(), "Dynamic para callback failed!");
            return false;
        }
        initializePublisher();
        initializeSubscriber();

        RCLCPP_INFO(get_logger(), "Initialize Params End");
        return true;
    }


    void MaskPcRoiExtractor::initializeSubscriber()
    {
        RCLCPP_INFO(get_logger(), "Initialize Subscriber Start");

        {
            std::vector<std::string> required_topics = {params_->camera_info_topic,
                                                        params_->class_info_topic,
                                                        params_->depth_image_topic,
                                                        params_->detect_info_topic};
            constexpr int    kMaxRetries = 50;     // 50 × 100ms = 5 s max wait
            constexpr auto   kRetryDelay = std::chrono::milliseconds(100);
            bool all_found = false;
            for (int attempt = 0; attempt < kMaxRetries; ++attempt)
            {
                if (check_topic_list_quiet(required_topics))
                {
                    all_found = true;
                    RCLCPP_INFO(get_logger(), "All required topics found (attempt %d)", attempt + 1);
                    break;
                }
                rclcpp::sleep_for(kRetryDelay);
            }
            if (!all_found)
            {
                // Log which topics are still missing for easier debugging.
                for (const auto& t : required_topics)
                {
                    if (!check_single_topic(t))
                    {
                        RCLCPP_ERROR(get_logger(), "Topic not found after %d retries: %s",
                                    kMaxRetries, t.c_str());
                    }
                }
            }
        }

        // Create camera info subscriber
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            params_->camera_info_topic,
            rclcpp::QoS(DEFAULT_QOS_DEPTH),
            std::bind(&MaskPcRoiExtractor::cameraInfoCallback, 
                    this, 
                    std::placeholders::_1));

        // Create class info subscriber
        class_info_sub_ = create_subscription<ai_msgs::msg::PerceptionInfo>(
                        params_->class_info_topic,
                        rclcpp::QoS(DEFAULT_QOS_DEPTH),
                        std::bind(&MaskPcRoiExtractor::classInfoCallback,
                                this,
                                std::placeholders::_1));

        // ---- message_filters sync (pure rclcpp::Node* — no LifecycleNode crash) ----
        RCLCPP_INFO(get_logger(), "Initializing time synchronizer (ApproximateTime)");
        depth_sub_.subscribe(this, params_->depth_image_topic);
        detect_info_sub_.subscribe(this, params_->detect_info_topic);
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(params_->queue_size),
                                                                depth_sub_,
                                                                detect_info_sub_));
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(params_->allow_timestamp_deviation));
        sync_->registerCallback(std::bind(&MaskPcRoiExtractor::parserDepthMaskCallback,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "Time synchronizer initialized (ApproximateTime)");
        
        RCLCPP_INFO(get_logger(), "Initialize Subscriber End");

        return;
    }

    void MaskPcRoiExtractor::initializePublisher()
    {
        RCLCPP_INFO(get_logger(), "Initialize Publisher Start");

        // Create publisher
        filtered_depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
            params_->filtered_depth_topic,
            rclcpp::QoS(DEFAULT_QOS_PUB));

        filtered_depth_mask_pub_ = create_publisher<sensor_msgs::msg::Image>(
            params_->filtered_mask_topic,
            rclcpp::QoS(DEFAULT_QOS_PUB));

        filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            params_->filtered_cloud_topic, 1);

        roi_cloud_pub_ = create_publisher<::mask_pc_roi_extractor::msg::ROIPointClouds>(
            params_->roi_cloud_topic, 1);

        RCLCPP_INFO(get_logger(), "Initialize Publisher End");
        return;
    }


    bool MaskPcRoiExtractor::check_single_topic(const std::string &topic_name)
    {
        if (topic_name.empty())
        {
            RCLCPP_ERROR(get_logger(), "❌ Error: Topic name is empty!");
            return false;
        }

        // const std::string full_topic = this->resolve_topic_name(topic_name);
        const auto all_topics = this->get_node_graph_interface()->get_topic_names_and_types();
        const bool exists = all_topics.find(topic_name) != all_topics.end();
        if (exists)
        {
            RCLCPP_INFO(get_logger(), "✅ Topic [%s] - Exists", topic_name.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "❌ Topic [%s] - does not exist, Viewing method : [ros2 topic info %s --once]",
                                            topic_name.c_str(), topic_name.c_str());
            return false;
        }
    }


    bool MaskPcRoiExtractor::check_topic_list(const std::vector<std::string> &topic_list)
    {
        if (topic_list.empty())
        {
            RCLCPP_ERROR(get_logger(), "❌ Topic list is empty!");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Starting to detect %zu topics...", topic_list.size());
        bool all_exists = true;
        for (const auto &topic : topic_list)
        {
            if (!check_single_topic(topic))
            {
                all_exists = false;
            }
        }
        return all_exists;
    }

    bool MaskPcRoiExtractor::check_topic_list_quiet(const std::vector<std::string> &topic_list)
    {
        if (topic_list.empty()) return false;
        const auto all_topics = this->get_node_graph_interface()->get_topic_names_and_types();
        for (const auto &topic : topic_list)
        {
            if (all_topics.find(topic) == all_topics.end()) return false;
        }
        return true;
    }

}  // namespace robot::mask_pc_roi_extractor
