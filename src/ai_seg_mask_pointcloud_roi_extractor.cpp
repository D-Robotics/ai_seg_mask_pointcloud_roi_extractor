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


#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "ai_seg_mask_pointcloud_roi_extractor/ai_seg_mask_pointcloud_roi_extractor.hpp"

namespace robot::ai_seg_mask_pointcloud_roi_extractor
{

    AISegMaskPointCloudROIExtractor::AISegMaskPointCloudROIExtractor(const rclcpp::NodeOptions &options)
        : rclcpp::Node("depth_mask_extractor_node", options)
    {
        RCLCPP_INFO(get_logger(), "AISegMaskPointCloudROIExtractor Constructed Start");

        {
        std::string node_start_time_str = timeStampTimeToString(this->now());
        RCLCPP_INFO(this->get_logger(), "\n Node Start Time is: %s \n", node_start_time_str.c_str());
        }
        
        // Create parameters object
        if (!params_)
        {
        params_ = std::make_shared<AISegMaskPointCloudROIExtractorPara>();
        }

        if (!initParam())
        {
        RCLCPP_ERROR(get_logger(), "Init Failed !");
        }
        
        RCLCPP_INFO(get_logger(), "AISegMaskPointCloudROIExtractor Constructed End");
    }


    bool AISegMaskPointCloudROIExtractor::initParam()
    {
        RCLCPP_INFO(get_logger(), "Initialize Start");

        // Declare parameters using the parameter management utility
        params_->declare_parameters(this);

        // Get parameters using the parameter management utility
        params_->get_parameters(this);
        
        // Print parameters using the parameter's print method
        params_->print(get_logger());

        if (!parserYamlParam())
        {
        RCLCPP_ERROR(get_logger(), "Failed to convert class info to maps!");
        return false;
        }

        if (!dynamicParaCallback())
        {
            RCLCPP_INFO(get_logger(), "Dynamic para callback failed !");
            return false;
        }

        initializePublisher();

        initializeSubscriber();

        RCLCPP_INFO(get_logger(), "Initialize End");
        return true;
    }


    void AISegMaskPointCloudROIExtractor::initializeSubscriber()
    {
        RCLCPP_INFO(get_logger(), "Initialize Subscriber Start");

        std::vector<std::string> required_topics = {params_->camera_info_topic,
                                                    params_->class_info_topic,
                                                    params_->depth_image_topic,
                                                    params_->detect_info_topic};
        if (check_topic_list(required_topics))
        {
            RCLCPP_INFO(get_logger(), "🔍 Detection completed: All topics exist");
        } 
        else 
        {
            RCLCPP_ERROR(get_logger(), "🔍 Detection completed: Some topics are missing, please check the previous log prompts");
            return;
        }

        // Create camera info subscriber
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            params_->camera_info_topic, 
            rclcpp::QoS(DEFAULT_QOS_DEPTH), 
            std::bind(&AISegMaskPointCloudROIExtractor::cameraInfoCallback, this, std::placeholders::_1));

        // Create class info subscriber
        class_info_sub_ = create_subscription<ai_msgs::msg::PerceptionInfo>(
                        params_->class_info_topic, 
                        rclcpp::QoS(DEFAULT_QOS_DEPTH), 
                        std::bind(&AISegMaskPointCloudROIExtractor::classInfoCallback, this, std::placeholders::_1));

        // Ensure that the camera intrinsic parameters and category information have been successfully subscribed before executing the following callback
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        // Create message filter subscribers for time synchronization
        RCLCPP_INFO(get_logger(), "Time synchronizer initialized with absolute time alignment");
        depth_sub_.subscribe(this, params_->depth_image_topic);
        detect_info_sub_.subscribe(this, params_->detect_info_topic);
        
        // Create time synchronizer using absolute time alignment
        // If allow_timestamp_deviation is true, an approximate time synchronizer could be used
        // But according to requirements, we use absolute time alignment (TimeSynchronizer)
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(params_->queue_size), 
                                                                depth_sub_, 
                                                                detect_info_sub_ ));
        sync_->registerCallback(std::bind(&AISegMaskPointCloudROIExtractor::parserDepthMaskCallback, 
                                        this, 
                                        std::placeholders::_1, 
                                        std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Initialize Subscriber End");

        return;
    }

    void AISegMaskPointCloudROIExtractor::initializePublisher()
    {
        RCLCPP_INFO(get_logger(), "Initialize publisher start");

        // Create publisher
        filtered_depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
            params_->filtered_depth_topic, 
            rclcpp::QoS(DEFAULT_QOS_PUB));
        
        filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            params_->filtered_cloud_topic, 1);

        filtered_depth_mask_pub_ = create_publisher<sensor_msgs::msg::Image>(
            params_->filtered_mask_topic, 
            rclcpp::QoS(DEFAULT_QOS_PUB));

        RCLCPP_INFO(get_logger(), "Initialize publisher end");
        return;
    }


    bool AISegMaskPointCloudROIExtractor::check_single_topic(const std::string &topic_name) 
    {
        if (topic_name.empty()) 
        {
            RCLCPP_ERROR(get_logger(), "❌ Error: Topic name is empty !");
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


    bool AISegMaskPointCloudROIExtractor::check_topic_list(const std::vector<std::string> &topic_list) 
    {
        if (topic_list.empty()) 
        {
            RCLCPP_ERROR(get_logger(), "❌ Topic list is empty !");
            return false;
        }

        RCLCPP_INFO(get_logger(), "📋 Starting to detect %zu topics...", topic_list.size());

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

}  // namespace robot::ai_seg_mask_pointcloud_roi_extractor
