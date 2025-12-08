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

namespace robot::ai_seg_mask_pointcloud_roi_extractor
{
    void AISegMaskPointCloudROIExtractor::parserDepthMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg, 
                                                      const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &detect_info_msg)
    {
        // Check if messages are valid
        if (!depth_msg || !detect_info_msg) 
        {
            RCLCPP_ERROR(get_logger(), "Invalid depth or detect info message received !");
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
                                        depth_msg->header.stamp.sec, depth_msg->header.stamp.nanosec);
            RCLCPP_DEBUG(get_logger(), "Detect-Info Msg timestamp: %d.%09u", 
                                        detect_info_msg->header.stamp.sec, detect_info_msg->header.stamp.nanosec);
            
            RCLCPP_DEBUG(get_logger(), "Depth Msg publish-subscribe delay: %.3f ms (%.6f s)", 
                                        depth_delay.nanoseconds() / 1000000.0, depth_delay.seconds());
            RCLCPP_DEBUG(get_logger(), "Detect-Info Msg publish-subscribe delay: %.3f ms (%.6f s)", 
                                        detect_info_delay.nanoseconds() / 1000000.0, detect_info_delay.seconds());
            RCLCPP_DEBUG(get_logger(), "Depth-Mask Msg time difference: %.3f ms (%.6f s)", 
                                        std::llabs(msg_time_diff.nanoseconds()) / 1000000.0, 
                                        std::abs(msg_time_diff.seconds()));
        }

        double time_stamp = headerTimeStampTimeToDoubleSec(depth_msg->header);
        std::string frame_id = depth_msg->header.frame_id;

        // Parse Depth Map
        cv::Mat depth_img = cv::Mat::zeros(params_->camera_height, params_->camera_width, CV_16UC1);
        if (!parserDepth(depth_msg, depth_img))
        {
            RCLCPP_ERROR(get_logger(), "Depth map parsing failed !");
            return;
        }

        // Parse detection information to generate a valid region mask
        cv::Mat mask_img = cv::Mat::zeros(params_->camera_height, params_->camera_width, CV_8UC1);
        if (!parserDetectInfo(detect_info_msg, mask_img)) 
        {
            RCLCPP_ERROR(get_logger(), "Detect info parsing failed !");
            return;

        }

        // Dilate the mask
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::dilate(mask_img,  mask_img, kernel, cv::Point(-1,-1), params_->dilate_iter_num);

        // Filter out areas of no interest
        setDepthZeroByMask(mask_img, depth_img);
        
        pubImage(time_stamp, frame_id, depth_img, filtered_depth_pub_);

        end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        RCLCPP_DEBUG(get_logger(), "[parserDepthMaskCallback()] Run Time = %.3f ms", duration.count() / 1000.0);

        // Obtain the mask of the depth map
        cv::Mat depth_mask;
        if (!generateMaskByDepth(depth_img, depth_mask))
        {
            RCLCPP_ERROR(get_logger(), "Failed to obtain the mask of the depth map !");
            return;
        }
        pubImage(time_stamp, frame_id, depth_mask, filtered_depth_mask_pub_);

        if(params_->debug)
        {
            // Depth Map to Point Cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cv::Mat mask_label = cv::Mat::zeros(depth_img.size(), CV_8UC1);
            if(!convertDepthToPointcloud(depth_img, mask_label, cloud))
            {
                RCLCPP_ERROR(get_logger(), "Depth map to point cloud conversion failed !");
                return;
            }
            pubPointCloud(cloud, frame_id, time_stamp, filtered_cloud_pub_);
        }

        return;
    }

    bool AISegMaskPointCloudROIExtractor::parserDepth(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
                                            cv::Mat &depth_img)
    {
        // Convert ROS messages to OpenCV images
        cv_bridge::CvImagePtr depth_cv_ptr;
        if (depth_msg->encoding != sensor_msgs::image_encodings::MONO16 and depth_msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1)
        {
            RCLCPP_ERROR(get_logger(), "Depth image encoding must be MONO16 or TYPE_16UC1 !");
            return false;
        }

        try
        {
            depth_cv_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "=> cv_bridge exception: %s", e.what());
            return false;
        }
        depth_img = depth_cv_ptr->image;

        if (depth_img.rows != static_cast<int>(params_->camera_height) || 
            depth_img.cols != static_cast<int>(params_->camera_width))
        {
            RCLCPP_ERROR(get_logger(), "Depth image size must be %zu x %zu !", params_->camera_height, params_->camera_width);
            return false;
        }

        return true;
    }

    bool AISegMaskPointCloudROIExtractor::parserDetectInfo(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &detect_info_msg,
                                                    cv::Mat &mask_img)
    {
        std::vector<BoxInfo> filtered_all_box_info;
        cv::Mat seg_mask;
        cv::Mat detect_valid_area_mask  = cv::Mat::zeros(params_->camera_height, params_->camera_width, CV_8UC1);

        auto class_names_id = getClassNamesInfo();
        if (class_names_id.empty()) 
        {
            RCLCPP_ERROR(get_logger(), "class_names_id is empty, no class info loaded!");
            return false;
        } 

        if (detect_info_msg->targets.empty())
        {
            RCLCPP_WARN(get_logger(), "detect_info_msg->targets is empty!");
            return true;
        }

        for (const auto &target : detect_info_msg->targets)
        {
            // Get name
            std::string class_name = target.type;

            // Get id
            size_t id = class_names_id[class_name];

            if (class_name == Semantic_Segmentation_Mask_Label)
            {
                auto captures = target.captures[0];

                RCLCPP_DEBUG(get_logger(), "captures img: width = %d, height=%d", captures.img.width, captures.img.height);

                // 目前PerceptionTargets.msg的capture是float数组，但存储的类别标签的int
                seg_mask = cv::Mat(captures.img.height, captures.img.width, CV_32FC1);
                if (seg_mask.total() != captures.features.size()) 
                {
                    RCLCPP_ERROR(get_logger(), "Features size(%zu) mismatch with seg_mask size(%ld)!", 
                                captures.features.size(), seg_mask.total());
                    return false;
                }

                std::memcpy(seg_mask.data, captures.features.data(), captures.features.size() * sizeof(float));
                seg_mask.convertTo(seg_mask, CV_8UC1);

                cv::Mat mask_valid = cv::Mat::zeros(seg_mask.size(), CV_8UC1);
                for (const auto& pair : target_names_confindece_) 
                {
                    int mask_id = class_names_id[pair.first] + 1;
                    mask_valid |= (seg_mask == mask_id);
                }

                cv::resize(mask_valid, 
                        seg_mask, 
                        cv::Size(seg_mask.cols * MASK_DOWNSAMPLING_RATIO, seg_mask.rows * MASK_DOWNSAMPLING_RATIO), 
                        0, 
                        0, 
                        cv::INTER_NEAREST);

                if (seg_mask.rows !=  static_cast<int>(params_->camera_height) || 
                    seg_mask.cols !=  static_cast<int>(params_->camera_width))
                {
                    RCLCPP_ERROR(get_logger(), "Image seg_mask size must be %zu x %zu !", params_->camera_height, params_->camera_width);
                    return false;
                }
                continue;
            }

            for (const auto &roi : target.rois)
            {
                // Get confindece
                double confidence = roi.confidence;

                if (target_names_confindece_.count(class_name) ==0 || confidence < target_names_confindece_[class_name])
                {
                    RCLCPP_DEBUG(get_logger(), "class_name = %s, confinence=%f, target_names_confindece=%f", 
                                                class_name.c_str(), 
                                                confidence, 
                                                target_names_confindece_[class_name]);
                    continue;
                }

                // RCLCPP_INFO(get_logger(), "CLASS_NAME = %s", class_name.c_str());

                BoxInfo box_info;
                {
                    box_info.name = class_name;
                    box_info.id = id;
                    box_info.confidence = confidence;
                    box_info.x_offset = roi.rect.x_offset;
                    box_info.y_offset = roi.rect.y_offset;
                    box_info.width = roi.rect.width;
                    box_info.height = roi.rect.height;

                    if (static_cast<size_t>(box_info.x_offset + box_info.width) > params_->camera_width || 
                        static_cast<size_t>(box_info.y_offset + box_info.height) > params_->camera_height) 
                    {
                        RCLCPP_WARN(get_logger(), "The size of the detection box exceeds the size of the image.");
                        continue;
                    }
                    cv::Rect rect(box_info.x_offset, box_info.y_offset, box_info.width, box_info.height);
                    detect_valid_area_mask(rect).setTo(255);
                }
                
                filtered_all_box_info.push_back(box_info);
            }
        }

        if (seg_mask.size() != detect_valid_area_mask.size())
        {
            RCLCPP_ERROR(get_logger(), "Image seg_mask size %d x %d must be quivalent to detect_valid_area_mask size %d x %d !", 
                                        seg_mask.rows, 
                                        seg_mask.cols, 
                                        detect_valid_area_mask.rows, 
                                        detect_valid_area_mask.cols);
            return false;
        }
        cv::bitwise_and(seg_mask, detect_valid_area_mask, mask_img);

        return true;
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

        params_->camera_height = msg->height;

        params_->camera_width = msg->width;
        
        // Log camera info received
        RCLCPP_INFO(get_logger(), "Camera info received with frame_id: %s", camera_info_->header.frame_id.c_str());
        RCLCPP_INFO(get_logger(), "Camera Height = %ld, Width = %ld", params_->camera_height, params_->camera_width);
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
        std::lock_guard<std::mutex> lock(class_info_mutex_);

        if(!msg)
        {
            RCLCPP_ERROR(get_logger(), "Invalid class names message received");
            return;
        }
        RCLCPP_INFO(get_logger(), "Class_Info_Callback W:%d H:%d Class_Size:%ld", msg->width, msg->height, msg->class_names.size());

        for (size_t idx = 0; idx < msg->class_names.size(); idx++)
        {
            class_names_id_[msg->class_names[idx]] = idx;
        }

        if (class_names_id_.empty()) 
        {
            RCLCPP_WARN(get_logger(), "class_names_id_ is empty!");
            return;
        }

        RCLCPP_INFO(get_logger(), "Model_Class_Info : ");
        for (const auto& pair : class_names_id_) 
        {
            RCLCPP_INFO(get_logger(), "----------Name: %s  --> ID: %zu", pair.first.c_str(), pair.second);
        }

        // Unsubscribe from class names info topic after receiving the first message
        class_info_sub_.reset();

        return;

    }

    std::map<std::string, size_t> AISegMaskPointCloudROIExtractor::getClassNamesInfo()
    {
        std::lock_guard<std::mutex> lock(class_info_mutex_);
        return class_names_id_;
    }

    bool AISegMaskPointCloudROIExtractor::convertDepthToPointcloud(const cv::Mat& depth,
                                        const cv::Mat &mask,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        if (depth.size() != mask.size())
        {
            RCLCPP_ERROR(get_logger(), "Depth and mask images have different sizes! Skipping processing.");
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
            
            // Convert depth image to 3D points
            for (int v = 0; v < depth.rows; ++v) 
            {
                for (int u = 0; u < depth.cols; ++u) 
                {
                    if (mask.at<uchar>(v, u) == 255) continue;
                    
                    float depth_value = depth.at<uint16_t>(v, u) * DEPTH_SACLAE;
                    
                    // Skip invalid depth values
                    if (depth_value <= params_->min_depth || depth_value > params_->max_depth || std::isnan(depth_value) || std::isinf(depth_value)) 
                    {
                        continue;
                    }
                    
                    // Calculate 3D point
                    float x = (static_cast<float>(u) - cx) * depth_value / fx;
                    float y = (static_cast<float>(v) - cy) * depth_value / fy;
                    
                    pcl::PointXYZ point;
                    point.z = depth_value;
                    point.y = y;
                    point.x = x;

                    cloud->push_back(point);
                }
            }

            // Set point cloud metadata
            cloud->width = cloud->size();
            cloud->height = 1;
            cloud->is_dense = false;
        }
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(get_logger(), "Error converting depth to point cloud: %s", e.what());
            return false;
        }

        return true;
    }

    bool AISegMaskPointCloudROIExtractor::setDepthZeroByMask(const cv::Mat &mask, cv::Mat &depth_img) 
    {
        if (mask.empty() || depth_img.empty()) 
        {
            RCLCPP_ERROR(get_logger(), "Mask or depth_img is empty!");
            return false;
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

        try 
        {
            const uchar* mask_data = mask.data;
            // Bytes per row (adapted to continuous/discontinuous matrix)
            const size_t mask_step = mask.step; 
            const bool use_extractor = params_->use_extractor;

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
                    (params_->use_extractor ? "true" : "false"));
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

        mask = cv::Mat::zeros(depth_img.size(), CV_8UC1);

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

} // namespace robot::ai_seg_mask_pointcloud_roi_extractor