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

namespace seg_mask_roi_extractor
{
    void AISegMaskPointCloudROIExtractor::pubImage(double timestamp,
                            std::string frame_id,
                            cv::Mat &image,
                            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher)
    {
        sensor_msgs::msg::Image::SharedPtr ros_image;
        try
        {
            int image_type = image.type();
            std::string encoding;
            if (image_type == CV_16UC1)
            {
                encoding = "mono16";
            }
            else if (image_type == CV_32FC1)
            {
                encoding = "32FC1";
            }
            else
            {
                encoding = image.channels() > 1 ? "rgb8" : "mono8";
            }
            
            ros_image = cv_bridge::CvImage(
                        std_msgs::msg::Header(),
                        encoding, 
                        image)
                        .toImageMsg();

            ros_image->header.frame_id = frame_id;
            ros_image->header.stamp = timeStampDoubleToTime(timestamp);
            publisher->publish(*ros_image);
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: {}", e.what());
        }
        return;
    }

    void AISegMaskPointCloudROIExtractor::pubPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                    std::string frame_id,
                                    double timestamp,
                                    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher)
    {
        if (!cloud || !publisher) 
        {
        RCLCPP_ERROR(get_logger(), "Invalid point cloud data or publisher for publishing!");
        return;
        }
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud, ros_cloud);
        ros_cloud.header.frame_id = frame_id;
        ros_cloud.header.stamp = timeStampDoubleToTime(timestamp);
        publisher->publish(ros_cloud);
        return;
    }
    
} // namespace seg_mask_roi_extractor