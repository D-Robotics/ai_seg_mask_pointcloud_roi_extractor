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
// pubImage — raw / uncompressed
// ══════════════════════════════════════════════════════════════════════

void MaskPcRoiExtractor::pubImage(double timestamp,
                        const std::string& frame_id,
                        cv::Mat &image,
                        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher)
{
    pubImageCompressed(timestamp, frame_id, image, publisher,
                       ImageCompression::kNone, 0);
}

// ══════════════════════════════════════════════════════════════════════
// pubImageCompressed — with compression support
// ══════════════════════════════════════════════════════════════════════

void MaskPcRoiExtractor::pubImageCompressed(
    double timestamp,
    const std::string& frame_id,
    cv::Mat &image,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher,
    ImageCompression compression,
    int quality)
{
    if (!publisher)
    {
        RCLCPP_ERROR(get_logger(), "pubImageCompressed: publisher is null");
        return;
    }

    try
    {
        std::string encoding = inferImageEncoding(image);

        if (compression == ImageCompression::kNone)
        {
            // ── Raw / uncompressed path ─────────────────────────
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image)
                .toImageMsg(*msg);
            msg->header.frame_id = frame_id;
            msg->header.stamp = timeStampDoubleToTime(timestamp);
            publisher->publish(std::move(msg));
        }
        else
        {
            // ── Compressed path ─────────────────────────────────
            // Encode the cv::Mat into a compressed byte buffer,
            // then wrap it as a raw sensor_msgs::Image with
            // encoding set to "jpeg" or "png" so that subscribers
            // (e.g. rqt_image_view, RViz2) can decode transparently.
            std::vector<uchar> buf;
            std::vector<int> encode_params;
            std::string comp_encoding;
            std::string file_ext;

            if (compression == ImageCompression::kJpeg)
            {
                comp_encoding = "jpeg";
                file_ext = ".jpg";
                int q = (quality > 0) ? std::clamp(quality, 1, 100)
                                      : kDefaultJpegQuality;
                encode_params = {cv::IMWRITE_JPEG_QUALITY, q};
            }
            else  // kPng
            {
                comp_encoding = "png";
                file_ext = ".png";
                int level = (quality > 0) ? std::clamp(quality, 0, 9)
                                          : kDefaultPngCompression;
                encode_params = {cv::IMWRITE_PNG_COMPRESSION, level};
            }

            if (!cv::imencode(file_ext, image, buf, encode_params))
            {
                RCLCPP_ERROR(get_logger(), "Failed to encode image as %s",
                             comp_encoding.c_str());
                return;
            }

            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.frame_id = frame_id;
            msg->header.stamp    = timeStampDoubleToTime(timestamp);
            msg->encoding        = comp_encoding;
            // Per ROS convention for compressed data in Image messages:
            // height=1, width=step=data_size so subscribers can compute
            // the full buffer as step * height.
            msg->height          = 1;
            msg->width           = static_cast<uint32_t>(buf.size());
            msg->step            = static_cast<uint32_t>(buf.size());
            msg->data            = std::move(buf);
            publisher->publish(std::move(msg));
        }
    }
    catch (const cv::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "pubImageCompressed exception: %s", e.what());
    }
}

// ══════════════════════════════════════════════════════════════════════
// pubPointCloud (template)
// ══════════════════════════════════════════════════════════════════════

template <typename PointT>
void MaskPcRoiExtractor::pubPointCloud(
    const pcl::PointCloud<PointT>& cloud,
    const std::string& frame_id,
    double timestamp,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher)
{
    if (!publisher || cloud.empty()) return;

    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *msg);
    msg->header.frame_id = frame_id;
    msg->header.stamp = timeStampDoubleToTime(timestamp);
    publisher->publish(std::move(msg));
}

// Explicit instantiations for the point types used in this project.
template void MaskPcRoiExtractor::pubPointCloud<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>&, const std::string&, double,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr);

template void MaskPcRoiExtractor::pubPointCloud<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>&, const std::string&, double,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr);

template void MaskPcRoiExtractor::pubPointCloud<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>&, const std::string&, double,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr);

// ══════════════════════════════════════════════════════════════════════
// Helper
// ══════════════════════════════════════════════════════════════════════

std::string MaskPcRoiExtractor::inferImageEncoding(const cv::Mat& image)
{
    int type = image.type();

    if (type == CV_16UC1)  return sensor_msgs::image_encodings::MONO16;
    if (type == CV_32FC1)  return sensor_msgs::image_encodings::TYPE_32FC1;
    if (type == CV_8UC3)   return sensor_msgs::image_encodings::BGR8;
    if (type == CV_8UC1)   return sensor_msgs::image_encodings::MONO8;
    if (type == CV_8UC4)   return sensor_msgs::image_encodings::BGRA8;

    // Fallback: guess from channel count.
    return image.channels() > 1 ? sensor_msgs::image_encodings::BGR8
                                : sensor_msgs::image_encodings::MONO8;
}

} // namespace robot::mask_pc_roi_extractor
