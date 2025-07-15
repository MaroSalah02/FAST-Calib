/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include "CustomMsg.h"  // Uncomment this line
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>

using namespace std;
using namespace cv;

class DataPreprocess
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_;
    cv::Mat img_input_;

    DataPreprocess(Params &params)
        : cloud_input_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        string bag_path = params.bag_path;
        string image_path = params.image_path;
        string lidar_topic = params.lidar_topic;

        img_input_ = cv::imread(params.image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty()) 
        {
            std::string msg = "Loading the image " + image_path + " failed";
            RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), "%s", msg.c_str());
            return;
        }

        // Check if bag file exists
        std::fstream file_;
        file_.open(bag_path, ios::in);
        if (!file_) 
        {
            std::string msg = "Loading the rosbag " + bag_path + " failed";
            RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), "%s", msg.c_str());
            return;
        }
        file_.close();
        
        RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), "Loading the rosbag %s", bag_path.c_str());
        
        // ROS 2 rosbag reading
        rosbag2_cpp::readers::SequentialReader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        try {
            reader.open(storage_options, converter_options);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), "LOADING BAG FAILED: %s", e.what());
            return;
        }

        // Check available topics and their types
        auto topics = reader.get_all_topics_and_types();
        bool topic_found = false;
        std::string actual_topic_type;
        
        for (const auto& topic_info : topics) {
            RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                       "Available topic: %s, type: %s", 
                       topic_info.name.c_str(), topic_info.type.c_str());
            
            if (topic_info.name == lidar_topic) {
                actual_topic_type = topic_info.type;
                topic_found = true;
                break;
            }
        }
        
        if (!topic_found) {
            RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), 
                        "Topic %s not found in rosbag", lidar_topic.c_str());
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                   "Found topic %s with type: %s", 
                   lidar_topic.c_str(), actual_topic_type.c_str());

        //  Not only point cloud2
        // if (actual_topic_type != "sensor_msgs/msg/PointCloud2") {
        //     RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), 
        //                 "Expected sensor_msgs/msg/PointCloud2, but found: %s", 
        //                 actual_topic_type.c_str());
        //     return;
        // }
        
        // Set topic filter
        rosbag2_storage::StorageFilter filter;
        filter.topics.push_back(lidar_topic);
        reader.set_filter(filter);

        int message_count = 0;
        
        if (actual_topic_type == "sensor_msgs/msg/PointCloud2") {
            RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                       "Processing standard PointCloud2 messages...");
            // Handle standard PointCloud2 format
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pcl_serialization;
            
            while (reader.has_next()) {
                auto bag_message = reader.read_next();
                
                if (bag_message->topic_name == lidar_topic) {
                    try {
                        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                        sensor_msgs::msg::PointCloud2 pcl_msg;
                        pcl_serialization.deserialize_message(&serialized_msg, &pcl_msg);
                        
                        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
                        pcl::fromROSMsg(pcl_msg, temp_cloud);
                        *cloud_input_ += temp_cloud;
                        message_count++;
                        
                        if (message_count % 10 == 0) {
                            RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                                       "Processed %d messages, total points: %ld", 
                                       message_count, cloud_input_->size());
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), 
                                    "Error deserializing message %d: %s", message_count, e.what());
                        continue;
                    }
                }
            }
        } else if (actual_topic_type == "livox_ros_driver2/msg/CustomMsg") {
            RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                       "Processing Livox CustomMsg messages...");
            
            while (reader.has_next()) {
                auto bag_message = reader.read_next();
                
                if (bag_message->topic_name == lidar_topic) {
                    try {
                        // Access the raw serialized data
                        const auto& serialized_data = bag_message->serialized_data;
                        
                        // For now, let's try to convert this to a standard PointCloud2
                        // This is a workaround - you might need to adjust based on your actual data
                        
                        // Skip this message for now and log that we found it
                        message_count++;
                        
                        if (message_count % 10 == 0) {
                            RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                                       "Found %d CustomMsg messages (conversion not implemented yet)", 
                                       message_count);
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), 
                                    "Error processing CustomMsg %d: %s", message_count, e.what());
                        continue;
                    }
                }
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), 
                        "Unsupported topic type: %s", actual_topic_type.c_str());
            return;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                   "Loaded %ld points from %d messages in the rosbag.", 
                   cloud_input_->size(), message_count);
                   
        if (cloud_input_->size() == 0) {
            RCLCPP_WARN(rclcpp::get_logger("data_preprocess"), 
                       "No points loaded! Check your rosbag and topic configuration.");
        }
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif