/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

// #include "CustomMsg.h"  // Comment this out for now
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

        // Set topic filter
        rosbag2_storage::StorageFilter filter;
        filter.topics.push_back(lidar_topic);
        reader.set_filter(filter);

        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pcl_serialization;
        
        while (reader.has_next()) {
            auto bag_message = reader.read_next();
            
            if (bag_message->topic_name == lidar_topic) {
                // Handle standard PointCloud2 format
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                sensor_msgs::msg::PointCloud2 pcl_msg;
                pcl_serialization.deserialize_message(&serialized_msg, &pcl_msg);
                
                pcl::PointCloud<pcl::PointXYZ> temp_cloud;
                pcl::fromROSMsg(pcl_msg, temp_cloud);
                *cloud_input_ += temp_cloud;
            }
        }
        
        RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), "Loaded %ld points from the rosbag.", cloud_input_->size()); 
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif