/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <filesystem>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "color.h"
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace cv;
using namespace pcl;

#define TARGET_NUM_CIRCLES 4
#define DEBUG 1
#define GEOMETRY_TOLERANCE 0.06

// Parameters structure
struct Params {
  double x_min, x_max, y_min, y_max, z_min, z_max;
  double fx, fy, cx, cy, k1, k2, p1, p2;
  double marker_size, delta_width_qr_center, delta_height_qr_center;
  double delta_width_circles, delta_height_circles, circle_radius;
  int min_detected_markers;
  string image_path;
  string bag_path;
  string lidar_topic;
  string output_path;
};

// Load parameters using ROS 2 parameter interface
Params loadParameters(std::shared_ptr<rclcpp::Node> node) {
  Params params;
  
  // Declare and get parameters with default values
  node->declare_parameter("fx", 1215.31801774424);
  node->declare_parameter("fy", 1214.72961288138);
  node->declare_parameter("cx", 1047.86571859677);
  node->declare_parameter("cy", 745.068353101898);
  node->declare_parameter("k1", -0.33574781188503);
  node->declare_parameter("k2", 0.10996870793601);
  node->declare_parameter("p1", 0.000157303079833973);
  node->declare_parameter("p2", 0.000544930726278493);
  node->declare_parameter("marker_size", 0.2);
  node->declare_parameter("delta_width_qr_center", 0.55);
  node->declare_parameter("delta_height_qr_center", 0.35);
  node->declare_parameter("delta_width_circles", 0.5);
  node->declare_parameter("delta_height_circles", 0.4);
  node->declare_parameter("min_detected_markers", 3);
  node->declare_parameter("circle_radius", 0.12);
  node->declare_parameter("image_path", std::string("/path/to/image.png"));
  node->declare_parameter("bag_path", std::string("/path/to/input.bag"));
  node->declare_parameter("lidar_topic", std::string("/livox/lidar"));
  node->declare_parameter("output_path", std::string("/media/psf/Home/FAST-Calib/bag_images"));
  node->declare_parameter("x_min", 1.5);
  node->declare_parameter("x_max", 3.0);
  node->declare_parameter("y_min", -1.5);
  node->declare_parameter("y_max", 2.0);
  node->declare_parameter("z_min", -0.5);
  node->declare_parameter("z_max", 2.0);

  // Get parameter values with error handling
  try {
    params.fx = node->get_parameter("fx").as_double();
    params.fy = node->get_parameter("fy").as_double();
    params.cx = node->get_parameter("cx").as_double();
    params.cy = node->get_parameter("cy").as_double();
    params.k1 = node->get_parameter("k1").as_double();
    params.k2 = node->get_parameter("k2").as_double();
    params.p1 = node->get_parameter("p1").as_double();
    params.p2 = node->get_parameter("p2").as_double();
    params.marker_size = node->get_parameter("marker_size").as_double();
    params.delta_width_qr_center = node->get_parameter("delta_width_qr_center").as_double();
    params.delta_height_qr_center = node->get_parameter("delta_height_qr_center").as_double();
    params.delta_width_circles = node->get_parameter("delta_width_circles").as_double();
    params.delta_height_circles = node->get_parameter("delta_height_circles").as_double();
    params.min_detected_markers = node->get_parameter("min_detected_markers").as_int();
    params.circle_radius = node->get_parameter("circle_radius").as_double();
    params.image_path = node->get_parameter("image_path").as_string();
    params.bag_path = node->get_parameter("bag_path").as_string();
    params.lidar_topic = node->get_parameter("lidar_topic").as_string();
    params.output_path = node->get_parameter("output_path").as_string();
    params.x_min = node->get_parameter("x_min").as_double();
    params.x_max = node->get_parameter("x_max").as_double();
    params.y_min = node->get_parameter("y_min").as_double();
    params.y_max = node->get_parameter("y_max").as_double();
    params.z_min = node->get_parameter("z_min").as_double();
    params.z_max = node->get_parameter("z_max").as_double();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Error loading parameters: %s", e.what());
    throw;
  }
  
  // Log loaded parameters for debugging
  RCLCPP_INFO(node->get_logger(), "Loaded parameters:");
  RCLCPP_INFO(node->get_logger(), "  Camera: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
              params.fx, params.fy, params.cx, params.cy);
  RCLCPP_INFO(node->get_logger(), "  Image path: %s", params.image_path.c_str());
  RCLCPP_INFO(node->get_logger(), "  Bag path: %s", params.bag_path.c_str());
  RCLCPP_INFO(node->get_logger(), "  LiDAR topic: %s", params.lidar_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "  Output path: %s", params.output_path.c_str());
  
  return params;
}

double computeRMSE(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, 
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2) 
{
    if (cloud1->size() != cloud2->size()) 
    {
      std::cerr << BOLDRED << "[computeRMSE] Point cloud sizes do not match, cannot compute RMSE." << RESET << std::endl;
      return -1.0;
    }

    double sum = 0.0;
    for (size_t i = 0; i < cloud1->size(); ++i) 
    {
      double dx = cloud1->points[i].x - cloud2->points[i].x;
      double dy = cloud1->points[i].y - cloud2->points[i].y;
      double dz = cloud1->points[i].z - cloud2->points[i].z;

      sum += dx * dx + dy * dy + dz * dz;
    }

    double mse = sum / cloud1->size();
    return std::sqrt(mse);
}

// 将 LiDAR 点云转换到 QR 码坐标系
void alignPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, const Eigen::Matrix4f &transformation) 
{
  output_cloud->clear();
  for (const auto &pt : input_cloud->points) 
  {
    Eigen::Vector4f pt_homogeneous(pt.x, pt.y, pt.z, 1.0);
    Eigen::Vector4f transformed_pt = transformation * pt_homogeneous;
    output_cloud->push_back(pcl::PointXYZ(transformed_pt(0), transformed_pt(1), transformed_pt(2)));
  }
}

void projectPointCloudToImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
  const Eigen::Matrix4f& transformation,
  const cv::Mat& cameraMatrix,
  const cv::Mat& distCoeffs,
  const cv::Mat& image,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud) 
{
  colored_cloud->clear();
  colored_cloud->reserve(cloud->size());

  // Undistort the entire image (preprocess outside if possible)
  cv::Mat undistortedImage;
  cv::undistort(image, undistortedImage, cameraMatrix, distCoeffs);

  // Precompute rotation and translation vectors (zero for this case)
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat zeroDistCoeffs = cv::Mat::zeros(5, 1, CV_32F);

  // Preallocate memory for projection
  std::vector<cv::Point3f> objectPoints(1);
  std::vector<cv::Point2f> imagePoints(1);

  for (const auto& point : *cloud) 
  {
    // Transform the point
    Eigen::Vector4f homogeneous_point(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f transformed_point = transformation * homogeneous_point;

    // Skip points behind the camera
    if (transformed_point(2) < 0) continue;

    // Project the point to the image plane
    objectPoints[0] = cv::Point3f(transformed_point(0), transformed_point(1), transformed_point(2));
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, zeroDistCoeffs, imagePoints);

    int u = static_cast<int>(imagePoints[0].x);
    int v = static_cast<int>(imagePoints[0].y);

    // Check if the point is within the image bounds
    if (u >= 0 && u < undistortedImage.cols && v >= 0 && v < undistortedImage.rows) 
    {
      // Get the color from the undistorted image
      cv::Vec3b color = undistortedImage.at<cv::Vec3b>(v, u);

      // Create a colored point and add it to the cloud
      pcl::PointXYZRGB colored_point;
      colored_point.x = transformed_point(0);
      colored_point.y = transformed_point(1);
      colored_point.z = transformed_point(2);
      colored_point.r = color[2];
      colored_point.g = color[1];
      colored_point.b = color[0];
      colored_cloud->push_back(colored_point);
    }
  }
}

void saveCalibrationResults(const Params& params, const Eigen::Matrix4f& transformation, 
     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud, const cv::Mat& img_input)
{
  if(colored_cloud->empty()) 
  {
    std::cerr << BOLDRED << "[saveCalibrationResults] Colored point cloud is empty!" << RESET << std::endl;
    return;
  }
  
  std::string outputDir = params.output_path;
  if (outputDir.back() != '/') outputDir += '/';

  // Create output directory if it doesn't exist
  std::filesystem::create_directories(outputDir);

  // Save calibration parameters
  std::ofstream outFile(outputDir + "calib_result.txt");
  if (outFile.is_open()) 
  {
    outFile << "# FAST-LIVO2 calibration format\n";
    outFile << "cam_model: Pinhole\n";
    outFile << "cam_width: " << img_input.cols << "\n";
    outFile << "cam_height: " << img_input.rows << "\n";
    outFile << "scale: 1.0\n";
    outFile << "cam_fx: " << params.fx << "\n";
    outFile << "cam_fy: " << params.fy << "\n";
    outFile << "cam_cx: " << params.cx << "\n";
    outFile << "cam_cy: " << params.cy << "\n";
    outFile << "cam_d0: " << params.k1 << "\n";
    outFile << "cam_d1: " << params.k2 << "\n";
    outFile << "cam_d2: " << params.p1 << "\n";
    outFile << "cam_d3: " << params.p2 << "\n";

    outFile << "\nRcl: [" << std::fixed << std::setprecision(6);
    outFile << std::setw(10) << transformation(0, 0) << ", " << std::setw(10) << transformation(0, 1) << ", " << std::setw(10) << transformation(0, 2) << ",\n";
    outFile << "      " << std::setw(10) << transformation(1, 0) << ", " << std::setw(10) << transformation(1, 1) << ", " << std::setw(10) << transformation(1, 2) << ",\n";
    outFile << "      " << std::setw(10) << transformation(2, 0) << ", " << std::setw(10) << transformation(2, 1) << ", " << std::setw(10) << transformation(2, 2) << "]\n";

    outFile << "Pcl: [";
    outFile << std::setw(10) << transformation(0, 3) << ", " << std::setw(10) << transformation(1, 3) << ", " << std::setw(10) << transformation(2, 3) << "]\n";

    outFile.close();
    std::cout << BOLDYELLOW << "[Result] Calibration results saved to " << BOLDWHITE << outputDir << "calib_result.txt" << RESET << std::endl;
  } 
  else
  {
    std::cerr << BOLDRED << "[Error] Failed to open calib_result.txt for writing!" << RESET << std::endl;
  }
  
  // Save colored point cloud
  if (pcl::io::savePCDFileASCII(outputDir + "colored_cloud.pcd", *colored_cloud) == 0) 
  {
    std::cout << BOLDYELLOW << "[Result] Saved colored point cloud to: " << BOLDWHITE << outputDir << "colored_cloud.pcd" << RESET << std::endl;
  } 
  else 
  {
    std::cerr << BOLDRED << "[Error] Failed to save colored point cloud to " << outputDir << "colored_cloud.pcd" "!" << RESET << std::endl;
  }
 
  // Save detection image
  if (cv::imwrite(outputDir + "qr_detect.png", img_input)) {
    std::cout << BOLDYELLOW << "[Result] Saved QR detection image to: " << BOLDWHITE << outputDir << "qr_detect.png" << RESET << std::endl;
  } else {
    std::cerr << BOLDRED << "[Error] Failed to save QR detection image!" << RESET << std::endl;
  }
}

void sortPatternCenters(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, pcl::PointCloud<pcl::PointXYZ>::Ptr v, const std::string& axis_mode = "camera") 
{
  // 0 -- 1
  // |    |
  // 3 -- 2
  if(pc->size() != 4) 
  {
    std::cerr << BOLDRED << "[sortPatternCenters] Number of " << axis_mode << " center points to be sorted is not 4." << RESET << std::endl;
    return;
  }
  if (v->empty()) {
    v->clear();
    v->reserve(4);
  }

  // Check axis mode
  bool is_lidar_mode = (axis_mode == "lidar");

  if (is_lidar_mode)
  {
    for (auto& point : pc->points) 
    {
      float x_new = -point.y;   // LiDAR Y → 相机 -X
      float y_new = -point.z;   // LiDAR Z → 相机 -Y
      float z_new = point.x;    // LiDAR X → 相机  Z

      point.x = x_new;
      point.y = y_new;
      point.z = z_new;
    }
  }

  // Transform points to polar coordinates
  pcl::PointCloud<pcl::PointXYZ>::Ptr spherical_centers(
  new pcl::PointCloud<pcl::PointXYZ>());
  int top_pt = 0;
  int index = 0;  // Auxiliar index to be used inside loop
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = pc->points.begin();
  pt < pc->points.end(); pt++, index++) 
  {
    pcl::PointXYZ spherical_center;
    spherical_center.x = atan2(pt->y, pt->x);  // Horizontal
    spherical_center.y =
    atan2(sqrt(pt->x * pt->x + pt->y * pt->y), pt->z);  // Vertical
    spherical_center.z =
    sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);  // Range
    spherical_centers->push_back(spherical_center);

    if (spherical_center.y < spherical_centers->points[top_pt].y) 
    {
      top_pt = index;
    }
  }

  // Compute distances from top-most center to rest of points
  vector<double> distances;
  for (int i = 0; i < 4; i++) {
    pcl::PointXYZ pt = pc->points[i];
    pcl::PointXYZ upper_pt = pc->points[top_pt];
    distances.push_back(sqrt(pow(pt.x - upper_pt.x, 2) +
        pow(pt.y - upper_pt.y, 2) +
        pow(pt.z - upper_pt.z, 2)));
  }

  // Get indices of closest and furthest points
  int min_dist = (top_pt + 1) % 4, max_dist = top_pt;
  for (int i = 0; i < 4; i++) {
    if (i == top_pt) continue;
    if (distances[i] > distances[max_dist]) {
      max_dist = i;
    }
    if (distances[i] < distances[min_dist]) {
      min_dist = i;
    }
  }

  // Second highest point shoud be the one whose distance is the median value
  int top_pt2 = 6 - (top_pt + max_dist + min_dist);  // 0 + 1 + 2 + 3 = 6

  // Order upper row centers
  int lefttop_pt = top_pt;
  int righttop_pt = top_pt2;

  if (spherical_centers->points[top_pt].x <
    spherical_centers->points[top_pt2].x) {
    int aux = lefttop_pt;
    lefttop_pt = righttop_pt;
    righttop_pt = aux;
  }

  // Swap indices if target is located in the pi,-pi discontinuity
  double angle_diff = spherical_centers->points[lefttop_pt].x -
  spherical_centers->points[righttop_pt].x;
  if (angle_diff > M_PI - spherical_centers->points[lefttop_pt].x) {
    int aux = lefttop_pt;
    lefttop_pt = righttop_pt;
    righttop_pt = aux;
  }

  // Define bottom row centers using lefttop == top_pt as hypothesis
  int leftbottom_pt = min_dist;
  int rightbottom_pt = max_dist;

  // If lefttop != top_pt, swap indices
  if (righttop_pt == top_pt) {
    leftbottom_pt = max_dist;
    rightbottom_pt = min_dist;
  }

  // Fill vector with sorted centers
  v->push_back(pc->points[lefttop_pt]);
  v->push_back(pc->points[righttop_pt]);
  v->push_back(pc->points[rightbottom_pt]);
  v->push_back(pc->points[leftbottom_pt]);

  if (is_lidar_mode) 
  {
    for (auto& point : v->points)
    {
      float x_new = point.z;  
      float y_new = -point.x; 
      float z_new = -point.y;  

      point.x = x_new;
      point.y = y_new;
      point.z = z_new;
    }
  }
}

// Square class for geometry validation
class Square {
private:
    std::vector<pcl::PointXYZ> _candidates;
    float _delta_width;
    float _delta_height;

public:
    Square(std::vector<pcl::PointXYZ> candidates, float delta_width, float delta_height)
        : _candidates(candidates), _delta_width(delta_width), _delta_height(delta_height) 
    {
        // Sort candidates if needed
        if (_candidates.size() == 4) {
            // Simple sorting by distance from origin
            std::sort(_candidates.begin(), _candidates.end(), 
                [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                    return (a.x*a.x + a.y*a.y + a.z*a.z) < (b.x*b.x + b.y*b.y + b.z*b.z);
                });
        }
    }

    bool is_valid() {
        if (_candidates.size() != 4) return false;
        
        // Check if the points form a rectangle with expected dimensions
        // Simple geometry check - you can make this more sophisticated
        float avg_width = 0, avg_height = 0;
        int width_count = 0, height_count = 0;
        
        for (size_t i = 0; i < _candidates.size(); ++i) {
            for (size_t j = i + 1; j < _candidates.size(); ++j) {
                float dx = _candidates[i].x - _candidates[j].x;
                float dy = _candidates[i].y - _candidates[j].y;
                float distance = sqrt(dx*dx + dy*dy);
                
                // Check if this distance matches expected width or height
                if (abs(distance - _delta_width) < GEOMETRY_TOLERANCE) {
                    avg_width += distance;
                    width_count++;
                } else if (abs(distance - _delta_height) < GEOMETRY_TOLERANCE) {
                    avg_height += distance;
                    height_count++;
                }
            }
        }
        
        // We should have at least 2 width measurements and 2 height measurements
        return (width_count >= 2 && height_count >= 2);
    }
};

#endif