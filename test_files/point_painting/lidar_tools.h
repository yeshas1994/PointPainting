#include <Eigen/Eigen>
#include <memory>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace lidar_tools {

void truncate_lidar(pcl::PointCloud<pcl::PointXYZ>::Ptr &current_cloud, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud,
                    const float x_start, const float x_end) 
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(current_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_start, x_end);
    //pass.setFilterLimitsNegative
    pass.filter(*transformed_cloud);
}

// K is camera matrix
void lidar_to_pixel(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
                Eigen::Matrix4f transform, Eigen::Matrix3f K,
                cv::Mat rgb, cv::Mat seg) {

    cv::Mat buffer = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
    cv::Mat depth = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
    for (size_t i; i < cloud->points.size(); i++) {
        cv::Point uv;
        Eigen::Vector4f point;
        Eigen::Vector4f rotated_point;
        // Tx & Ty are 0 
        point[0] = cloud->points[i].x;
        point[1] = cloud->points[i].y;
        point[2] = cloud->points[i].z;
        point[3] = 1.0;

        rotated_point = transform * point;

        uv.x = (K(0, 0) * rotated_point[0] + 0) / rotated_point[2] + K(0, 2);
        uv.y = (K(1, 1) * rotated_point[1] + 0) / rotated_point[2] + K(1, 2);
        
        if (uv.x < 0 || uv.x >= 640 || uv.y < 0 || uv.y >= 480)
            continue; 

        // std::cout << "Drawing on Buffer" << std::endl;
        // std::cout << "Point: " << uv << std::endl;
        // std::cout << buffer.at<cv::Vec3b>(uv) << std::endl;
        // std::cout << depth.at<uchar>(uv) << std::endl;
        cv::Vec3b class_color = seg.at<cv::Vec3b>(uv);
        // std::cout << class_color << std::endl;

        if (buffer.at<cv::Vec3b>(uv) != cv::Vec3b(0, 0, 0)) { 
            if (rotated_point[2] <  depth.at<uchar>(uv)) {
                // std::cout << "Replacing Point" << std::endl;
                cv::circle(buffer, uv, 2, class_color, -1);
                cv::circle(rgb, uv, 2, class_color, -1);
                depth.at<uchar>(uv) = rotated_point[2];
            }
        } else {
            // std::cout << "Creating New Point" << std::endl;
            cv::circle(buffer, uv, 2, class_color, -1);
            cv::circle(rgb, uv, 2, class_color, -1);
            depth.at<uchar>(uv) = rotated_point[2];
        }
    }

    cv::imshow("Lidar Buffer", buffer);
    cv::imshow("Rgb_Image", rgb);
    cv::imshow("Seg Image", seg);
    cv::waitKey(1);
}
}