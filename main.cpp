#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#include "utils.h"
#include "lidar_tools.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

int main (int argc, char** argv) {
    std::string pcd_dir = "../assets/pcd_files/";
    std::string seg_dir = "../assets/seg_files/"; // seg is segmentation
    std::string rgb_dir = "../assets/rgb_files/";
    
    std::string pcd_file_type = ".pcd";
    std::string seg_file_type = ".png";
    std::string rgb_file_type = ".png";

    utils::file_iterator rgb_iterator(rgb_dir, rgb_file_type);
    utils::file_iterator pcd_iterator(pcd_dir, pcd_file_type);
    utils::file_iterator seg_iterator(seg_dir, seg_file_type);

    std::shared_ptr <std::vector<std::string>> rgb_files = rgb_iterator.iterate();
    std::shared_ptr <std::vector<std::string>> pcd_files = pcd_iterator.iterate();
    std::shared_ptr <std::vector<std::string>> seg_files = seg_iterator.iterate();

    Eigen::Matrix3f K;

    K << 617.99617, 0.0, 323.00136, 
        0.0, 617.47651, 237.02399, 
        0.0, 0.0, 1.0;

    Eigen::Matrix<float, 3, 4> P;
    P << 624.93848, 0.0, 325.82961, 0.0, 
        0.0, 626.70953, 239.62017, 0.0, 
        0.0, 0.0, 1.0, 0.0;

    Eigen::Matrix4f transform;
    transform << -7.96325701e-04,  -9.99998415e-01,  -1.59265292e-03,  -4.28088135e-02,
             -7.95058187e-04,  1.59328604e-03,  -9.99998415e-01,   1.00100155e-01,
              9.99999367e-01, -7.95058187e-04,  -7.96325701e-04,  -3.99545296e-02,
              0.00000000e+00,  0.00000000e+00,   0.00000000e+00,   1.00000000e+00;
              
    std::cout << "Beginning Overlay" << std::endl;
    int image_index = 0;
    for (int i = 0; i < pcd_files->size(); i++) {
        if (i % 3 == 0)
            image_index = image_index;
        else 
            image_index++;

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_files->at(i), *current_cloud) == -1 ) {
            std::printf("Cannot read pcd file %s", pcd_files->at(i)); 
            return -1;
        }
        
        cv::Mat rgb_image = cv::imread(rgb_files->at(image_index));
        cv::Mat seg_image = cv::imread(seg_files->at(image_index));
        cv::resize(seg_image, seg_image, cv::Size(640, 480), cv::INTER_NEAREST);
        lidar_tools::truncate_lidar(current_cloud, transformed_cloud, 0.0, 10.0);
        lidar_tools::lidar_to_pixel(transformed_cloud, transform, K, rgb_image, seg_image);
    }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yeshas/ARC/PointPainting/1589958659796070.pcd", *cloud) == -1 ) {
    //     std::printf("Cannot read pcd file\n"); 
    //     return -1;
    // } 
    // std::cout << "Loaded" << cloud->width * cloud->height << std::endl;
    
    // for (size_t i = 0; i < cloud->points.size(); i++) {
    //     std::cout << " " << cloud->points[i].x 
    //               << " " << cloud->points[i].y  
    //               << " " << cloud->points[i].z << std::endl;
    // }

    return 0;
}

