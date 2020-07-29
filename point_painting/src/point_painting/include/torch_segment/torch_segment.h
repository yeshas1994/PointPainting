#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <torch/script.h> // One-stop header.
#include <torch/torch.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <memory>
#include <mutex>

class TorchEngine {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    image_transport::ImageTransport it;
    image_transport::Publisher inference_pub_;
    image_transport::Subscriber rgb_sub_;

    cv::Mat rgb_image_;
    std::mutex rgb_mutex_;
    std_msgs::Header image_header_;
    std::string torch_engine_;
    std::string camera_topic_;
    std::vector<float> mean_;
    std::vector<float> std_;
    cv::Scalar rgb_mean_;
    cv::Scalar rgb_std_;
    bool use_cuda_;

public:
    TorchEngine(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void image_callback(const sensor_msgs::ImageConstPtr &image);
    void run_inference();
    void run_gpu_inference();
};
    
