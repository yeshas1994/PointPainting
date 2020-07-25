#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <boost/foreach.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <mutex>
#include <torch/script.h>
#include <torch/torch.h> 

class PointPainting {
public:
    struct PtData {
        public:
            PtData(float xIn, float yIn, float zIn, uint8_t classIn)
            : x(xIn), y(yIn), z(zIn), class_idx(classIn)
            {};

            PtData() = default;

            float x;
            float y;
            float z;
            uint8_t class_idx;
    };
public:
    PointPainting(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private); 
    void segmentation_callback(const sensor_msgs::ImageConstPtr &image);
    void image_callback(const sensor_msgs::ImageConstPtr &image);
    void lidar_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input);
    void callback(const sensor_msgs::ImageConstPtr &image, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input);
    pcl::PointXYZRGB get_colored_point(float x, float y, float z, int r, int g, int b);
    void lidar_to_pixel(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
    cv::Mat run_inference();


private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    nlohmann::json j_;

    image_transport::Publisher painted_pts_pub_;
    image_transport::ImageTransport it;
    ros::Publisher lidar_pub_;

    // tf not implemented yet
    tf::TransformListener tf_listener;
    tf::StampedTransform velodyne_to_camera; 
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> inference_sub_;
    message_filters::Subscriber< pcl::PointCloud<pcl::PointXYZ> > lidar_sub_;
    typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, pcl::PointCloud<pcl::PointXYZ> > sync_policy;
    typedef message_filters::Synchronizer<sync_policy> sync;
    boost::shared_ptr<sync> sync_;

    std::string lidar_max_distance;
    std::string inference_engine;
    std::string json_path_;
    std::string torch_engine_;
    std::vector<float> mean_;
    std::vector<float> std_;
    cv::Scalar rgb_mean_;
    cv::Scalar rgb_std_;

    // sensor_msgs::PointCloud2::Ptr lidar_scans;
    // std::vector<pcl::PointXYZ> lidar_points;
    std::mutex rgb_mutex_, seg_mutex_;
    cv::Mat rgb_image_, seg_image_;
    std_msgs::Header rgb_header_, lidar_header_;
    Eigen::Matrix3f K;
    Eigen::Matrix<float, 3, 4> P;
    Eigen::Matrix4f transform;
    std::string velodyne_topic_;
    std::string camera_topic_;
    std::string inference_topic_;
    std::string camera_frame_;
    bool use_cuda_;
    std::array<std::array<uchar, 3>, 12> color_map{}; 
    // point vectors buffers

};