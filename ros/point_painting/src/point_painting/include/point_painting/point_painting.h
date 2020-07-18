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

class PointPainting {
public:
    struct PcData {
        PcData(float xIn, float yIn, float zIn, uint8_t classIn)
        : x(xIn), y(yIn), z(zIn), class_idx(classIn)
        {}

        uint8_t class_idx;
        float x;
        float y;
        float z;

    };

public:
    PointPainting(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private); 
    void segmentation_callback(const sensor_msgs::ImageConstPtr &image);
    void image_callback(const sensor_msgs::ImageConstPtr &image);
    void lidar_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input);
    void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::ImageConstPtr &segmentation_image, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input);
    void lidar_to_pixel(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);


private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    image_transport::Publisher painted_pts_pub_;
    image_transport::ImageTransport it;
    image_transport::Subscriber inference_sub_;
    image_transport::Subscriber rgb_sub_;
    ros::Subscriber lidar_sub_;

    tf::TransformListener tf_listener;
    tf::StampedTransform velodyne_to_camera;
    std::string lidar_max_distance;
    std::string inference_engine;

    // message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    // message_filters::Subscriber<sensor_msgs::Image> inference_sub_;
    // message_filters::Subscriber< pcl::PointCloud<pcl::PointXYZ> > lidar_sub_;
    // typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image, pcl::PointCloud<pcl::PointXYZ> > sync_policy;
    // typedef message_filters::Synchronizer<sync_policy> sync;
    // boost::shared_ptr<sync> sync_;

    // sensor_msgs::PointCloud2::Ptr lidar_scans;
    // std::vector<pcl::PointXYZ> lidar_points;
    cv::Mat rgb_image;
    std_msgs::Header rgb_header;
    cv::Mat seg_image;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    Eigen::Matrix3f K;
    Eigen::Matrix<float, 3, 4> P;
    Eigen::Matrix4f transform;
    std::string velodyne_topic_;
    std::string camera_topic_;
    std::string inference_topic_;
    std::string camera_frame_;
    std::vector<PcData> painted_points;
    // point vectors buffers

};