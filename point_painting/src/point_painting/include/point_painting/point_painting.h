#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
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
#include <thread> 
#include <dosCore/wsRobotClient.h>
#include <dosCore/wsServer.h>
#include <dosCore/commandPacket.h>

#include <torch/script.h>
#include <torch/torch.h> 

class PointPainting {
public:
    struct PtData {
        public:
            PtData(float xIn, float yIn, float zIn, int idx)
            : x(xIn), y(yIn), z(zIn), idx(idx)
            {};
            PtData() = default;
            float x;
            float y;
            float z;
            int idx;
    };
public:
    PointPainting(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private); 
    void segmentation_callback(const sensor_msgs::ImageConstPtr &image);
    void debug(const sensor_msgs::ImageConstPtr &image);
    void lidar_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input);
    void callback(const sensor_msgs::ImageConstPtr &image, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input);
    pcl::PointXYZRGB get_colored_point(float x, float y, float z, int r, int g, int b);
    void filter_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input);
    void paint_points();
    void lidar_to_pixel(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
    cv::Mat run_inference();
    void create_costmap();
    void inflate_obstacle(const std::pair<float, float>& point);
    inline bool point_valid(const std::pair<float, float>& point);
    bool start_client();

private:
    void bresenham(int x1, int y1, int x2, int y2);
    void send_image();
    void load_images();

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    nlohmann::json j_;

    image_transport::Publisher painted_pts_pub_;
    image_transport::ImageTransport it;
    ros::Publisher lidar_pub_;
    ros::Publisher map_pub_;
    ros::Publisher occ_pub_;

    // tf not implemented yet
    tf::TransformListener tf_listener;
    tf::StampedTransform velodyne_to_camera; 
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> inference_sub_;
    message_filters::Subscriber< pcl::PointCloud<pcl::PointXYZ> > lidar_sub_;
    typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, pcl::PointCloud<pcl::PointXYZ> > sync_policy;
    typedef message_filters::Synchronizer<sync_policy> sync;
    boost::shared_ptr<sync> sync_;

    image_transport::Subscriber image_sub_debug_;

    std::string lidar_max_distance;
    std::string inference_engine;
    std::string json_path_;
    std::string env_;
    int path_index; // this is for freespace path (depends on env)
    std::string torch_engine_;
    std::vector<float> mean_;
    std::vector<float> std_;
    cv::Scalar rgb_mean_;
    cv::Scalar rgb_std_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    cv::Mat rgb_image_, seg_image_;
    std_msgs::Header rgb_header_, lidar_header_;
    // camera/transform for deprojection
    Eigen::Matrix3f K;
    Eigen::Matrix<float, 3, 4> P;
    Eigen::Matrix4f transform;

    std::vector<PointPainting::PtData> painted_points_;
    Eigen::MatrixXf costmap_;
    // std::vector<signed char> occ;
    std::vector<float> occ;
    std_msgs::Float32MultiArray costmap_kr_;
    nav_msgs::OccupancyGrid costmap_viz_;
    cv::Mat costmap_image_;
    std::string velodyne_topic_, camera_topic_, inference_topic_ , camera_frame_;
    bool use_cuda_;
    bool use_server_;
    bool first_ = true;
    int serverPort;
    std::mutex seg_lock;
    std::string serverAddress;
    std::shared_ptr<DosClient::WsRobotClient> client_handler;
    std::shared_ptr<std::vector<uchar>> image_in;
    std::shared_ptr<std::vector<uchar>> image_out;
    std::array<std::array<uchar, 3>, 12> color_map_{}; 
    float world_costmap_size_;
    float costmap_resolution_;
    float inflation_radius_;
    int costmap_size_;
    float size_;
};