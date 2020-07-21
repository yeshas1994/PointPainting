#include "point_painting/point_painting.h"
#include <Eigen/Eigen>
PointPainting::PointPainting(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : 
  nh_(nh),
  nh_private_(nh_private),
  it(ros::NodeHandle())
{
  // velodyne_sub_(nh_.subscribe("velodyne_points", 100, &PointPainting::lidar_callback, this));
  // image_sub_(it.subscribe("d435/color/image_raw", 100, &PointPainting::image_callback, this));
  nh_private_.getParam("velodyne_topic", velodyne_topic_);
  nh_private_.getParam("camera_topic", camera_topic_);
  nh_private_.getParam("inference_topic", inference_topic_);
  nh_private_.getParam("camera_frame", camera_frame_);
  nh_private_.getParam("json_path", json_path_);

  std::ifstream json_in(json_path_);
  j = nlohmann::json::parse(json_in);

  K << 617.99617, 0.0, 323.00136,  
    0.0, 617.47651, 237.02399, 
    0.0, 0.0, 1.0;

  P << 624.93848, 0.0, 325.82961, 0.0, 
    0.0, 626.70953, 239.62017, 
    0.0, 0.0, 0.0, 1.0, 0.0;

  transform << -7.96325701e-04,  -9.99998415e-01,  -1.59265292e-03,  -4.28088135e-02,
            -7.95058187e-04,  1.59328604e-03,  -9.99998415e-01,   1.00100155e-01,
            9.99999367e-01, -7.95058187e-04,  -7.96325701e-04,  -3.99545296e-02,
            0.00000000e+00,  0.00000000e+00,   0.00000000e+00,   1.00000000e+00;

  cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  painted_pts_pub_ = it.advertise("d435/color/painted_points", 1);
  inference_sub_ = it.subscribe(inference_topic_, 100, &PointPainting::segmentation_callback, this);
  rgb_sub_ = it.subscribe(camera_topic_, 100, &PointPainting::image_callback, this);
  lidar_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZ>>(velodyne_topic_, 1000, &PointPainting::lidar_callback, this);
  // lidar_sub_.subscribe(nh_, velodyne_topic_, 1);
  // image_sub_.subscribe(nh_, camera_topic_, 1);
  // inference_sub_.subscribe(nh_, inference_topic_, 1);
  // sync_.reset(new sync(sync_policy(1), image_sub_, inference_sub_, lidar_sub_));
  // sync_->registerCallback(boost::bind(&PointPainting::callback, this, _1, _2, _3));
}

// inference on rgb_image
void PointPainting::segmentation_callback(const sensor_msgs::ImageConstPtr &image) 
{
  seg_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
}
void PointPainting::image_callback(const sensor_msgs::ImageConstPtr &image) 
{
  rgb_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
  rgb_header = image->header;
}
void PointPainting::lidar_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input) 
{
  pcl::PassThrough<pcl::PointXYZ> pass;                                                                                                                                                                          
  pass.setInputCloud(cloud_input);                                                                                                                                                                             
  pass.setFilterFieldName("x");                                                                                                                                                                                  
  pass.setFilterLimits(0.0, 10.0);                                                                                                                                                                          
  pass.filter(*cloud); 
  lidar_to_pixel(cloud);
}

void PointPainting::callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::ImageConstPtr &segmentation_image, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input) 
{
  ROS_INFO("start");
  std_msgs::Header image_header = segmentation_image->header;
  rgb_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
  seg_image = cv_bridge::toCvCopy(segmentation_image, sensor_msgs::image_encodings::BGR8)->image;

  if (seg_image.empty())
  {
    ROS_WARN("Segmentation Image not loaded");
    return;
  }

  if (rgb_image.empty()) 
  {
    ROS_WARN("RGB Image not loaded");
    return;

  }

  // tf_listener.lookupTransform("d435_color_optical_frame", "world", ros::Time(0), velodyne_to_camera); // target , source , time, tf::StampedTransform

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;                                                                                                                                                                          
  pass.setInputCloud(cloud_input);                                                                                                                                                                             
  pass.setFilterFieldName("x");                                                                                                                                                                                  
  pass.setFilterLimits(0.0, 5.0);                                                                                                                                                                          
  pass.filter(*cloud); 

  cv::Mat buffer = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
  cv::Mat depth = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
  BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points) {
    cv::Point uv;
    Eigen::Vector4f point;
    Eigen::Vector4f rotated_point;
    // Tx & Ty are 0 
    point[0] = pt.x;
    point[1] = pt.y;
    point[2] = pt.z;
    point[3] = 1.0;

    rotated_point = transform * point;  
    uv.x = (K(0, 0) * rotated_point[0] + 0) / rotated_point[2] + K(0, 2);
    uv.y = (K(1, 1) * rotated_point[1] + 0) / rotated_point[2] + K(1, 2);

    if (uv.x <= 0 || uv.x >= 640 || uv.y <= 0 || uv.y >= 480)
      continue; 

    cv::Vec3b class_color = seg_image.at<cv::Vec3b>(uv);
    // get color_map from yaml or whatever
    painted_points.push_back(PointPainting::PtData(point[0], point[1], point[2], j["color_map"][0]));
    
    if (buffer.at<cv::Vec3b>(uv) != cv::Vec3b(0, 0, 0)) { 
      if (rotated_point[2] <  depth.at<uchar>(uv)) {
        ROS_INFO("Drawing");
        cv::circle(buffer, uv, 2, class_color, -1);
        cv::circle(rgb_image, uv, 2, class_color, -1);
        depth.at<uchar>(uv) = rotated_point[2];
      }
    } else {
      ROS_INFO("ReDrawing");
      cv::circle(buffer, uv, 2, class_color, -1);
      cv::circle(rgb_image, uv, 2, class_color, -1);
      depth.at<uchar>(uv) = rotated_point[2];
    }
  }
  
  sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(image_header, "bgr8", rgb_image).toImageMsg();
  painted_pts_pub_.publish(image_msg);
}

/*
   Function to transform lidar points to image and broadcast based on color
 */
void PointPainting::lidar_to_pixel(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
  // tf_listener.lookupTransform("d435_color_optical_frame", "world", ros::Time(0), velodyne_to_camera); // target , source , time, tf::StampedTransform

  ROS_INFO("start");
  if (seg_image.empty())
  {
    ROS_WARN("Segmentation Image not loaded");
    return;
  }

  cv::Mat buffer = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
  cv::Mat depth = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
  painted_points.reserve(cloud->points.size());
  BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points) {
    cv::Point uv;
    Eigen::Vector4f point;
    Eigen::Vector4f rotated_point;
    // Tx & Ty are 0 
    point[0] = pt.x;
    point[1] = pt.y;
    point[2] = pt.z;
    point[3] = 1.0;

    rotated_point = transform * point;  
    uv.x = (K(0, 0) * rotated_point[0] + 0) / rotated_point[2] + K(0, 2);
    uv.y = (K(1, 1) * rotated_point[1] + 0) / rotated_point[2] + K(1, 2);

    if (uv.x < 0 || uv.x >= 640 || uv.y < 0 || uv.y >= 480)
      continue; 


    cv::Vec3b class_color = seg_image.at<cv::Vec3b>(uv);
    // get color_map from yaml or whatever
    // find index based on color from color_map
    // int index = -1;
    // ROS_INFO_STREAM("yeshas");
    int index = 0;
    int class_index = -1;
    for (auto &array : j["color_map"]) {
      if (class_color == cv::Vec3b(array[0], array[1], array[2])) {
        class_index = index;
        break;
      }
      index++;
    }
    painted_points.push_back(PointPainting::PtData(point[0], point[1], point[2], class_index));
    
    if (buffer.at<cv::Vec3b>(uv) != cv::Vec3b(0, 0, 0)) { 
      if (rotated_point[2] <  depth.at<uchar>(uv)) {
        cv::circle(buffer, uv, 2, class_color, -1);
        cv::circle(rgb_image, uv, 2, class_color, -1);
        depth.at<uchar>(uv) = rotated_point[2];
      }
    } else {
      cv::circle(buffer, uv, 2, class_color, -1);
      cv::circle(rgb_image, uv, 2, class_color, -1);
      depth.at<uchar>(uv) = rotated_point[2];
    }
  }

  sensor_msgs::ImagePtr image = cv_bridge::CvImage(rgb_header, "bgr8", rgb_image).toImageMsg();
  painted_pts_pub_.publish(image);
}

