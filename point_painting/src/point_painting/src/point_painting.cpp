#include "point_painting/point_painting.h"

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
  nh_private_.getParam("serialized_engine", torch_engine_);
  nh_private_.getParam("json_path", json_path_);
  nh_private_.getParam("use_cuda", use_cuda_);
  nh_private_.getParam("mean", mean_);
  nh_private_.getParam("std",  std_);
  nh_private_.getParam("costmap_size", world_costmap_size_); // in m
  nh_private_.getParam("costmap_resolution", costmap_resolution_);
  nh_private_.getParam("environment", env_);
  
  rgb_mean_ = cv::Scalar(mean_[0], mean_[1], mean_[2]);
  rgb_std_ = cv::Scalar(std_[0] ,std_[1], std_[2]);
  size_ = (world_costmap_size_ / costmap_resolution_);
  costmap_size_  = static_cast<int>(size_);

  if (env_ == "office")
    path_index = 1;
  else 
    path_index = 2;

  std::ifstream json_in(json_path_);
  j_ = nlohmann::json::parse(json_in);
  
  cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

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
  
  painted_pts_pub_ = it.advertise("d435/color/painted_points", 1);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid", 100);
  lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("painted_cloud", 1);
  lidar_sub_.subscribe(nh_, velodyne_topic_, 1);
  image_sub_.subscribe(nh_, camera_topic_, 1);
  sync_.reset(new sync(sync_policy(1), image_sub_, lidar_sub_));
  sync_->registerCallback(boost::bind(&PointPainting::callback, this, _1, _2));
}

pcl::PointXYZRGB PointPainting::get_colored_point(float x, float y, float z, int r, int g, int b) {
  pcl::PointXYZRGB point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.r = r;
  point.g = g;
  point.b = b;
  return point;
}

void PointPainting::filter_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input) {
  pcl::PassThrough<pcl::PointXYZ> pass;                                                                                                                                                                          
  pass.setInputCloud(cloud_input);                                                                                                                                                                             
  pass.setFilterFieldName("x");                                                                                                                                                                                  
  pass.setFilterLimits(0.0, world_costmap_size_);                                                                                                                                                                          
  pass.filter(*cloud_); 
}

void PointPainting::paint_points() {

  // Declare variables
  cv::Mat painted_image = rgb_image_.clone();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr painted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 
  cv::Mat buffer = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
  cv::Mat depth = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
  painted_points_.clear();
  painted_points_.reserve(cloud_->points.size());
  // Main loop 
  BOOST_FOREACH (const pcl::PointXYZ& pt, cloud_->points) {
    cv::Point uv;
    Eigen::Vector4f point;
    Eigen::Vector4f rotated_point;
    // Tx & Ty are 0 
    point[0] = pt.x;
    point[1] = pt.y;
    point[2] = pt.z;
    point[3] = 1.0;

    // get image coordinates 
    rotated_point = transform * point;  
    uv.x = (K(0, 0) * rotated_point[0] + 0) / rotated_point[2] + K(0, 2);
    uv.y = (K(1, 1) * rotated_point[1] + 0) / rotated_point[2] + K(1, 2);

    if (uv.x <= 0 || uv.x >= 640 || uv.y <= 0 || uv.y >= 480)
      continue; 

    // check color of segmentation for specific pixel
    cv::Vec3b class_color = seg_image_.at<cv::Vec3b>(uv);
    // ROS_INFO_STREAM(class_color);

    // get color_map and apply to respective classes
    int index = 0;
    for (auto &array : j_["color_map"]) {
      if (class_color == cv::Vec3b(array[2], array[1], array[0])) {
        PointPainting::PtData new_point = PointPainting::PtData(point[0], point[1], point[2], index);
        painted_points_.push_back(new_point);
        break;
      }
      index++;
    }

    // create new painted point if new pixel is closer than currently exisiting pixel
    if (buffer.at<cv::Vec3b>(uv) != cv::Vec3b(0, 0, 0)) { 
      if (rotated_point[2] <  depth.at<uchar>(uv)) {
        cv::circle(buffer, uv, 2, class_color, -1);
        cv::circle(painted_image, uv, 2, class_color, -1);
        depth.at<uchar>(uv) = rotated_point[2];
        pcl::PointXYZRGB color_point = get_colored_point(point[0], point[1], point[2], class_color[2], class_color[1], class_color[0]);
        painted_cloud->push_back(color_point);
      }
    } else {
      cv::circle(buffer, uv, 2, class_color, -1);
      cv::circle(painted_image, uv, 2, class_color, -1);
      depth.at<uchar>(uv) = rotated_point[2];
      pcl::PointXYZRGB color_point = get_colored_point(point[0], point[1], point[2], class_color[2], class_color[1], class_color[0]);
      painted_cloud->push_back(color_point);  
    }
  }
  

  // Publish topics
  std_msgs::Header image_header;
  image_header.frame_id = camera_frame_;
  image_header.stamp = ros::Time::now();
  sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(image_header, "bgr8", painted_image).toImageMsg();
  painted_pts_pub_.publish(image_msg);

  sensor_msgs::PointCloud2 rosCloud;
  pcl::toROSMsg(*painted_cloud, rosCloud);
  rosCloud.header.frame_id = "world";
  rosCloud.header.stamp = ros::Time::now();
  lidar_pub_.publish(rosCloud);

}


cv::Mat PointPainting::run_inference() 
{
    cv::Mat rgb_image = rgb_image_.clone();
    ROS_INFO("run inference");
    torch::Device device(torch::kCPU);
    if (use_cuda_)
    {
      device = torch::kCUDA;
    } 

    torch::jit::script::Module module;

    ROS_INFO_STREAM(torch_engine_);
    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        module = torch::jit::load(torch_engine_);
        module.to(device);
    } catch (const c10::Error &e) {
        ROS_ERROR("Error Loading Model\n");
    }

    // Preprocess
    auto time = std::chrono::steady_clock::now();
    cv::cvtColor(rgb_image, rgb_image, cv::COLOR_BGR2RGB);
    rgb_image.convertTo(rgb_image, CV_32FC3);
    cv::resize(rgb_image, rgb_image, cv::Size(512, 256));
    // Normalize 
    cv::divide(rgb_image, cv::Scalar(255, 255, 255), rgb_image);
    cv::subtract(rgb_image, rgb_mean_, rgb_image);
    cv::divide(rgb_image, rgb_std_, rgb_image);

    //conversion of cv::Mat to at::Tensor
    auto input = ((torch::from_blob(rgb_image.data, {1, 256, 512, 3}).permute({0, 3, 1, 2})));

    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-time).count();
    float preprocess_time = duration/1e9;
    ROS_INFO_STREAM("Preprocess: " << preprocess_time);
    input = input.to(device);

    //Inference 
    time = std::chrono::steady_clock::now();  
    
    at::Tensor output = module.forward({input}).toTensor();
    
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-time).count();
    float inference_time = duration/1e9;
    ROS_INFO_STREAM("Inference: " << inference_time);
    
    // PostProcess
    time = std::chrono::steady_clock::now();  
    
    output = output.argmax(1).permute({1, 2, 0}).to(torch::kU8).to(torch::kCPU); 
    //faster than memcpy (for some reason)
    uchar* ptr = reinterpret_cast<uchar*>(output.data_ptr());
    cv::Mat result_image(cv::Size(512, 256), CV_8UC1, ptr);

    // time = std::chrono::steady_clock::now();
    // std::memcpy((void*)result_image.data, output.data_ptr(), sizeof(torch::kU8) * output.numel());
    // auto mem_duration_2 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-time).count();
    // float t2 = mem_duration_2 / 1e9;
    // ROS_INFO_STREAM("MemCpy: " << t2);

    int idx = 0;
    for (auto &array : j_["color_map"]) {
      color_map_[idx] = {array[0], array[1], array[2]};
      idx++;
    }

    cv::Mat inference_image(cv::Size(512, 256), CV_8UC3, cv::Scalar(0, 0, 0));

    for (int r = 0; r < 256; r++)
		{
			for (int c = 0; c < 512; c++) 
      {
        int color_map_index = result_image.at<uchar>(cv::Point(c, r));
        std::array<uchar, 3> class_color = color_map_[color_map_index];
        inference_image.at<cv::Vec3b>(cv::Point(c, r)) = cv::Vec3b(class_color[2], class_color[1], class_color[0]);
      }
    }

    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-time).count();
    float postprocess_time = duration / 1e9;
    ROS_INFO_STREAM("Postprocess Time: " << postprocess_time);

    cv::resize(inference_image, inference_image, cv::Size(640, 480), cv::INTER_NEAREST);

    return inference_image;
}

void PointPainting::create_costmap() {
  costmap_.resize(costmap_size_, costmap_size_);
  costmap_.setZero();

  // transformations due to costmap_point being in opencv coordinates
  tf2::Quaternion q;
  q.setRPY(0, 0, -M_PI / 2.0f);
  q.normalize();

  costmap_viz_.header.frame_id = "map";
  costmap_viz_.info.width = costmap_size_;
  costmap_viz_.info.height = costmap_size_;
  costmap_viz_.info.resolution = costmap_resolution_;
  costmap_viz_.header.stamp.sec = ros::Time::now().sec;

  costmap_viz_.info.origin.position.z = 0;
  costmap_viz_.info.origin.position.x = 0;
  costmap_viz_.info.origin.position.y = 0;
  // costmap_viz_.info.origin.orientation.w = q[3];
  // costmap_viz_.info.origin.orientation.x = q[0];
  // costmap_viz_.info.origin.orientation.y = q[1];
  // costmap_viz_.info.origin.orientation.z = q[2];
  
  occ.resize(costmap_size_ * costmap_size_);
  std::fill(occ.begin(), occ.end(), 100);
  
  // costmap stuff
  costmap_image_ = cv::Mat(cv::Size(costmap_size_, costmap_size_), CV_8UC3);
  cv::Mat large_costmap;
  int robot_x = costmap_size_ / 2;
  int robot_y = costmap_size_ - 1;
  costmap_image_.setTo(cv::Vec3b(0, 0, 0));
  // robot location
  cv::circle(costmap_image_, cv::Point(robot_x, robot_y), 5, cv::Vec3b(155, 0, 255), -1);
  for (auto point : painted_points_) {
    std::pair<int, int> costmap_point;
    std::pair<int, int> grid_point;
    // only check for y since x axis was dealt with during filter
    if (point.y >= world_costmap_size_ / 2.0f || point.y <= -world_costmap_size_ / 2.0f) { 
      continue;
    }

    // geometry_msgs::Pose pc_point;
    // costmap_point.first = std::floor((-point.y + (world_costmap_size_ / 2.0f)) / costmap_resolution_);
    // costmap_point.second = std::floor((-point.x + world_costmap_size_) / costmap_resolution_);

    // pc_point.position.x = std::floor(point.x / costmap_resolution_);
    // pc_point.position.y = std::floor((point.y + 2.5) / costmap_resolution_);

    // grid_point.first = std::floor((-point.y + 2.5) / costmap_resolution_);
    // grid_point.second = std::floor((point.x) / costmap_resolution_);
    grid_point.first = std::floor((-point.y + 2.5) / costmap_resolution_);
    grid_point.second = std::floor(point.x / costmap_resolution_);
    if (costmap_point.first >= costmap_size_ || costmap_point.second >= costmap_size_)
      continue;
    // ROS_INFO_STREAM("map index " << grid_point << "world index " << point.y << " " << point.x);
    if (point.idx == path_index) { // 1 for office & 2 for park
      // continue;
      costmap_(costmap_point.first, costmap_point.second) = 1;
      occ.at(grid_point.first + grid_point.second * costmap_size_) = 1;
      // bresenham(grid_point.first, grid_point.second, robot_x, robot_y);
      costmap_image_.at<cv::Vec3b>(cv::Point(costmap_point.first, costmap_point.second)) = cv::Vec3b(0, 255, 0);
    } else {
      costmap_(costmap_point.first, costmap_point.second) = 10;
      std::array<uchar, 3> class_color = color_map_[point.idx];
      occ.at(grid_point.first + grid_point.second * costmap_size_) = 100;
      costmap_image_.at<cv::Vec3b>(cv::Point(costmap_point.first, costmap_point.second)) \
            = cv::Vec3b(class_color[2], class_color[1], class_color[0]);
    }
  }
  
  costmap_viz_.data = occ;
  map_pub_.publish(costmap_viz_);
  cv::resize(costmap_image_, large_costmap, cv::Size(500,500), cv::INTER_LINEAR);
  cv::imshow("Obstacle Costmap", large_costmap);
  cv::waitKey(10);
}

void PointPainting::bresenham(int x1, int y1, int x2, int y2) {

  bool steep = std::abs(y2 - y1) > std::abs(x2 - x1);
  if (steep) {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  if (x1 > x2) {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  const int dx = std::abs(x2 - x1);
  const int dy = std::abs(y2 - y1);
  float error = dx / 2;
  const int ystep = (y1 < y2) ? 1 : -1;
  
  int y = y1;
  
  for (int x = x1; x <= x2; x++) 
  { 
    if (steep) {
      // if (costmap_(y, x) == 0) {
      //   costmap_(y, x) = 1;
      //   costmap_image_.at<cv::Vec3b>(cv::Point(y, x)) = cv::Vec3b(255, 255, 255);  
        occ.at(y * costmap_size_ + x) = 1;
      // }
    } else {
      // if (costmap_(x, y) == 0) {
      //   costmap_(x, y) = 1;
      //   costmap_image_.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(255, 255, 255);  
        occ.at(x * costmap_size_ + y) = 1;
      // }
    }
    // Add slope to increment angle formed 
    error -= dy;

    // Slope error reached limit, time to 
    // increment y and update slope error. 
    if (error < 0) 
    { 
        y += ystep; 
        error += dx;
    } 
  } 
}

void PointPainting::callback(const sensor_msgs::ImageConstPtr &image, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input) 
{
  ROS_INFO("start");
  // for headers use ros::Time::now();
  rgb_image_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;


  if (rgb_image_.empty()) 
  {
    ROS_WARN("RGB Image not loaded");
    return;

  }
  seg_image_ = run_inference();
  
  if (seg_image_.empty())
  {
    ROS_WARN("Segmentation Image not loaded");
    return;
  }

  filter_pointcloud(cloud_input);
  paint_points();
  create_costmap();
}
