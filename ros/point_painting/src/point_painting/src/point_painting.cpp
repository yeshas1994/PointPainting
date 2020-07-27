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

  rgb_mean_ = cv::Scalar(mean_[0], mean_[1], mean_[2]);
  rgb_std_ = cv::Scalar(std_[0] ,std_[1], std_[2]);

  std::ifstream json_in(json_path_);
  j_ = nlohmann::json::parse(json_in);
  
  
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
  lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("painted_cloud", 1);
  // inference_sub_ = it.subscribe(inference_topic_, 100, &PointPainting::segmentation_callback, this);
  // inference_sub_ = it.subscribe(inference_topic_, 100,
  //                               [this](const sensor_msgs::ImageConstPtr &image) {                            
  //                                 std::scoped_lock<std::mutex> lock(seg_mutex_);
  //                                 seg_image_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
  //                               });
  // rgb_sub_ = it.subscribe(camera_topic_, 100, 
  //                         [this](const sensor_msgs::ImageConstPtr &image) {                            
  //                           std::scoped_lock<std::mutex> lock(rgb_mutex_);
  //                           rgb_image_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
  //                           rgb_header_ = image->header;
  //                         });
  // lidar_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZ>>(velodyne_topic_, 1000, &PointPainting::lidar_callback, this);
  lidar_sub_.subscribe(nh_, velodyne_topic_, 1);
  image_sub_.subscribe(nh_, camera_topic_, 1);
  // inference_sub_.subscribe(nh_, inference_topic_, 1);
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

void PointPainting::callback(const sensor_msgs::ImageConstPtr &image, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_input) 
{
  ROS_INFO("start");
  // for headers use ros::Time::now();
  rgb_image_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
  cv::Mat painted_image = rgb_image_.clone();
  seg_image_ = run_inference();

  if (seg_image_.empty())
  {
    ROS_WARN("Segmentation Image not loaded");
    return;
  }

  if (rgb_image_.empty()) 
  {
    ROS_WARN("RGB Image not loaded");
    return;

  }

  
  // tf_listener.lookupTransform("d435_color_optical_frame", "world", ros::Time(0), velodyne_to_camera); // target , source , time, tf::StampedTransform
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr painted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<PointPainting::PtData> painted_points;
  cv::Mat buffer = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
  cv::Mat depth = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);

  pcl::PassThrough<pcl::PointXYZ> pass;                                                                                                                                                                          
  pass.setInputCloud(cloud_input);                                                                                                                                                                             
  pass.setFilterFieldName("x");                                                                                                                                                                                  
  pass.setFilterLimits(0.0, 5.0);                                                                                                                                                                          
  pass.filter(*cloud); 

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

    if (uv.x <= 0 || uv.x >= 640 || uv.y <= 0 || uv.y >= 480)
      continue; 

    cv::Vec3b class_color = seg_image_.at<cv::Vec3b>(uv);
    // get color_map from yaml or whatever
    int index = 0;
    int class_index = -1;
    for (auto &array : j_["color_map"]) {
      if (class_color == cv::Vec3b(array[0], array[1], array[2])) {
        class_index = index;
        // ROS_INFO("point found!!");
        break;
      }
      index++;
    }
    painted_points.push_back(PointPainting::PtData(point[0], point[1], point[2], class_index));
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

  std_msgs::Header image_header;
  image_header.frame_id = camera_frame_;
  image_header.stamp = ros::Time::now();
  // cv::imshow("rgb", painted_image);
  // cv::waitKey(100);
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
    // ROS_INFO_STREAM(torch_engine_);
    // std::scoped_lock<std::mutex> lock(rgb_mutex_);
    // torch::Device device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);
    torch::Device device(torch::kCPU);
    if (use_cuda_)
    {
      device = torch::kCUDA;
    } 

    torch::jit::script::Module module;
    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        module = torch::jit::load(torch_engine_);
        module.to(device);
    } catch (const c10::Error &e) {
        ROS_ERROR("Error Loading Model\n");
    }

    // Preprocess
    auto time = std::chrono::steady_clock::now();
    cv::cvtColor(rgb_image_, rgb_image_, cv::COLOR_BGR2RGB);
    rgb_image_.convertTo(rgb_image_, CV_32FC3);
    cv::resize(rgb_image_, rgb_image_, cv::Size(512, 256));
    // Normalize 
    cv::divide(rgb_image_, cv::Scalar(255, 255, 255), rgb_image_);
    cv::subtract(rgb_image_, rgb_mean_, rgb_image_);
    cv::divide(rgb_image_, rgb_std_, rgb_image_);

    //conversion of cv::Mat to at::Tensor
    auto input = ((torch::from_blob(rgb_image_.data, {1, 256, 512, 3}).permute({0, 3, 1, 2})));

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
    // free(output.data_ptr());


    // time = std::chrono::steady_clock::now();
    // std::memcpy((void*)result_image.data, output.data_ptr(), sizeof(torch::kU8) * output.numel());
    // auto mem_duration_2 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-time).count();
    // float t2 = mem_duration_2 / 1e9;
    // ROS_INFO_STREAM("MemCpy: " << t2);

    int idx = 0;
    for (auto &array : j_["color_map"]) {
      color_map[idx] = {array[0], array[1], array[2]};
      idx++;
    }
    // for (auto &array : color_map) 
    //   ROS_INFO("%d, %d, %d", array[0], array[1], array[2]);

    cv::Mat inference_image(cv::Size(512, 256), CV_8UC3, cv::Scalar(0, 0, 0));
    // ROS_INFO("row: %d, col: %d", result_image.rows, result_image.cols);
    for (int r = 0; r < 256; r++)
		{
			for (int c = 0; c < 512; c++) 
      {
        int color_map_index = result_image.at<uchar>(cv::Point(c, r));
        std::array<uchar, 3> class_color = color_map[color_map_index];
        inference_image.at<cv::Vec3b>(cv::Point(c, r)) = cv::Vec3b(class_color[2], class_color[1], class_color[0]);
      }
    }
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-time).count();
    float postprocess_time = duration / 1e9;
    ROS_INFO_STREAM("Postprocess Time: " << postprocess_time);


    // ROS_INFO("done");
    // result_image.forEach<uchar>(
    //   [inference_image, color_map](uchar &pixel, const int* position) mutable 
    //   {
    //     inference_image.at<cv::Vec3b>(cv::Point(position[0], position[1])) = cv::Vec3b(color_map[pixel][0], color_map[pixel][1], color_map[pixel][2]);
    //   }); 

    cv::resize(inference_image, inference_image, cv::Size(640, 480), cv::INTER_NEAREST);
    // cv::imshow("infer", inference_image);
    // cv::waitKey(100);
    
    return inference_image;
}

