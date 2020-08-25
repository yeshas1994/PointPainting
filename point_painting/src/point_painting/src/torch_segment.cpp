#include <ros/ros.h>
#include <torch_segment/torch_segment.h>


TorchEngine::TorchEngine(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
    nh_(nh),
    nh_private_(nh_private),
    it(ros::NodeHandle())
{
    nh_private_.getParam("serialized_engine", torch_engine_);
    nh_private_.getParam("mean", mean_);
    nh_private_.getParam("std", std_);
    nh_private_.getParam("use_cuda", use_cuda_);
    nh_private_.getParam("camera_topic", camera_topic_);
    nh_private_.getParam("json_path", json_path_);

    std::ifstream json_in(json_path_);
    j_ = nlohmann::json::parse(json_in);

    rgb_mean_ = cv::Scalar(mean_[0], mean_[1], mean_[2]);
    rgb_std_ = cv::Scalar(std_[0] ,std_[1], std_[2]);
    
    inference_pub_ = it.advertise("d435/color/inference_image", 1);
    // rgb_sub_ = it.subscribe(camera_topic_, 100, &TorchEngine::image_callback, this);
    // rgb_sub_ = it.subscribe(camera_topic_, 100, 
                            // [this](const sensor_msgs::ImageConstPtr &image) {
                            //     std::lock_guard<std::mutex> lock(rgb_mutex_);
                            //     rgb_image_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
                            //     image_header_ = image->header;
                            // });
    
    start_server();
}

void TorchEngine::image_callback(const sensor_msgs::ImageConstPtr &image) 
{   
    // save header
    image_header_ = image->header;
    rgb_image_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
    // run_inference(); 
}

void TorchEngine::start_server() {
    if (first_) {
        image_server_handler = std::make_shared<DosServer::WsServer>(image_serverPort);

        image_server_handler->registerPktFn(
            DosClient::COMMAND_IMG_LIST_TYPE,
            [this](DosServer::WsServer& serverIn, DosClient::BaseCommandPacket* cPkt) 
            {
                if (auto msgPacket = dynamic_cast<DosClient::ImageListCommandPacket*>(cPkt))
                {
                    // if (cMsg[0].label == "seg") {
                        std::unique_ptr<DosClient::BaseCommandPacket> newPkt = std::make_unique<DosClient::ImageListCommandPacket>();
                        {
                            // lock guard for image 
                            std::lock_guard<std::mutex> scopeLock(seg_lock);
                            std::vector<uchar> imageIn;
                            std::vector<DosClient::ImageListCommandPacket::SubImage> imagePkt;
                            DosClient::ImageListCommandPacket::SubImage subImage;
                            std::vector<uchar> imageOut;
                            auto cMsg = msgPacket->getImages();
                            imageIn = cMsg[0].bytes;
                            // image_data_pkt = cMsg;
                            cv::Mat test(cv::Size(640, 480), CV_8UC3, imageIn.data());
                            // memcpy(test.data, imageIn.data(), imageIn.size() * sizeof( uchar ));
                            // run inference here              
                            // ROS_INFO("inference");
                            // ROS_INFO_STREAM(test.at<cv::Vec3b>(124, 124));
                            seg = run_inference(test);
                            ROS_WARN("client failed to connect5");
                            imageOut.assign(seg.datastart, seg.dataend);
                            subImage.bytes = imageOut;
                            imagePkt.push_back(subImage);
                            ROS_WARN("client failed to connect7");
                            // ROS_INFO_STREAM(msgPacket->getStrMsg());
                            dynamic_cast<DosClient::ImageListCommandPacket*>(newPkt.get())->setImage(imagePkt);
                        }
                        serverIn.addMsg(newPkt);
                    // }
                }
            });
        
        std::thread server_thread([this]()-> void {
        ROS_WARN("client failed to connect8");
            image_server_handler->startServer();
        });

        // ROS_INFO_STREAM(image_data_pkt[0].bytes);
        server_thread.detach();
        first_ = false;
    } else {
        ROS_INFO("Say it aint sooooo");
    }
}

cv::Mat TorchEngine::run_inference(cv::Mat rgb_image_) 
{
    // ROS_INFO_STREAM(torch_engine_);
    // std::lock_guard<std::mutex> lock(rgb_mutex_);
    torch::Device device(use_cuda_ ? torch::kCUDA : torch::kCPU);
    torch::jit::script::Module module;
    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        module = torch::jit::load(torch_engine_);
        module.to(device);
    } catch (const c10::Error &e) {
        ROS_ERROR("Error Loading Model\n");
        return rgb_image_;
    }

    

    // cv::Mat input_image;
    // ROS_INFO("hi");
    cv::cvtColor(rgb_image_, rgb_image_, cv::COLOR_BGR2RGB);
    rgb_image_.convertTo(rgb_image_, CV_32FC3);
    cv::resize(rgb_image_, rgb_image_, cv::Size(512, 256));
    // Normalize 
    cv::divide(rgb_image_, cv::Scalar(255, 255, 255), rgb_image_);
    cv::subtract(rgb_image_, rgb_mean_, rgb_image_);
    cv::divide(rgb_image_, rgb_std_, rgb_image_);

    //conversion of cv::Mat to at::Tensor
    auto input = ((torch::from_blob(rgb_image_.data, {1, 256, 512, 3}).permute({0, 3, 1, 2})));
    input = input.to(device);
    // input = input.reshape({input.size(0), input.size(3), input.size(1), input.size(2)});
    // Execution
    /* TODO add cuda device transfer */ 
    at::Tensor output = module.forward({input}).toTensor();
    // ROS_INFO("JESUS");
    // ROS_INFO_STREAM(output.size(0) << " " << output.size(1) << " " << output.size(2));
    output = output.argmax(1); 
    output = output.permute({1, 2, 0}).to(torch::kCPU).to(torch::kU8);
    cv::Mat result_image(256, 512, CV_8UC1);
    cv::Mat inference_image(256, 512, CV_8UC3);
    std::memcpy((void *) result_image.data, output.data_ptr(), sizeof(torch::kU8) * output.numel());
    
    // // colormapping
    // result_image.convertTo(inference_image, CV_8UC3, (255.0/2.0));
    // cv::applyColorMap(inference_image, inference_image, cv::COLORMAP_RAINBOW);
    int idx = 0;
    for (auto &array : j_["color_map"]) {
      color_map_[idx] = {array[0], array[1], array[2]};
      idx++;
    }

    // cv::Mat inference_image(cv::Size(512, 256), CV_8UC3, cv::Scalar(0, 0, 0));

    for (int r = 0; r < 256; r++)
	{
		for (int c = 0; c < 512; c++) 
        {
            int color_map_index = result_image.at<uchar>(cv::Point(c, r));
            std::array<uchar, 3> class_color = color_map_[color_map_index];
            inference_image.at<cv::Vec3b>(cv::Point(c, r)) = cv::Vec3b(class_color[2], class_color[1], class_color[0]);
        }
    }
    // cv::imshow("infer", inference_image);
    // cv::waitKey(10);
    cv::resize(inference_image, inference_image, cv::Size(640, 480), cv::INTER_NEAREST);
    image_header_.stamp = ros::Time::now();
    sensor_msgs::ImagePtr image = cv_bridge::CvImage(image_header_, "bgr8", inference_image).toImageMsg();
    inference_pub_.publish(image);

    return inference_image;
}