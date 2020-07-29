#include <torch/script.h> // One-stop header.
#include <torch/torch.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>

class torch_engine {
    private:
        void preprocess();
        void inference();
        void postprocess();

        std::string torch_engine_;
        std::array<std::array<uchar, 3>, 12> color_map_{};
        cv::Mat rgb_image_, seg_image_, result_image_;
        cv::Scalar rgb_mean_, rgb_std_;
    public:
        torch_engine();
        cv::Mat run(cv::Mat& rgb_image);
};



torch_engine::torch_engine() {

    std::ifstream json_in("/home/yeshas/PointPainting/test_files/point_painting/color_mapping.json");
    nlohmann::json json_file = nlohmann::json::parse(json_in);
    int idx = 0;
    for (auto &array : json_file["color_map"]) {
      color_map_[idx] = {array[0], array[1], array[2]};
      idx++;
    }
    auto mean_array = json_file["mean"];
    auto std_array = json_file["std"];
    rgb_mean_ = cv::Scalar(mean_array[0], mean_array[1], mean_array[2]);
    rgb_std_ = cv::Scalar(std_array[0], std_array[1], std_array[2]);
    torch_engine_ = json_file["torch_engine"];
}

cv::Mat torch_engine::run(cv::Mat& rgb_image) {
    rgb_image_ = rgb_image.clone();
    seg_image_ = cv::Mat(cv::Size(512, 256), CV_8UC3, cv::Scalar(0, 0, 0));
    preprocess();
    inference();
    postprocess();

    return seg_image_;
}

void torch_engine::preprocess() {
    cv::cvtColor(rgb_image_, rgb_image_, cv::COLOR_BGR2RGB);
    rgb_image_.convertTo(rgb_image_, CV_32FC3);
    cv::resize(rgb_image_, rgb_image_, cv::Size(512, 256));
    // Normalize 
    cv::divide(rgb_image_, cv::Scalar(255, 255, 255), rgb_image_);
    cv::subtract(rgb_image_, rgb_mean_, rgb_image_);
    cv::divide(rgb_image_, rgb_std_, rgb_image_);
}

void torch_engine::inference() {

    torch::Device device(torch::kCPU);
    // try {
    //     device = torch::kCUDA;
    // } catch (const torch::Error &e) {
    //     device = torch::kCPU;
    // }

    torch::jit::script::Module module;
    
    module = torch::jit::load("/home/yeshas/PointPainting/test_files/torch_script/fcn32_torchscript.pt");
    module.to(device);
    

    auto input = ((torch::from_blob(rgb_image_.data, {1, 256, 512, 3}).permute({0, 3, 1, 2})));
    input = input.to(device);
    at::Tensor output = module.forward({input}).toTensor();
    output = output.argmax(1).permute({1, 2, 0}).to(torch::kU8).to(torch::kCPU); 
    //faster than memcpy (for some reason)
    uchar* ptr = reinterpret_cast<uchar*>(output.data_ptr());
    result_image_ = cv::Mat(cv::Size(512, 256), CV_8UC1, ptr);    
}


void torch_engine::postprocess() {

    for (int r = 0; r < 256; r++) {
	    for (int c = 0; c < 512; c++) {
            int color_map_index = result_image_.at<uchar>(cv::Point(c, r));
            std::array<uchar, 3> class_color = color_map_[color_map_index];
            seg_image_.at<cv::Vec3b>(cv::Point(c, r)) = cv::Vec3b(class_color[2], class_color[1], class_color[0]);
        }
    }

}

