#include <torch/script.h> // One-stop header.
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <memory>

int main(int argc, const char* argv[]) {

  torch::jit::script::Module module;
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load(argv[1]);
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    return -1;
  }
  // Create a vector of inputs.
  std::vector<torch::jit::IValue> inputs;
  // inputs.push_back(torch::ones({1, 3, 256, 512}));

  cv::Mat frame = cv::imread("/home/yeshas/ARC/PointPainting/assets/rgb_files/image_110.png");
  cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB); 
  cv::imshow("input", frame);
  cv::waitKey(1000);
  // const float mean[3] = { 0.485, 0.456 , 0.406}; // in RGB Format based on ImageNet
  // const float sd[3] = { 0.225, 0.225, 0.225};
  cv::Vec3f mean = cv::Vec3f(0.485, 0.456 , 0.406);
  cv::Vec3f std = cv::Vec3f(0.225, 0.225, 0.225);
  std::cout << "yo" << std::endl;
  frame.convertTo(frame, CV_32FC3);
  cv::resize(frame, frame, cv::Size(512, 256));
  cv::Mat input_image = cv::Mat::zeros(cv::Size(512, 256), CV_32FC3);
  cv::divide(frame, cv::Scalar(255, 255, 255), frame);
  cv::subtract(frame, cv::Scalar(0.485, 0.456 , 0.406), frame);
  cv::divide(frame, cv::Scalar(0.225, 0.225, 0.225), frame);
  for (int r = 0; r < 256; r++) {
    for (int c = 0; c < 512; c++) {
        // std::cout << frame.data[c + r * 512 + d * (512*256)] << " ";
        // input_image.data[c + r * 512 + d * (512*256)] = static_cast<float>((frame.data[d + c*3 + r*(3*512)]/255.0 - mean[d])/sd[d]);
        // std::cout <<frame.data[c + r * 512 + d * (512*256)] << " ";
        std::cout << frame.at<cv::Vec3f>(r,c) << " ";
    }
    std::cout << std::endl;
  }
  // frame = (frame - mean) / std;
  

  auto input = ((torch::from_blob(frame.data, {1, 256, 512, 3}).permute({0,3,1,2})));
  
  // Execute the model and turn its output into a tensor.
  at::Tensor output = module.forward({input}).toTensor();
  std::cout << output.size(0) << " " << output.size(1) << " " <<  output.size(2) << " " <<  output.size(3) << std::endl;
  output = output.argmax(1);
  std::cout << output.slice(2, 0, 5);
  std::cout << output.size(0) << " " << output.size(1) << " " <<  output.size(2) << std::endl;
  // std::cout << output.dtype() << std::endl;
  output = output.permute({1, 2, 0}).to(torch::kU8);
  std::cout << output.size(0) << " " << output.size(1) << " " <<  output.size(2) << std::endl;

  cv::Mat resultImg(256, 512, CV_8UC1);
  cv::Mat inference(256, 512, CV_8UC3);
  std::memcpy((void *) resultImg.data, output.data_ptr(), sizeof(torch::kU8) * output.numel());
  // for (int r = 0; r < 256; r++) {
    // for (int c = 0; c < 512; c++) {
        // std::cout << frame.data[c + r * 512 + d * (512*256)] << " ";
        // input_image.data[c + r * 512 + d * (512*256)] = static_cast<float>((frame.data[d + c*3 + r*(3*512)]/255.0 - mean[d])/sd[d]);
        // std::cout <<frame.data[c + r * 512 + d * (512*256)] << " ";
        // uint8_t val = (*(output[r][c][0]).data_ptr<int>());
        // resultImg.at<uchar>(r,c) = val * 255;
        // int val = int(resultImg.at<uchar>(r,c));
        // if (val == 2) {
        //   inference.at<cv::Vec3b>(r,c) = cv::Vec3b(255, 0, 0);
        // } else if (val == 5) {
        //   inference.at<cv::Vec3b>(r,c) = cv::Vec3b(255, 0, 255);
        // } else {
        //   inference.at<cv::Vec3b>(r,c) = cv::Vec3b(0, 255, 0);
        // }
        // std::cout << int(resultImg.at<uchar>(r,c)) << " ";
    // }
    // std::cout << std::endl;
  // }
  resultImg.convertTo(resultImg, CV_8UC3, (255.0/12.0));
  
  // int outputC = 3;
  // int outputH = 256;
  // int outputW = 512;
  // std::vector<uchar> argmax_result;
  // cv::Mat inference_img{cv::Size(outputW,outputH),CV_8UC1}; 
  // argmax_result.empty();
  // argmax_result.reserve(outputH*outputW);
  // for(int row_ind = 0; row_ind<outputH;row_ind++){
  //     for(int col_ind= 0 ; col_ind<outputW;col_ind++){
  //         int max_channel_ind = 0;
  //         float value = output_buffer[col_ind+row_ind*outputW];
  //         for(int c=1;c<outputC;c++){
  //             if(output_buffer[col_ind + row_ind*outputW+c*(outputH*outputW)]>value){
  //                 max_channel_ind = c;
  //                 value = output_buffer[col_ind + row_ind*outputW+c*(outputH*outputW)];
  //             }
  //         }
  //         argmax_result[col_ind + row_ind*outputW] = max_channel_ind;
  //         inference_img.at<uchar>(cv::Point(col_ind,row_ind)) = max_channel_ind*uchar(255/outputC);
  //     }
  // }
  // cv::cvtColor(resultImg, inference, cv::COLOR_GRAY2BGR);
  cv::applyColorMap(resultImg, resultImg, cv::COLORMAP_RAINBOW);
  for (int r = 0; r < 256; r++) {
    for (int c = 0; c < 512; c++) {
      std::cout << inference.at<cv::Vec3b>(r,c) << " ";
    }
    std::cout << std::endl;
  }
  cv::imshow("poop", resultImg);  
  cv::imshow("Infernece Result 1", inference);
  // cv::resize(output,output,original_img_size_,0,0,cv::INTER_NEAREST);
  // cv::imshow("Infernece Result",resultImg);
  cv::waitKey(0);
  //std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << '\n';
  std::cout << "ok\n";
}

