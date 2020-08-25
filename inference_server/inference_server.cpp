#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#include "file_utils.h"
#include "torch.h"
#include "lidar_tools.h"
#include <opencv2/highgui.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include "dosCore/commandPacket.h"
#include "dosCore/wsServer.h"
#include "dosCore/wsRobotClient.h"

class InferenceServer {
    private:
        nlohmann::json j_in_;
        std::string json_path;
        int image_server_port;

        std::mutex inference_lock;
        std::shared_ptr<DosServer::WsServer> inference_server_handler;
        std::shared_ptr<torch_engine> engine;
        cv::Mat seg_image;

        cv::Mat infer(cv::Mat image);
    public:
        InferenceServer();
        void start_server();
        void listen();

};

int main (int argc, char** argv) {
    InferenceServer inf_server;
    // inf_server.start_server();
    inf_server.listen();

    return 0;
}

// int main (int argc, char** argv) {
InferenceServer::InferenceServer() {
    
    json_path = "../color_mapping.json";
    std::ifstream json_data(json_path);
    j_in_ = nlohmann::json::parse(json_data);

    // load from json_config
    image_server_port = 5555;

    engine = std::make_shared<torch_engine>(json_path);

    start_server();
    // listen();

}

void InferenceServer::start_server() {
    inference_server_handler = std::make_shared<DosServer::WsServer>(image_server_port);
    
    inference_server_handler->registerPktFn(
        DosClient::COMMAND_IMG_LIST_TYPE,
        [this](DosServer::WsServer& serverIn, DosClient::BaseCommandPacket* pktIn) {
            auto imgPkt = dynamic_cast<DosClient::ImageListCommandPacket*>(pktIn);
            if (imgPkt) {
                std::unique_ptr<DosClient::BaseCommandPacket> newPkt = std::make_unique<DosClient::ImageListCommandPacket>();
                {
                    std::lock_guard<std::mutex> scopeLock(inference_lock);
                    std::vector<uchar> imageIn;
                    std::vector<DosClient::ImageListCommandPacket::SubImage> imagePkt;
                    DosClient::ImageListCommandPacket::SubImage subImage;
                    std::vector<uchar> imageOut;
                    auto imgMsg = imgPkt->getImages();
                    imageIn = imgMsg[0].bytes;
                    // std::cout << imageIn << endl;
                    cv::Mat test(cv::Size(640, 480), CV_8UC3, imageIn.data());
                    // std::cout << "poop" << endl;
                    seg_image = infer(test);
                    imageOut.assign(seg_image.datastart, seg_image.dataend);
                    subImage.bytes = imageOut;
                    imagePkt.push_back(subImage);
                    dynamic_cast<DosClient::ImageListCommandPacket*>(newPkt.get())->setImage(imagePkt);
                    // std::cout << "poopy" << endl;
                }
                serverIn.addMsg(newPkt);
            }
        }
    );

    // inference_server_handler->registerPktFn(
    //     DosClient::COMMAND_MSG_TYPE,
    //     [this](DosServer::WsServer& serverIn, DosClient::BaseCommandPacket* pktIn) {
    //         if (auto msg = dynamic_cast<DosClient::MsgStrCommandPacket*>(pktIn)) {
    //             {
    //                 std::lock_guard<std::mutex> scopeLock(inference_lock);
    //                 std::cout << msg->getStrMsg() << endl;
    //             }
    //         }
    //     }
    // );

    std::thread server_thread([this]()-> void {
        inference_server_handler->startServer();
    });

    server_thread.detach();
}

void InferenceServer::listen() {
    // inference_server_handler->sendMsgPacket("listening");
    while (true)
        continue;
}

cv::Mat InferenceServer::infer(cv::Mat image) {
    // std::cout << "infering" << endl;
    return engine->run(image);
}