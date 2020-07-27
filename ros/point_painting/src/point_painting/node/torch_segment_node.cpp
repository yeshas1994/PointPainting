#include <ros/ros.h>
#include <torch_segment/torch_segment.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "torch_segment");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    TorchEngine torch_engine(nh, nh_private);
    ros::spin();
    return 0;
}
