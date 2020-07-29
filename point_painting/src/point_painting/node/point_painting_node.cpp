#include <ros/ros.h>
#include "point_painting/point_painting.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "point_painting");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    PointPainting point_painting(nh, nh_private);
    ros::spin();
    return 0;
}
