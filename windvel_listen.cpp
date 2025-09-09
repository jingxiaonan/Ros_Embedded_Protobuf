#include <ros/protobuffer_traits.h>
#include <ros/serialization_protobuffer.h>
#include <ros/ros.h>
#include "wind_vel.pb.h"  // Protobuf 生成的头

void callback(const sensor::WindSpeed& msg) {
    ROS_INFO("Received wind speed: %.2f m/s", msg.value_mps());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "airspeed_proto_listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/airspeed/filtered", 10, callback);
    ros::spin();
    return 0;
}
