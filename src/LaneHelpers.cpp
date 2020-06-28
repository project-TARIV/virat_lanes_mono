#include "../include/lanes_mono/LaneHelpers.h"
// #include <ros/ros.h>

PCPublisher::PCPublisher(const ros::Publisher &pub) : _pub(pub), header(nullptr) {

}

using pc_iter = sensor_msgs::PointCloud2Iterator<float>;

std::array<pc_iter, 3> PCPublisher::get_iter(size_t size) {
    pc->width = size;

    sensor_msgs::PointCloud2Modifier(*pc).setPointCloud2FieldsByString(1, "xyz");

    return {
            pc_iter(*pc, "x"),
            pc_iter(*pc, "y"),
            pc_iter(*pc, "z"),
    };
}

void PCPublisher::clear_cloud() {
    pc = boost::make_shared<sensor_msgs::PointCloud2>();
    header = &pc->header;

    pc->height = 1;
    pc->is_bigendian = false;
    pc->is_dense = false;
}

void PCPublisher::publish() {
    _pub.publish(pc);
}

cv::Scalar getCvScalar(std::vector<double> v) {
    v.resize(3);
    return {v[0], v[1], v[2]};
}
