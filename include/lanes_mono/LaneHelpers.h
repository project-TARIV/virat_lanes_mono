#ifndef LINE_LANES_LANEHELPERS_H
#define LINE_LANES_LANEHELPERS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <mutex>
#include <utility>
#include <opencv/cv.h>

template<typename T>
T getLaneParam(const std::string &key, T default_value) {
    T val;
    if (ros::param::get("lanes_mono/" + key, val))
        return val;
    return default_value;
}

cv::Scalar getCvScalar(std::vector<double> v);

/*
template int getParam<int>;
template double getParam<double>;
*/

class PCPublisher {
    ros::Publisher _pub;
    sensor_msgs::PointCloud2Ptr pc;

public:
    sensor_msgs::PointCloud2::_header_type *header{};

    PCPublisher(const ros::Publisher &pub);

    void clear_cloud();

    std::array<sensor_msgs::PointCloud2Iterator<float>, 3> get_iter(size_t size);

    void publish();
};

#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>

struct Helpers {
    struct Publishers {
        image_transport::Publisher masked;
        PCPublisher pc;
    } pub;

    struct CVParams {
        cv::Scalar white_lower, white_upper, barrel_lower, barrel_upper; // HSV range for color white.

        double rect_frac;

        uint8_t erosion_size, erosion_iter;
        cv::Mat erosion_element;

        uint8_t blur_size;

        bool publish_masked;
    } cv{
            getCvScalar(getLaneParam<std::vector<double> >("cv/lane_lower", {0, 0, 0})),
            getCvScalar(getLaneParam<std::vector<double> >("cv/lane_upper", {180, 40, 255})),

            getCvScalar(getLaneParam<std::vector<double> >("cv/barrel_lower", {0, 250, 0})),
            getCvScalar(getLaneParam<std::vector<double> >("cv/barrel_upper", {180, 255, 255})),

            (getLaneParam("cv/upper_mask_percent", 60) % 100) / 100.0,

            (uint8_t) getLaneParam("cv/erosion_size", 2),
            (uint8_t) getLaneParam("cv/erosion_iter", 1),
            cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(2 * 3 + 1, 2 * 3 + 1)),

            (uint8_t) getLaneParam("cv/blur_size", 7),
            getLaneParam("cv/publish_masked", true)
    };

    // For projecting the image onto the ground.
    image_geometry::PinholeCameraModel cameraModel;
    tf::TransformListener listener;

    bool dynamic_reconfigure{getLaneParam("dynamic_reconfigure", false)};

    std::mutex mutex;

    Helpers(Publishers pubs) : pub(std::move(pubs)), mutex() {}
};

#endif //LINE_LANES_LANEHELPERS_H
