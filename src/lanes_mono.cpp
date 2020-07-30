#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <lanes_mono/LanesConfig.h>

#include "../include/lanes_mono/LaneHelpers.h"
//#include <lanes_mono/LaneHelpers.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

const char *topic_image = "image_rect_color";
const char *topic_camera_info = "camera_info";
const char *topic_masked = "image_masked";

const char *topic_pc = "points2";

// In rviz map/odom seems to be a true horizontal plane located at ground level.
const char *ground_frame = "odom";
const char *robot_frame = "base_footprint";

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    Eigen::VectorXd result = A.colPivHouseholderQr().solve(yvals);

    return result;
}

void process_image(const cv_bridge::CvImageConstPtr &cv_img, std::vector<cv::Point> &points,
                   const Helpers::CVParams &params, image_transport::Publisher *cv_pub);

void callback(const sensor_msgs::ImageConstPtr &msg,
              Helpers &helper) {

    try {
        tf::StampedTransform transform;
        helper.listener.lookupTransform(ground_frame, helper.cameraModel.tfFrame(), ros::Time(0), transform);

        std::vector<cv::Point> points;
        process_image(cv_bridge::toCvCopy(msg, "bgr8"), points, helper.cv, &helper.pub.masked);

        // PC Publishing
        helper.pub.pc.clear_cloud();
        //auto[x, y, z] = helper.pub.pc.get_iter(points.size());
        auto _pc_iters = helper.pub.pc.get_iter(points.size());
        auto &x = _pc_iters[0], &y = _pc_iters[1], &z = _pc_iters[2];
        helper.pub.pc.header->frame_id = ground_frame;
        helper.pub.pc.header->stamp = ros::Time::now();

        // Change the transform to a more useful form.
        const tf::Quaternion trans_rot = transform.getRotation();
        const cv::Vec3d trans_vec{trans_rot.x(), trans_rot.y(), trans_rot.z()};
        const double trans_sca = trans_rot.w();

        std::vector<double> ptsy;
        std::vector<double> ptsx;

        for (const auto &point : points) {
            // ___________ Ray is a vector that points from the camera to the pixel: __________
            // Its calculation is pretty simple but is easier to use the image_geometry package.
            /* Basically:
             * cv::Point3d ray_cameraModel_frame;
             * ray_cameraModel_frame.x = (uv_rect.x - cx() - Tx()) / fx();
             * ray_cameraModel_frame.y = (uv_rect.y - cy() - Ty()) / fy();
             * ray_cameraModel_frame.z = 1.0;
             * f is focal length
             */
            cv::Point3d ray_cameraModel_frame = helper.cameraModel.projectPixelTo3dRay(point);

            /* Note: the ray_cameraModel_frame is not in the same frame as the tf_frame: camera_left
             * Ray frame:
             * x points to the right of the image, y points down, z points inward
             *
             * camera_left frame:
             * x points in the image and y points up
             *
             * If you look at the camera from above:
             *
             * ray_cameraModel_frame: x <---(x) y  camera_left: z (.)---> y
             *             |                     |
             *             |                     |
             *             z                     x
             *
             * What we do is basically scale the ray_cameraModel_frame such that the end touches the ground (we know the lane points are actually on the ground.
             * Then we add those co-ords to the point cloud.
             * ___________ We are basically checking the coords where the ray_cameraModel_frame intersects the ground ________
             * This can later be extended for non planar surfaces by checking where the ray intersects a lidar generated 3d mesh
             */
            cv::Vec3d ray{ray_cameraModel_frame.z, -ray_cameraModel_frame.x, -ray_cameraModel_frame.y};

            // Rotate ray by using the transform.
            // Kinda black mathematics on v_p = q * v * q'
            // https://gamedev.stackexchange.com/a/50545/90578
            cv::Vec3d ray_p =
                    2.0 * trans_vec.dot(ray) * trans_vec +
                    (trans_sca * trans_sca - trans_vec.dot(trans_vec)) * ray +
                    2.0f * trans_sca * trans_vec.cross(ray);

            if (ray_p[2] == 0) // Should never happen
                continue;

            // Scale factor for ray so it touches the ground
            const double scale = transform.getOrigin().z() / ray_p[2];

            // Add ray to camera_left's origin
            *x = transform.getOrigin().x() - ray_p[0] * scale;
            *y = transform.getOrigin().y() - ray_p[1] * scale;
            *z = 0; // transform.getOrigin().z() - ray_p[2] * scale;

            ptsx.push_back(*x);
            ptsy.push_back(*y);

            // Pointcloud iterators
            ++x;
            ++y;
            ++z;
        }
        helper.pub.pc.publish();

        double*ptrx = &ptsx[0];
        Eigen::Map<Eigen::VectorXd> ptsx_transformed(ptrx, ptsx.size());

        double*ptry = &ptsy[0];
        Eigen::Map<Eigen::VectorXd> ptsy_transformed(ptry, ptsy.size());

        Eigen::VectorXd coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);

        std::cout << coeffs[0] << "\n" << coeffs[1] << "\n" << coeffs[2] << "\n" << coeffs[3] << "\n" << "---------"
                  << "\n";
    }
    catch (std::exception &e) { // Not a good idea....
        ROS_ERROR("Callback failed: %s", e.what());
    }
}

void process_image(const cv_bridge::CvImageConstPtr &cv_img, std::vector<cv::Point> &points,
                   const Helpers::CVParams &params, image_transport::Publisher *cv_pub) {
    cv::Mat hsv, blur, raw_mask, eroded_mask, masked, barrel_mask;

    // TODO: Should we just downscale image?
    cv::cvtColor(cv_img->image, hsv, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(hsv, blur, cv::Size(params.blur_size - 1, params.blur_size - 1), 0, 0);
    // Get white pixels
    cv::inRange(blur, params.white_lower, params.white_upper, raw_mask);

    // Flood Fill from the top of the mask to remove the sky in gazebo.
    cv::floodFill(raw_mask, cv::Point(raw_mask.cols / 2, 2), cv::Scalar(0));

    // Errors in projection increase as we approach the halfway point of the image:
    // Apply a mask to remove top 60%
    raw_mask(cv::Rect(0, 0, raw_mask.cols, (int) (raw_mask.rows * params.rect_frac))) = 0;

    // TODO: Very expensive; switch to laser scan
    std::vector<cv::Point> barrel_points; // Yellow points are part of barrel
    cv::inRange(blur, params.barrel_lower, params.barrel_upper, barrel_mask);
    cv::findNonZero(barrel_mask, barrel_points);
    if (!barrel_points.empty()) {
        int minx = barrel_mask.cols, maxx = 0;
        int maxy = 0;

        for (auto &v : barrel_points) {
            minx = std::min(minx, v.x);
            maxx = std::max(maxx, v.x);
            maxy = std::max(maxy, v.y);
        }

        if (minx < maxx)
            raw_mask(cv::Rect(minx, 0, maxx - minx, /*raw_mask.rows*/ maxy)) = 0;
    }

    cv::erode(raw_mask, eroded_mask, params.erosion_element, cv::Point(-1, -1), params.erosion_iter);

    cv_img->image.copyTo(masked, eroded_mask);

    if (params.publish_masked && cv_pub)
        cv_pub->publish(cv_bridge::CvImage(cv_img->header, cv_img->encoding, masked).toImageMsg());

    cv::findNonZero(eroded_mask, points);
}

// This allows us to change params of the node while it is running: uses cfg/lanes.cfg.
// Try running `rosrun rqt_reconfigure rqt_reconfigure` while node is running.
// This also auto loads any params initially set in the param server.
void dynamic_reconfigure_callback(const lanes_mono::LanesConfig &config, const uint32_t &level, Helpers &helper) {
    std::lock_guard<std::mutex> lock(helper.mutex);

    if (level & 1u << 0u) {
        ROS_INFO("Reconfiguring lanes params.");
        auto &params = helper.cv;

        params.white_lower = cv::Scalar(config.h_lower, config.s_lower, config.v_lower);
        params.white_upper = cv::Scalar(config.h_upper, config.s_upper, config.v_upper);
        params.erosion_size = config.erosion_size;
        params.erosion_iter = config.erosion_iter;
        params.erosion_element = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE,
                                                           cv::Size(2 * params.erosion_size + 1,
                                                                    2 * params.erosion_size + 1));
        params.blur_size = 2 * config.blur_size + 1;
        params.rect_frac = config.upper_mask_percent / 100.0;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lane");
    ros::NodeHandle nh;

    ROS_INFO("Started");

    // For receiving and publishing the images in an easy way.
    image_transport::ImageTransport imageTransport(nh);

    Helpers helper({imageTransport.advertise(topic_masked, 1),
                    {nh.advertise<sensor_msgs::PointCloud2>(topic_pc, 2)}});

    sensor_msgs::CameraInfo::ConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            topic_camera_info);
    helper.cameraModel.fromCameraInfo(camera_info);

    ros::NodeHandle cv_params{"lanes_mono/cv"};
    // https://github.com/ros-visualization/rqt_reconfigure/issues/4#issuecomment-599606006

    std::shared_ptr<dynamic_reconfigure::Server<lanes_mono::LanesConfig>> dyn_reconfigure_server;
    if (helper.dynamic_reconfigure) {
        // For the dynamic parameter reconfiguration. see the function dynamic_reconfigure_callback
        dyn_reconfigure_server = std::make_shared<dynamic_reconfigure::Server<lanes_mono::LanesConfig>>(cv_params);
        dynamic_reconfigure::Server<lanes_mono::LanesConfig>::CallbackType dynamic_reconfigure_callback_function = boost::bind(
                &dynamic_reconfigure_callback, _1, _2, boost::ref(helper));

        dyn_reconfigure_server->setCallback(dynamic_reconfigure_callback_function);
        ROS_INFO("Dynamic Reconfigure Ready");
    }

    image_transport::Subscriber sub = imageTransport.subscribe(topic_image, 2,
                                                               boost::bind(&callback, _1, boost::ref(helper)));

    ROS_INFO("Spinning...");
    ros::spin();
}
