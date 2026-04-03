#include <chrono>
#include <memory>
#include <map>
#include <cmath>
#include <vector>
#include <tuple>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

using namespace std::chrono_literals;

class CusCamera : public rclcpp::Node
{
public:
    CusCamera() : Node("cus_camera"), is_calibrated_(false), marker_length_(80), robot_marker_id_(69)
    {
        RCLCPP_INFO(this->get_logger(), "OpenCV version: %d", CV_VERSION_MAJOR);

        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        rclcpp::QoS qos(rclcpp::KeepLast(2));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", qos, std::bind(&CusCamera::imageCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("camera_odom", 1000);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Старый API: словарь и параметры как Ptr
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        detector_params_ = cv::aruco::DetectorParameters::create();

        int half = marker_length_ / 2;
        object_corners_ = {
            cv::Point3f(-half,  half, 0),
            cv::Point3f( half,  half, 0),
            cv::Point3f( half, -half, 0),
            cv::Point3f(-half, -half, 0)
        };

        // Карта маркеров доски (сохраняем для калибровки)
        board_markers_ = {
            {20, {{550, 550, 0}, {650, 550, 0}, {650, 650, 0}, {550, 650, 0}}},
            {21, {{2350, 550, 0}, {2450, 550, 0}, {2450, 650, 0}, {2350, 650, 0}}},
            {22, {{550, 1350, 0}, {650, 1350, 0}, {650, 1450, 0}, {550, 1450, 0}}},
            {23, {{2350, 1350, 0}, {2450, 1350, 0}, {2450, 1450, 0}, {2350, 1450, 0}}}
        };

        // Создаём объект доски (для совместимости, но не используем matchImagePoints)
        std::vector<std::vector<cv::Point3f>> obj_points;
        std::vector<int> ids;
        for (const auto& m : board_markers_) {
            ids.push_back(m.first);
            obj_points.push_back(m.second);
        }
        cv::aruco::Dictionary board_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        auto board_dict_ptr = cv::makePtr<cv::aruco::Dictionary>(board_dict);
        board_ = cv::aruco::Board::create(obj_points, board_dict_ptr, ids);

        // Матрица камеры и коэффициенты искажений (для симуляции)
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 761.80910110473633, 0., 960.,
                                                      0., 761.80913686752319, 540.,
                                                      0., 0., 1.);
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

        RCLCPP_INFO(this->get_logger(), "Node started");
    }

private:
    static geometry_msgs::msg::Quaternion quaternionFromEuler(double roll, double pitch, double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        return tf2::toMsg(q);
    }

    std::map<int, std::vector<cv::Point2f>> findMarkers(bool calibrate = false)
    {
        std::map<int, std::vector<cv::Point2f>> result;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> rejected;

        // Используем старый функциональный API
        cv::aruco::detectMarkers(image_, dictionary_, corners, ids, detector_params_, rejected);

        if (!ids.empty()) {
            for (size_t i = 0; i < ids.size(); ++i) {
                if (calibrate) {
                    result[ids[i]] = corners[i];
                } else {
                    cv::putText(image_, std::to_string(ids[i]),
                                corners[i][2],
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                    result[ids[i]] = corners[i];
                }
            }
        }
        return result;
    }

    void calibrate()
    {
        auto markers = findMarkers(true);
        if (markers.empty()) {
            RCLCPP_WARN(this->get_logger(), "No markers found for calibration");
            return;
        }

        // Собираем все 3D и 2D точки всех маркеров доски
        std::vector<cv::Point3f> all_obj_points;
        std::vector<cv::Point2f> all_img_points;

        for (const auto& m : markers) {
            int id = m.first;
            const auto& img_corners = m.second;
            auto it = board_markers_.find(id);
            if (it != board_markers_.end()) {
                const auto& obj_corners = it->second;
                for (size_t i = 0; i < img_corners.size(); ++i) {
                    all_obj_points.push_back(obj_corners[i]);
                    all_img_points.push_back(img_corners[i]);
                }
            }
        }

        if (all_obj_points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to match board points");
            return;
        }

        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(all_obj_points, all_img_points, camera_matrix_, dist_coeffs_,
                                    rvec, tvec, false, cv::SOLVEPNP_IPPE);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "solvePnP failed during calibration");
            return;
        }

        cv::Mat R_board;
        cv::Rodrigues(rvec, R_board);
        cv::Mat Rt(3, 4, CV_64F);
        R_board.copyTo(Rt(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(Rt(cv::Rect(3, 0, 1, 3)));

        cv::Mat T = camera_matrix_ * Rt;
        cv::Mat T_33 = T(cv::Rect(0, 0, 3, 3));
        cv::invert(T_33, r_);
        t_ = T.col(3);

        RCLCPP_INFO(this->get_logger(), "Calibration successful");
        RCLCPP_INFO_STREAM(this->get_logger(), "r:\n" << r_);
        RCLCPP_INFO_STREAM(this->get_logger(), "t: " << t_.t());

        setDefault();
        is_calibrated_ = true;
    }

    void setDefault()
    {
        auto markers = findMarkers();
        if (markers.find(robot_marker_id_) == markers.end()) {
            RCLCPP_ERROR(this->get_logger(), "Robot marker not found during setDefault");
            return;
        }
        double x, y, theta;
        std::tie(x, y, theta) = transform(markers[robot_marker_id_], 435.0);
        default_theta_ = theta;
        last_x_ = y;
        last_y_ = -x;
        last_theta_ = 0.0;
        last_time_ = this->now();
        last_time_w_ = last_time_;
        RCLCPP_INFO(this->get_logger(), "Default theta: %.5f", default_theta_);
        RCLCPP_INFO(this->get_logger(), "Start odometry: (%.5f, %.5f, %.5f)",
                    last_x_, last_y_, last_theta_);
    }

    std::tuple<double, double, double> transform(const std::vector<cv::Point2f>& corners, double z)
    {
        // Центр маркера в пикселях
        cv::Point2f center_px(0, 0);
        for (const auto& p : corners) {
            center_px.x += p.x;
            center_px.y += p.y;
        }
        center_px.x /= 4.0;
        center_px.y /= 4.0;
        cv::Mat center = (cv::Mat_<double>(3, 1) << center_px.x, center_px.y, 1.0);

        cv::Mat r_last_row = r_.row(2);
        double alpha_num = -z + r_last_row.dot(t_);
        double alpha_den = r_last_row.dot(center);
        double alpha = alpha_num / alpha_den;

        cv::Mat pose = r_ * (alpha * center - t_);
        double x = pose.at<double>(0);
        double y = pose.at<double>(1);

        y = 2000.0 - y;
        x /= 1000.0;
        y /= 1000.0;

        // Оценка поворота маркера
        std::vector<cv::Point3f> obj_pts = object_corners_;
        std::vector<cv::Point2f> img_pts = corners;
        cv::Mat rvec_marker, tvec_marker;
        cv::solvePnP(obj_pts, img_pts, camera_matrix_, dist_coeffs_,
                     rvec_marker, tvec_marker, false, cv::SOLVEPNP_IPPE_SQUARE);
        cv::Mat R_marker;
        cv::Rodrigues(rvec_marker, R_marker);
        double theta = std::atan2(R_marker.at<double>(0, 0), R_marker.at<double>(1, 0));

        return {x, y, theta};
    }

    void sendOdometry(double x, double y, double theta)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";

        double dt = (this->now() - last_time_).seconds();
        double dt_w = (this->now() - last_time_w_).seconds();

        double vel_x = (y - last_x_) / dt;
        double vel_y = (-x - last_y_) / dt;
        double vel_w = (theta - last_theta_) / dt_w;

        if (std::abs(vel_w) > 1.0) {
            RCLCPP_WARN(this->get_logger(), "High angular velocity: %.5f rad/s", vel_w);
            theta = last_theta_;
        } else {
            last_theta_ = theta;
            last_time_w_ = this->now();
        }

        odom.pose.pose.position.x = y;
        odom.pose.pose.position.y = -x;
        odom.pose.pose.position.z = 0.01;
        odom.pose.pose.orientation = quaternionFromEuler(0, 0, theta);

        odom.twist.twist.linear.x = vel_x;
        odom.twist.twist.linear.y = vel_y;
        odom.twist.twist.angular.z = vel_w;

        odom_pub_->publish(odom);

        RCLCPP_INFO(this->get_logger(), "Odometry: x=%.5f y=%.5f theta=%.5f deg, vel_w=%.5f",
                    y, -x, theta * 180.0 / M_PI, vel_w);

        last_x_ = y;
        last_y_ = -x;
        last_time_ = this->now();
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            image_ = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (!is_calibrated_) {
            calibrate();
        } else {
            try {
                auto markers = findMarkers();
                if (markers.find(robot_marker_id_) != markers.end()) {
                    auto [x, y, theta] = transform(markers[robot_marker_id_], 435.0);
                    sendOdometry(x, y, theta);
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                         "Robot marker %d not found", robot_marker_id_);
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Error during processing: %s", e.what());
            }
        }

        cv::imshow("Camera Image", image_);
        int key = cv::waitKey(1);
        if (key == 'q') {
            rclcpp::shutdown();
        }
    }

    // Члены класса
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    cv::Mat image_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Ptr<cv::aruco::Board> board_;
    std::vector<cv::Point3f> object_corners_;

    cv::Mat camera_matrix_, dist_coeffs_;
    bool is_calibrated_;
    cv::Mat r_, t_;
    double default_theta_;

    double last_x_, last_y_, last_theta_;
    rclcpp::Time last_time_, last_time_w_;

    const int marker_length_;
    const int robot_marker_id_;

    std::map<int, std::vector<cv::Point3f>> board_markers_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CusCamera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}