#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

//#include <image_transport/image_transport.hpp>

// Soglie HSV
inline const cv::Scalar RED_LO1(0,   120, 70), RED_HI1(10,  255, 255);
inline const cv::Scalar RED_LO2(170, 120, 70), RED_HI2(180, 255, 255);
inline const cv::Scalar BLUE_LO(100, 150, 50), BLUE_HI(130, 255, 255);
inline const cv::Scalar GREEN_LO(40,  70,  50), GREEN_HI(80, 255, 255);

struct Object3D {
    std::string name;
    cv::Point2f pixel;
    geometry_msgs::msg::Point world;
    bool found = false;
};

class PerceptionNode : public rclcpp::Node
{
public:
    geometry_msgs::msg::Point cube_pos_, obstacle_pos_, goal_pos_;
    bool positions_ready_ = false;
    bool mission_done_    = false;

    PerceptionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("perception_node", options)
    {
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fr3/depth_camera/image", 10,
            std::bind(&PerceptionNode::rgb_callback, this, std::placeholders::_1));
        points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/fr3/depth_camera/points", 10,
            std::bind(&PerceptionNode::points_callback, this, std::placeholders::_1));
        cube_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/cube_position", 10);
        obs_pub_  = this->create_publisher<geometry_msgs::msg::PointStamped>("/obstacle_position", 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/goal_position", 10);
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        RCLCPP_INFO(this->get_logger(), "PerceptionNode avviato");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cube_pub_, obs_pub_, goal_pub_;
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    sensor_msgs::msg::PointCloud2 last_cloud_;
    bool has_cloud_ = false;
    std::string cloud_frame_id_;

    // ── Controllo di stabilità: evita di congelare posizioni mentre
    //    l'oggetto sta ancora cadendo/rimbalzando dopo lo spawn ──
    geometry_msgs::msg::Point prev_cube_, prev_obstacle_, prev_goal_;
    bool has_prev_ = false;
    int  stable_count_ = 0;
    static constexpr int    REQUIRED_STABLE_FRAMES = 15;   // ~1.5s a 10Hz RGB
    static constexpr double STABLE_THRESHOLD       = 0.003; // 3 mm

    static double dist(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
    {
        double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        last_cloud_ = *msg;
        cloud_frame_id_ = msg->header.frame_id;
        has_cloud_ = true;
    }

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!has_cloud_) return;

        cv::Mat rgb;
        try {
            rgb = cv_bridge::toCvCopy(msg, "rgb8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "rgb cv_bridge: %s", e.what());
            return;
        }
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

        cv::Mat hsv;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

        Object3D cube, obstacle, goal;
        cube.name = "cube"; obstacle.name = "obstacle"; goal.name = "goal";

        cube.found     = segment_color(hsv, bgr, RED_LO1, RED_HI1, RED_LO2, RED_HI2, cube);
        obstacle.found = segment_color_single(hsv, bgr, BLUE_LO, BLUE_HI, obstacle);
        goal.found     = segment_color_single(hsv, bgr, GREEN_LO, GREEN_HI, goal);
        show_debug(bgr, cube, obstacle, goal);

        auto stamp = msg->header;
        if (cube.found)     publish_point(cube_pub_, cube.world, stamp);
        if (obstacle.found) publish_point(obs_pub_,  obstacle.world, stamp);
        if (goal.found)     publish_point(goal_pub_, goal.world, stamp);

        if (cube.found && obstacle.found && goal.found && !mission_done_) {

            if (!has_prev_) {
                prev_cube_ = cube.world;
                prev_obstacle_ = obstacle.world;
                prev_goal_ = goal.world;
                has_prev_ = true;
                stable_count_ = 0;
                return;
            }

            double d_cube     = dist(cube.world,     prev_cube_);
            double d_obstacle = dist(obstacle.world, prev_obstacle_);
            double d_goal     = dist(goal.world,     prev_goal_);

            prev_cube_ = cube.world;
            prev_obstacle_ = obstacle.world;
            prev_goal_ = goal.world;

            if (d_cube < STABLE_THRESHOLD &&
                d_obstacle < STABLE_THRESHOLD &&
                d_goal < STABLE_THRESHOLD) {
                stable_count_++;
            } else {
                stable_count_ = 0; // qualcosa si è ancora mosso: ricomincia il conteggio
            }

            if (stable_count_ >= REQUIRED_STABLE_FRAMES && !positions_ready_) {
                cube_pos_     = cube.world;
                obstacle_pos_ = obstacle.world;
                goal_pos_     = goal.world;
                positions_ready_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "Posizioni stabili -> cube(%.3f,%.3f,%.3f) obstacle(%.3f,%.3f,%.3f) goal(%.3f,%.3f,%.3f)",
                    cube_pos_.x, cube_pos_.y, cube_pos_.z,
                    obstacle_pos_.x, obstacle_pos_.y, obstacle_pos_.z,
                    goal_pos_.x, goal_pos_.y, goal_pos_.z);
            }
        }
    }

    bool segment_color(const cv::Mat &hsv, cv::Mat &bgr,
                       const cv::Scalar &lo1, const cv::Scalar &hi1,
                       const cv::Scalar &lo2, const cv::Scalar &hi2,
                       Object3D &obj)
    {
        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv, lo1, hi1, mask1);
        cv::inRange(hsv, lo2, hi2, mask2);
        cv::bitwise_or(mask1, mask2, mask);
        return extract_centroid(mask, bgr, obj);
    }

    bool segment_color_single(const cv::Mat &hsv, cv::Mat &bgr,
                               const cv::Scalar &lo, const cv::Scalar &hi,
                               Object3D &obj)
    {
        cv::Mat mask;
        cv::inRange(hsv, lo, hi, mask);
        return extract_centroid(mask, bgr, obj);
    }

    bool extract_centroid(const cv::Mat &mask, cv::Mat &bgr, Object3D &obj)
    {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
        cv::Mat clean;
        cv::morphologyEx(mask, clean, cv::MORPH_OPEN,  kernel);
        cv::morphologyEx(clean, clean, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(clean, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return false;

        auto largest = *std::max_element(contours.begin(), contours.end(),
            [](const auto &a, const auto &b){
                return cv::contourArea(a) < cv::contourArea(b);
            });
        if (cv::contourArea(largest) < 200.0) return false;

        cv::Moments M = cv::moments(largest);
        if (M.m00 == 0) return false;
        int u = static_cast<int>(M.m10 / M.m00);
        int v = static_cast<int>(M.m01 / M.m00);
        obj.pixel = {static_cast<float>(u), static_cast<float>(v)};

        cv::drawContours(bgr, std::vector<std::vector<cv::Point>>{largest},
                         -1, {0, 255, 255}, 2);
        cv::circle(bgr, {u, v}, 5, {0, 0, 255}, -1);

        if (!get_point_from_cloud(u, v, obj)) return false;

        return true;
    }

    // ── Legge il punto 3D (in camera frame) dalla point cloud al pixel (u,v) ──
    bool get_point_from_cloud(int u, int v, Object3D &obj)
    {
        if (last_cloud_.width == 0 || last_cloud_.height == 0) return false;
        if (u < 0 || u >= static_cast<int>(last_cloud_.width) ||
            v < 0 || v >= static_cast<int>(last_cloud_.height)) return false;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(last_cloud_, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(last_cloud_, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(last_cloud_, "z");

        size_t index = static_cast<size_t>(v) * last_cloud_.width + static_cast<size_t>(u);
        iter_x += index;
        iter_y += index;
        iter_z += index;

        float X = *iter_x, Y = *iter_y, Z = *iter_z;
        if (!std::isfinite(X) || !std::isfinite(Y) || !std::isfinite(Z)) return false;

        geometry_msgs::msg::PointStamped cam_pt, world_pt;
        cam_pt.header.frame_id = cloud_frame_id_;
        cam_pt.header.stamp    = this->now();
        cam_pt.point.x = X;
        cam_pt.point.y = Y;
        cam_pt.point.z = Z;

        try {
            tf_buffer_->transform(cam_pt, world_pt, "world",
                                  tf2::durationFromSec(0.2));
            obj.world = world_pt.point;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF2: %s", ex.what());
            obj.world.x = X;
            obj.world.y = Y;
            obj.world.z = Z;
        }
        return true;
    }

    void publish_point(
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr &pub,
        const geometry_msgs::msg::Point &pt,
        const std_msgs::msg::Header &header)
    {
        geometry_msgs::msg::PointStamped msg;
        msg.header = header;
        msg.point  = pt;
        pub->publish(msg);
    }

   void show_debug(const cv::Mat &bgr, const Object3D &cube,
                const Object3D &obstacle, const Object3D &goal)
 {
    // Finestra di debug disabilitata: cv::imshow/waitKey chiamati da un
    // thread diverso da quello principale causano crash intermittenti
    // con OpenCV HighGUI (bug noto GTK + multi-thread).
    (void)bgr; (void)cube; (void)obstacle; (void)goal;
 }
};