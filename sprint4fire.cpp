#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

class ObjectFinderNode : public rclcpp::Node {
public:
    ObjectFinderNode() : Node("object_finder_node") {
        // Subscriptions
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ObjectFinderNode::cameraCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ObjectFinderNode::scanCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ObjectFinderNode::odomCallback, this, std::placeholders::_1));
        
        // Publisher
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/detected_object_marker", 10);
    }

private:
    // Variables for storing data from the odom and scan topics
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    
    // Color detection settings for RGB[247, 55, 24]
    cv::Scalar lower_bound_ = cv::Scalar(247 - 10, 55 - 10, 24 - 10);
    cv::Scalar upper_bound_ = cv::Scalar(247 + 10, 55 + 10, 24 + 10);
    
    // Callbacks for subscribed topics
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            
            // Process the image to find the color
            cv::Mat mask;
            cv::inRange(image, lower_bound_, upper_bound_, mask);

            // Find contours in the mask
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if (!contours.empty()) {
                // Get the largest contour assuming it's the object
                auto max_contour = *std::max_element(contours.begin(), contours.end(),
                                                     [](const auto& a, const auto& b) {
                                                         return cv::contourArea(a) < cv::contourArea(b);
                                                     });
                // Get the center of the object in image coordinates
                cv::Moments M = cv::moments(max_contour);
                if (M.m00 > 0) {
                    int cx = static_cast<int>(M.m10 / M.m00);
                    int cy = static_cast<int>(M.m01 / M.m00);

                    // Estimate object position based on laser scan and odometry data
                    locateObject(cx, cy);
                }
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
    }

    // Helper function to locate object and publish its position
    void locateObject(int cx, int cy) {
    if (!latest_scan_ || !latest_odom_) return;

    // Convert image coordinates (cx, cy) to an angle relative to the center of the camera image
    int image_width = 640; // Assuming image width; replace with actual width if known
    int image_center_x = image_width / 2;

    // Calculate the angle in radians of the object in the image relative to the camera's center
    float fov_horizontal = 1.0472; // Assuming a 60-degree FOV in radians (adjust if needed)
    float angle_to_object = ((float)(cx - image_center_x) / image_center_x) * (fov_horizontal / 2);

    // Now we need to match this angle to the LaserScan's angle increment
    int scan_size = latest_scan_->ranges.size();
    float scan_angle_increment = latest_scan_->angle_increment;
    float scan_angle_min = latest_scan_->angle_min;

    // Find the closest index in the LaserScan data
    int scan_index = static_cast<int>((angle_to_object - scan_angle_min) / scan_angle_increment);

    // Ensure the index is within the bounds of the LaserScan data
    if (scan_index >= 0 && scan_index < scan_size) {
        float object_distance = latest_scan_->ranges[scan_index];

        // Calculate the object's position in the map frame using odometry
        float robot_x = latest_odom_->pose.pose.position.x;
        float robot_y = latest_odom_->pose.pose.position.y;

        // Use the robot's orientation (yaw) to calculate the object's position
        double yaw = tf2::getYaw(latest_odom_->pose.pose.orientation);
        float object_x = robot_x + object_distance * std::cos(yaw + angle_to_object);
        float object_y = robot_y + object_distance * std::sin(yaw + angle_to_object);

        // Publish the detected object's position as a Marker
        publishMarker(object_x, object_y);
    } else {
        RCLCPP_WARN(this->get_logger(), "Object angle out of LaserScan range");
    }
}

    void publishMarker(float x, float y) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "detected_objects";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position of the marker
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Scale and color of the marker
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker_pub_->publish(marker);
    }

    // Subscribers and Publisher
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

// Main function
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectFinderNode>());
    rclcpp::shutdown();
    return 0;
}

