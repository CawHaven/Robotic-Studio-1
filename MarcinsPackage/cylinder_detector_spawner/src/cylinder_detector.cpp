/**
 * @file cylinder_detector.cpp
 * @brief This node detects cylindrical objects using LaserScan data and publishes markers to visualize them in RViz.
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/**
 * @class CylinderDetector
 * @brief A ROS 2 node that subscribes to LaserScan data to detect cylindrical objects and publishes visualization markers.
 */
class CylinderDetector : public rclcpp::Node {
public:

    /**
     * @brief Constructor for the CylinderDetector class.
     * Initializes the node, subscriptions, and publishers.
     */
    CylinderDetector() : Node("cylinder_detector") {
        // Subscribe to LaserScan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::laserCallback, this, std::placeholders::_1));

        // Publisher to mark the detected object
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/detected_cylinder_marker", 10);
        
        // Publisher for filtered LaserScan data
        filtered_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);

        // Initialize tf2 listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_; ///< Subscription to the LaserScan topic.
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; ///< Publisher for cylinder markers.
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub_; ///< Publisher for filtered LaserScan data.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; ///< Buffer for storing transforms.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; ///< Listener for receiving transform data.

     /**
     * @brief Callback function to process incoming LaserScan data.
     * Detects cylinders within a specified range and publishes a visualization marker.
     * 
     * @param scan_msg The LaserScan message received from the sensor.
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        const double cylinder_diameter = 0.30;  // Known diameter of the cylinder (30cm)
        const double cylinder_radius = cylinder_diameter / 2.0;
        const double min_distance = cylinder_radius - 0.05;
        const double max_distance = cylinder_radius + 0.05;

        // Prepare a copy of the laser scan data for filtering
        auto filtered_scan = *scan_msg;

        // Extract useful scan data (in polar form, ranges and angles)
        std::vector<double> angles;
        std::vector<double> distances;
        for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            double distance = scan_msg->ranges[i];
            if (distance >= scan_msg->range_min && distance <= scan_msg->range_max) {
                angles.push_back(angle);
                distances.push_back(distance);
            }
        }

        // Identify cylindrical object from scan data
        double object_x = 0.0, object_y = 0.0;
        bool cylinder_detected = false;
        for (size_t i = 0; i < distances.size(); ++i) {
            if (distances[i] >= min_distance && distances[i] <= max_distance) {
                double x = distances[i] * cos(angles[i]);
                double y = distances[i] * sin(angles[i]);

                object_x = x;
                object_y = y;
                cylinder_detected = true;

                // Exclude the cylinder points by setting the range to infinity
                filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
            }
        }

        if (cylinder_detected) {
            RCLCPP_INFO(this->get_logger(), "Cylinder detected at x: %f, y: %f", object_x, object_y);
            publishMarker(object_x, object_y);
        }

        // Publish the filtered LaserScan message
        filtered_scan_pub_->publish(filtered_scan);
    }

    /**
     * @brief Publishes a visualization marker for the detected cylindrical object in RViz.
     * 
     * @param x The x-coordinate of the detected cylinder in the robot's reference frame.
     * @param y The y-coordinate of the detected cylinder in the robot's reference frame.
     */
    void publishMarker(double x, double y) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";  // The frame of the detected object
        marker.header.stamp = this->now();
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the cylinder
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.5;  // Assuming the object is tall enough for the laser to see
        marker.pose.orientation.w = 1.0;

        // Set the dimensions of the marker
        marker.scale.x = 0.30;  // Diameter of the cylinder
        marker.scale.y = 0.30;
        marker.scale.z = 1.0;   // Height of the object

        // Set color to make it easily visible
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;   // Semi-transparent

        marker.lifetime = rclcpp::Duration::from_seconds(0);  // Marker remains indefinitely

        marker_pub_->publish(marker);
    }
};

/**
 * @brief Main function for the CylinderDetector node.
 * Initializes the ROS 2 system, creates the CylinderDetector node, and spins to process incoming data.
 * 
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the node.
 */
int main(int argc, char **argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
