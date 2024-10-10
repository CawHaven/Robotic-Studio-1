/**
 * @file cylinder_detection_node.cpp
 * @brief Node for detecting cylinders in the laser scan data and displaying results on the ground truth and laser scan maps.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <optional>

/**
 * @class CylinderDetectionNode
 * @brief Node for detecting cylinders using laser scan data and displaying on an OpenCV window.
 */
class CylinderDetectionNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for CylinderDetectionNode.
     * Initializes all components, loads the map, spawns the cylinder, and starts laser scan subscription.
     */
    CylinderDetectionNode() : Node("cylinder_detection_node")
    {
        // Load ground truth map 
        ground_truth_map_ = cv::imread("/home/student/ros2_ws/src/package/src/ground_truth_map.pgm", cv::IMREAD_GRAYSCALE);
        if (ground_truth_map_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not load the ground truth map.");
            rclcpp::shutdown();
            return;
        }

        // Threshold and find contours to crop the actual room footprint
        cv::threshold(ground_truth_map_, binary_map_, 200, 255, cv::THRESH_BINARY_INV);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_map_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Get the bounding rectangle of the largest contour
        cv::Rect room_roi = cv::boundingRect(contours[0]);
        for (size_t i = 1; i < contours.size(); i++)
        {
            cv::Rect current_rect = cv::boundingRect(contours[i]);
            room_roi = room_roi | current_rect; // Expand to include all detected contours
        }

        // Crop the ground truth map to get only the actual room footprint
        ground_truth_map_cropped_ = ground_truth_map_(room_roi).clone();
        cv::cvtColor(ground_truth_map_cropped_, ground_truth_map_cropped_, cv::COLOR_GRAY2BGR); // Convert to color for visualization

        // Initialize the laser scan map with the same size as the cropped ground truth map
        laser_scan_map_ = cv::Mat::zeros(ground_truth_map_cropped_.size(), CV_8UC3);

        // Show initial maps
        cv::imshow("Ground Truth Map", ground_truth_map_cropped_);
        cv::imshow("Laser Scan Map", laser_scan_map_);
        cv::waitKey(100); // Short wait to ensure the window displays properly

        // Parameters for map resolution and origin
        map_resolution_ = 0.05; // Resolution of the map in meters/pixel
        map_origin_ = {-7.5, -6.0}; // Origin of the map in meters (bottom-left corner)

        // TF2 Buffer and Listener for transformation data
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Spawn the cylinder in Gazebo
        spawn_cylinder(0, 2.0); // Change x and y values to place the cylinder at different positions

        // Subscribe to the LaserScan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetectionNode::laser_callback, this, std::placeholders::_1));

        // Keep the window alive
        rclcpp::Rate rate(10);
        while (rclcpp::ok())
        {
            cv::imshow("Ground Truth Map", ground_truth_map_cropped_);
            cv::imshow("Laser Scan Map", laser_scan_map_);
            if (cv::waitKey(1) == 27) // Press 'Esc' key to exit
            {
                break;
            }
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

private:
    cv::Mat ground_truth_map_, ground_truth_map_cropped_, binary_map_, laser_scan_map_; /**< OpenCV matrices for the maps */
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_; /**< Subscription to laser scan data */
    std::optional<std::pair<int, int>> detected_cylinder_; /**< Coordinates of detected cylinder (if any) */

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; /**< Buffer for storing transforms */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; /**< Listener for transform data */

    double map_resolution_; /**< Resolution of the map in meters/pixel */
    std::pair<double, double> map_origin_; /**< Origin of the map in meters (bottom-left corner) */

    /**
     * @brief Callback function for processing laser scan data.
     * @param msg Shared pointer to the LaserScan message.
     */
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Received a laser scan message.");

        // Reset the laser scan map to clear old data
        laser_scan_map_ = cv::Mat::zeros(ground_truth_map_cropped_.size(), CV_8UC3);

        try
        {
            // Get the transformation from the laser frame to the map frame
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform("map", msg->header.frame_id, rclcpp::Time(0));

            // Extract the yaw angle from the quaternion
            tf2::Quaternion q(
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            float angle = msg->angle_min;

            for (size_t i = 0; i < msg->ranges.size(); ++i)
            {
                float range = msg->ranges[i];
                if (range > msg->range_min && range < msg->range_max)
                {
                    // Convert the laser scan point to the world coordinates (using the transform rotation)
                    float x = range * cos(angle);
                    float y = range * sin(angle);

                    // Apply the transformation to get the coordinates in the map frame
                    float map_x = transform_stamped.transform.translation.x + (x * cos(yaw) - y * sin(yaw));
                    float map_y = transform_stamped.transform.translation.y + (x * sin(yaw) + y * cos(yaw));

                    // Convert (map_x, map_y) in world coordinates to pixel coordinates on the laser scan map
                    int pixel_x = static_cast<int>((map_x - map_origin_.first) / map_resolution_);
                    int pixel_y = static_cast<int>((-map_y - map_origin_.second) / map_resolution_);

                    if (pixel_x >= 0 && pixel_x < laser_scan_map_.cols && pixel_y >= 0 && pixel_y < laser_scan_map_.rows)
                    {
                        cv::circle(laser_scan_map_, cv::Point(pixel_x, pixel_y), 1, cv::Scalar(255, 255, 255), -1);

                        // Check if this point is an obstacle on the ground truth map
                        if (ground_truth_map_cropped_.at<cv::Vec3b>(pixel_y, pixel_x) == cv::Vec3b(255, 255, 255))
                        {
                            // Check if the point is at least 0.5 meters away from any other black part of the ground truth map
                            if (is_clear_around(pixel_x, pixel_y))
                            {
                                // If it is detected as an obstacle by the laser scan but not present on the ground truth, mark it as a potential cylinder
                                if (!detected_cylinder_)
                                {
                                    detected_cylinder_ = {pixel_x, pixel_y};
                                    draw_cylinder_marker(pixel_x, pixel_y);

                                    // Print the global coordinates of the detected cylinder
                                    RCLCPP_INFO(this->get_logger(), "Cylinder detected and marked at pixel coordinates (%d, %d) on the map.", pixel_x, pixel_y);
                                    RCLCPP_INFO(this->get_logger(), "Global coordinates of detected cylinder: x = %.2f, y = %.2f", map_x, map_y);
                                }
                            }
                        }
                    }
                }
                angle += msg->angle_increment;
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }

        // Display updated laser scan map
        cv::imshow("Laser Scan Map", laser_scan_map_);
        cv::waitKey(1);
    }

    /**
     * @brief Checks if a point is clear from obstacles within a radius of 0.5 meters.
     * @param pixel_x X coordinate of the point in pixels.
     * @param pixel_y Y coordinate of the point in pixels.
     * @return True if the area around the point is clear, false otherwise.
     */
    bool is_clear_around(int pixel_x, int pixel_y)
    {
        // Check if there is any black part within 0.5m radius in the ground truth map
        int radius_in_pixels = static_cast<int>(0.5 / map_resolution_); // Convert 0.5m to pixels

        for (int dy = -radius_in_pixels; dy <= radius_in_pixels; ++dy)
        {
            for (int dx = -radius_in_pixels; dx <= radius_in_pixels; ++dx)
            {
                int new_x = pixel_x + dx;
                int new_y = pixel_y + dy;

                if (new_x >= 0 && new_x < ground_truth_map_cropped_.cols && new_y >= 0 && new_y < ground_truth_map_cropped_.rows)
                {
                    if (ground_truth_map_cropped_.at<cv::Vec3b>(new_y, new_x) == cv::Vec3b(0, 0, 0))
                    {
                        return false; // There is an obstacle too close
                    }
                }
            }
        }
        return true;
    }

    /**
     * @brief Draws a marker to indicate the detected cylinder on both laser scan and ground truth maps.
     * @param pixel_x X coordinate of the detected cylinder in pixels.
     * @param pixel_y Y coordinate of the detected cylinder in pixels.
     */
    void draw_cylinder_marker(int pixel_x, int pixel_y)
    {
        // Draw a marker on both laser scan map and ground truth map
        cv::circle(laser_scan_map_, cv::Point(pixel_x, pixel_y), 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(ground_truth_map_cropped_, cv::Point(pixel_x, pixel_y), 5, cv::Scalar(0, 0, 255), -1);
    }

    /**
     * @brief Spawns a cylinder at the specified coordinates in the Gazebo simulation environment.
     * @param x X coordinate of the cylinder in meters.
     * @param y Y coordinate of the cylinder in meters.
     */
    void spawn_cylinder(double x, double y)
    {
        // Create a client to call the `/spawn_entity` service in Gazebo
        auto client = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

        // Wait for the service to be available
        RCLCPP_INFO(this->get_logger(), "Waiting for Gazebo `/spawn_entity` service...");
        if (!client->wait_for_service(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Spawn entity service is not available after waiting.");
            return;
        }

        // Create the service request
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = "cylinder";
        request->xml = R"(
            <sdf version='1.6'>
                <model name='cylinder'>
                    <pose>0 0 0.25 0 0 0</pose>
                    <link name='link'>
                        <inertial>
                            <mass>1.0</mass>
                            <inertia>
                                <ixx>1.0</ixx>
                                <iyy>1.0</iyy>
                                <izz>1.0</izz>
                            </inertia>
                        </inertial>
                        <collision name='collision'>
                            <geometry>
                                <cylinder>
                                    <radius>0.15</radius>
                                    <length>0.5</length>
                                </cylinder>
                            </geometry>
                        </collision>
                        <visual name='visual'>
                            <geometry>
                                <cylinder>
                                    <radius>0.15</radius>
                                    <length>0.5</length>
                                </cylinder>
                            </geometry>
                            <material>
                                <ambient>1 0 0 1</ambient>
                            </material>
                        </visual>
                    </link>
                </model>
            </sdf>
        )";
        request->initial_pose.position.x = x;
        request->initial_pose.position.y = y;
        request->initial_pose.position.z = 0.0;
        request->reference_frame = "world";

        // Call the service asynchronously
        auto future = client->async_send_request(request);

        // Handle the response when it's ready
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

        // Check the result of the service call
        if (future.valid())
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Successfully spawned cylinder at (%f, %f).", x, y);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to spawn cylinder: %s", response->status_message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service to spawn cylinder.");
        }
    }
};

/**
 * @brief Main function to create and spin the CylinderDetectionNode.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit status.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
