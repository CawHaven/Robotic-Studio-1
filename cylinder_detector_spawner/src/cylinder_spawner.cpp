/**
 * @file cylinder_spawner.cpp
 * @brief This node allows users to spawn cylindrical objects in Gazebo by clicking points in RViz, and it supports deleting previous cylinders.
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"  // For removing models in Gazebo
#include <sstream>
#include <cmath>

/**
 * @class CylinderSpawner
 * @brief A ROS 2 node that subscribes to clicked points to spawn or move cylindrical models in Gazebo, and deletes previously spawned models.
 */
class CylinderSpawner : public rclcpp::Node {
public:

    /**
     * @brief Constructor for the CylinderSpawner class.
     * Initializes the subscriptions, publishers, and service clients.
     */
    CylinderSpawner() : Node("cylinder_spawner") {
        // Subscribe to clicked point topic
        point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&CylinderSpawner::pointCallback, this, std::placeholders::_1));

        // Publisher to spawn an object in Gazebo
        sdf_pub_ = this->create_publisher<std_msgs::msg::String>("/gazebo/spawn_sdf_model", 10);

        // Service client to delete models in Gazebo
        delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("/gazebo/delete_entity");

        model_id_ = 0;
        current_model_name_ = "";
        cylinder_position_set_ = false;
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_; ///< Subscription to the clicked point topic.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sdf_pub_; ///< Publisher for sending the SDF model to spawn the cylinder in Gazebo.
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_; ///< Client for calling the delete service in Gazebo.

    int model_id_; ///< ID for the current cylinder model.
    bool cylinder_position_set_; ///< Flag indicating if the cylinder's position has been set.
    double cylinder_x_, cylinder_y_; ///< Coordinates for the cylinder's position.
    double cylinder_radius_ = 0.15; ///< The radius of the cylinder (30 cm diameter).
    std::string current_model_name_; ///< The name of the current cylinder model.

    /**
     * @brief Callback function that handles clicked points in RViz.
     * Spawns or repositions a cylinder at the clicked point in Gazebo.
     * 
     * @param point_msg The PointStamped message containing the clicked point's position.
     */
    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point_msg) {
        cylinder_x_ = point_msg->point.x;
        cylinder_y_ = point_msg->point.y;

        RCLCPP_INFO(this->get_logger(), "Received clicked point at x: %f, y: %f", cylinder_x_, cylinder_y_);

        if (cylinder_position_set_) {
            // If a cylinder is already placed, delete the old one before placing the new one
            removeOldCylinder();
        }

        // Place the new cylinder
        placeNewCylinder(cylinder_x_, cylinder_y_);
        cylinder_position_set_ = true;
    }

    /**
     * @brief Removes the currently placed cylinder from Gazebo.
     * Deletes the cylinder model that was previously spawned using the Gazebo DeleteEntity service.
     */
    void removeOldCylinder() {
        if (current_model_name_.empty()) return;

        // Wait for the delete entity service to be available
        while (!delete_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /gazebo/delete_entity service...");
        }

        // Create a request to delete the model
        auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        request->name = current_model_name_;  // The current model's name

        auto future_result = delete_client_->async_send_request(request);
        try {
            auto response = future_result.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Successfully removed cylinder: %s", current_model_name_.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to remove cylinder: %s", current_model_name_.c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    /**
     * @brief Places a new cylinder at the given position in Gazebo.
     * Publishes an SDF model of the cylinder to the Gazebo spawn topic.
     * 
     * @param x The x-coordinate of the cylinder's position.
     * @param y The y-coordinate of the cylinder's position.
     */
    void placeNewCylinder(double x, double y) {
        // Prepare the SDF model string
        std::string sdf_model = generateCylinderSDF(x, y);

        // Update the current model name
        current_model_name_ = "cylinder_model_" + std::to_string(model_id_++);

        // Publish the SDF model string to Gazebo
        std_msgs::msg::String sdf_msg;
        sdf_msg.data = sdf_model;
        sdf_pub_->publish(sdf_msg);

        RCLCPP_INFO(this->get_logger(), "New cylinder placed at x: %f, y: %f", x, y);
    }

    /**
     * @brief Generates the SDF (Simulation Description Format) model string for a cylinder.
     * Constructs an SDF string to define the cylinder's size, position, and visual properties.
     * 
     * @param x The x-coordinate of the cylinder's position.
     * @param y The y-coordinate of the cylinder's position.
     * @return std::string The SDF model string for the cylinder.
     */
    std::string generateCylinderSDF(double x, double y) {
        std::stringstream ss;
        double height = 1.0;  // Arbitrary height, adjust based on needs

        // Use the current model name to ensure it can be deleted later
        std::string model_name = current_model_name_;

        ss << "<sdf version='1.6'>"
           << "  <model name='" << model_name << "'>"
           << "    <pose>" << x << " " << y << " 0 0 0 0</pose>"  // Position in world
           << "    <link name='link'>"
           << "      <inertial>"
           << "        <mass>1.0</mass>"
           << "        <inertia>"
           << "          <ixx>0.1</ixx><ixy>0.0</ixy><ixz>0.0</ixz>"
           << "          <iyy>0.1</iyy><iyz>0.0</iyz>"
           << "          <izz>0.1</izz>"
           << "        </inertia>"
           << "      </inertial>"
           << "      <collision name='collision'>"
           << "        <geometry>"
           << "          <cylinder>"
           << "            <radius>" << cylinder_radius_ << "</radius>"
           << "            <length>" << height << "</length>"
           << "          </cylinder>"
           << "        </geometry>"
           << "      </collision>"
           << "      <visual name='visual'>"
           << "        <geometry>"
           << "          <cylinder>"
           << "            <radius>" << cylinder_radius_ << "</radius>"
           << "            <length>" << height << "</length>"
           << "          </cylinder>"
           << "        </geometry>"
           << "        <material>"
           << "          <ambient>0.0 0.0 1.0 1.0</ambient>"  // Color: blue
           << "          <diffuse>0.0 0.0 1.0 1.0</diffuse>"
           << "        </material>"
           << "      </visual>"
           << "    </link>"
           << "  </model>"
           << "</sdf>";

        return ss.str();
    }
};

/**
 * @brief Main function for the CylinderSpawner node.
 * Initializes the ROS 2 system, creates the CylinderSpawner node, and processes incoming data.
 * 
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the node.
 */
int main(int argc, char **argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
