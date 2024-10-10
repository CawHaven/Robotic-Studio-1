#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <math.h>
 
class LocalizationErrorCalculator : public rclcpp::Node
{
public:
    LocalizationErrorCalculator()
        : Node("localization_error_calculator")
    {
        // Subscribe to the /amcl_pose topic
        amcl_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&LocalizationErrorCalculator::amclCallback, this, std::placeholders::_1));
 
        // Subscribe to the /gazebo/model_states topic
        gt_subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 10, std::bind(&LocalizationErrorCalculator::gtCallback, this, std::placeholders::_1));
    }
 
private:
    geometry_msgs::msg::Pose amcl_pose_;  // Store the AMCL pose
    geometry_msgs::msg::Pose gt_pose_;    // Store the ground truth pose
    bool amcl_received_ = false;          // Flag to check if we received AMCL pose
    bool gt_received_ = false;            // Flag to check if we received ground truth pose
 
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscription_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr gt_subscription_;
 
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Store the AMCL pose
        amcl_pose_ = msg->pose.pose;
        amcl_received_ = true;
 
        // If both poses are received, calculate the error
        if (gt_received_)
        {
            calculateAndLogError();
        }
    }
 
    void gtCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        // Find the index of the robot (assuming the model name is 'turtlebot3')
        auto it = std::find(msg->name.begin(), msg->name.end(), "turtlebot3");
 
        if (it != msg->name.end())
        {
            int idx = std::distance(msg->name.begin(), it);
 
            // Store the ground truth pose
            gt_pose_ = msg->pose[idx];
            gt_received_ = true;
 
            // If both poses are received, calculate the error
            if (amcl_received_)
            {
                calculateAndLogError();
            }
        }
    }
 
    void calculateAndLogError()
    {
        // Calculate the Euclidean distance between the AMCL pose and ground truth pose
        double error = sqrt(pow(amcl_pose_.position.x - gt_pose_.position.x, 2) +
                            pow(amcl_pose_.position.y - gt_pose_.position.y, 2));
 
        // Log the error
        RCLCPP_INFO(this->get_logger(), "Localization Error: %.4f meters", error);
    }
};
 
int main(int argc, char **argv)
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);
 
    // Create an instance of the LocalizationErrorCalculator node
    auto node = std::make_shared<LocalizationErrorCalculator>();
 
    // Spin the node to keep it alive
    rclcpp::spin(node);
 
    // Shutdown the ROS 2 system
    rclcpp::shutdown();
 
    return 0;
}