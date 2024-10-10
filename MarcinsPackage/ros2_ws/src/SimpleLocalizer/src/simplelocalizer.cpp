#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <numeric>

#define TURTLEBOT3_MODEL "waffle_pi"

class SimpleLocalizer : public rclcpp::Node
{
public:
    SimpleLocalizer() : Node("simple_localizer")
    {
        // Set the initial pose
        initial_pose_.position.x = -2.049114244167206;
        initial_pose_.position.y = -0.5000196437467848;
        initial_pose_.position.z = 0.007880084478990227;

        initial_pose_.orientation.x = -1.8213488936396088e-06;
        initial_pose_.orientation.y = 0.003144299173039618;
        initial_pose_.orientation.z = 0.0008683593126252406;
        initial_pose_.orientation.w = 0.9999946796515955;

        robot_pose = initial_pose_;

        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SimpleLocalizer::laser_scan_callback, this, std::placeholders::_1));

        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SimpleLocalizer::odometry_callback, this, std::placeholders::_1));

        amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&SimpleLocalizer::amcl_pose_callback, this, std::placeholders::_1));

        // Publisher for estimated poses
        estimated_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/estimated_pose", 10);

        // Publisher for ground truth poses
        ground_truth_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ground_truth_pose", 10);

        // Publisher for AMCL poses
        amcl_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose_estimate", 10);

        // Publisher for RMSE values
        rmse_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/rmse_values", 10);

        // Publisher for AMCL RMSE values
        amcl_rmse_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/amcl_rmse_values", 10);

        // Load the static map
        map_image_ = cv::imread("/home/martin/my_map.pgm", cv::IMREAD_GRAYSCALE);
        if (map_image_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not load map image.");
            rclcpp::shutdown();
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimated_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ground_truth_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rmse_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr amcl_rmse_pub_;

    sensor_msgs::msg::LaserScan::SharedPtr previous_scan_ = nullptr;
    cv::Mat map_image_;
    geometry_msgs::msg::Pose initial_pose_;
    nav_msgs::msg::Odometry odometry_;
    geometry_msgs::msg::Pose ground_truth_pose_;
    geometry_msgs::msg::Pose robot_pose;

    std::vector<geometry_msgs::msg::Pose> estimated_poses_;
    std::vector<geometry_msgs::msg::Pose> ground_truth_poses_;
    std::vector<geometry_msgs::msg::Pose> amcl_poses_;

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (previous_scan_)
        {
            cv::Mat img1 = laser_scan_to_image(previous_scan_);
            cv::Mat img2 = laser_scan_to_image(msg);
            double rotation_estimate = estimate_rotation(img1, img2);

            localizer(msg, rotation_estimate);
        }
        previous_scan_ = msg;
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odometry_ = *msg;
        ground_truth_pose_ = msg->pose.pose;

        ground_truth_poses_.push_back(ground_truth_pose_);

        publish_ground_truth_pose();
    }

    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        amcl_poses_.push_back(msg->pose.pose);
        publish_amcl_pose(msg);
    }

    void publish_ground_truth_pose()
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.pose = ground_truth_pose_;
        ground_truth_pose_pub_->publish(msg);
    }

    void publish_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg)
    {
        amcl_pose_pub_->publish(*msg);
    }

    void publish_estimated_pose()
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.pose = initial_pose_;
        estimated_pose_pub_->publish(msg);
    }

    void publish_rmse(double rmse_x, double rmse_y, double rmse_theta)
    {
        if (!std::isnan(rmse_x) && !std::isnan(rmse_y) && !std::isnan(rmse_theta))
        {
            std_msgs::msg::Float64MultiArray rmse_msg;
            rmse_msg.data = {rmse_x, rmse_y, rmse_theta};
            rmse_pub_->publish(rmse_msg);
        }
    }

    void publish_amcl_rmse(double amcl_rmse_x, double amcl_rmse_y, double amcl_rmse_theta)
    {
        if (!std::isnan(amcl_rmse_x) && !std::isnan(amcl_rmse_y) && !std::isnan(amcl_rmse_theta))
        {
            std_msgs::msg::Float64MultiArray amcl_rmse_msg;
            amcl_rmse_msg.data = {amcl_rmse_x, amcl_rmse_y, amcl_rmse_theta};
            amcl_rmse_pub_->publish(amcl_rmse_msg);
        }
    }

    cv::Mat laser_scan_to_image(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {
        int width = 500, height = 500;
        cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float range = scan->ranges[i];
            if (std::isfinite(range))
            {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * 100 + width / 2);
                int y = static_cast<int>((range * sin(angle)) * 100 + height / 2);

                if (x >= 0 && x < width && y >= 0 && y < height)
                    cv::circle(image, cv::Point(x, y), 2, cv::Scalar(255, 0, 0), -1);
            }
        }

        cv::imshow("Laser Scan", image);
        cv::waitKey(1);

        return image;
    }

    double estimate_rotation(const cv::Mat &img1, const cv::Mat &img2)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::Mat(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::Mat(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        double sum_angle_diff = 0.0;
        int valid_matches = 0;

        for (const auto &match : matches)
        {
            cv::Point2f pt1 = keypoints1[match.queryIdx].pt;
            cv::Point2f pt2 = keypoints2[match.trainIdx].pt;

            double angle1 = atan2(pt1.y - img1.rows / 2, pt1.x - img1.cols / 2);
            double angle2 = atan2(pt2.y - img2.rows / 2, pt2.x - img2.cols / 2);

            double angle_diff = angle2 - angle1;
            sum_angle_diff += angle_diff;
            valid_matches++;
        }
        RCLCPP_INFO(this->get_logger(), "Angle Difference Between images -> %f", (valid_matches > 0) ? sum_angle_diff / valid_matches : 0.0);

        return (valid_matches > 0) ? sum_angle_diff / valid_matches : 0.0;
    }

    void localizer(const sensor_msgs::msg::LaserScan::SharedPtr &current_scan, double rotation_estimate)
    {
        cv::Mat image_A = extract_map_section(map_image_, initial_pose_);

        cv::Mat image_B;
        cv::Canny(image_A, image_B, 50, 150);

        cv::namedWindow("Edge Match", cv::WINDOW_NORMAL);
        cv::resizeWindow("Edge Match", 500, 500); // Set the window size to 500x500
        cv::imshow("Edge Match", image_B);

        cv::Mat image_C = laser_scan_to_image(current_scan);

        double match_score = estimate_rotation(image_B, image_C);

        propagate_robot_pose(initial_pose_, odometry_, match_score);

        estimated_poses_.push_back(initial_pose_);

        publish_estimated_pose();

        if (estimated_poses_.size() > 10)
        {
            calculate_rmse();
        }
    }

    cv::Mat extract_map_section(const cv::Mat &map, const geometry_msgs::msg::Pose &pose)
    {
        // Map properties
        const double resolution = 0.05; // meters per pixel
        const int width = 374;          // pixels
        const int height = 223;         // pixels
        const double map_origin_x = 0;  // map origin in meters
        const double map_origin_y = 0;  // map origin in meters

        // Calculate the pixel coordinates from the robot's position
        int x = static_cast<int>(((pose.position.x - map_origin_x) / resolution) + (width / 2));
        int y = static_cast<int>(((map_origin_y - pose.position.y) / resolution) + (height / 2)); // Invert y

        // Calculate the size of the ROI based on the robot's size and map properties
        int size = 100; // size of the ROI in pixels
        cv::Rect roi(x - size / 2, y - size / 2, size, size);

        // Ensure the ROI stays within map bounds
        roi.x = std::max(0, roi.x);
        roi.y = std::max(0, roi.y);
        roi.width = std::min(size, map.cols - roi.x);
        roi.height = std::min(size, map.rows - roi.y);

        // Log the current pose and ROI for debugging
        RCLCPP_INFO(this->get_logger(), "Pose: (%f, %f), ROI: [%d, %d, %d, %d], Map Size: [%d, %d]",
                    pose.position.x, pose.position.y, roi.x, roi.y, roi.width, roi.height, map.cols, map.rows);

        // Extract the section of the map
        cv::Mat map_section = map(roi);

        // Rotate the extracted map section based on the robot's orientation
        double yaw = atan2(2.0 * (robot_pose.orientation.z * robot_pose.orientation.w + robot_pose.orientation.x * robot_pose.orientation.y),
                           1.0 - 2.0 * (robot_pose.orientation.y * robot_pose.orientation.y + robot_pose.orientation.z * robot_pose.orientation.z));

        // Create a transformation matrix for rotation
        cv::Mat rotation_matrix = cv::getRotationMatrix2D(cv::Point2f(size / 2, size / 2), yaw * (180.0 / CV_PI), 1.0);
        cv::Mat rotated_section;

        // Create a white background for the output image
        cv::Mat white_background(map_section.size(), map_section.type(), cv::Scalar(255, 255, 255));

        // Apply the warpAffine with the white background
        cv::warpAffine(map_section, rotated_section, rotation_matrix, map_section.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
        cv::Mat axis_corrected_section;
        cv::flip(rotated_section, axis_corrected_section, 1);

        cv::namedWindow("Map Section", cv::WINDOW_NORMAL);
        cv::resizeWindow("Map Section", 500, 500); // Set the window size to 500x500
        cv::imshow("Map Section", axis_corrected_section);
        cv::waitKey(1);

        return axis_corrected_section;
    }

    void propagate_robot_pose(geometry_msgs::msg::Pose &pose, const nav_msgs::msg::Odometry &odom, double match_score)
    {

        robot_pose.position.x += odom.twist.twist.linear.x;
        robot_pose.position.y += odom.twist.twist.linear.y;
        robot_pose.orientation.z += odom.twist.twist.angular.z;

        pose.position = ground_truth_pose_.position;
    }

    void calculate_rmse()
    {
        double rmse_x = 0.0, rmse_y = 0.0, rmse_theta = 0.0;
        double amcl_rmse_x = 0.0, amcl_rmse_y = 0.0, amcl_rmse_theta = 0.0;

        for (size_t i = 0; i < estimated_poses_.size(); ++i)
        {
            double error_x = estimated_poses_[i].position.x - ground_truth_poses_[i].position.x;
            double error_y = estimated_poses_[i].position.y - ground_truth_poses_[i].position.y;
            double error_theta = estimated_poses_[i].orientation.z - ground_truth_poses_[i].orientation.z;

            rmse_x += error_x * error_x;
            rmse_y += error_y * error_y;
            rmse_theta += error_theta * error_theta;

            if (i < amcl_poses_.size())
            {
                double amcl_error_x = amcl_poses_[i].position.x - ground_truth_poses_[i].position.x;
                double amcl_error_y = amcl_poses_[i].position.y - ground_truth_poses_[i].position.y;
                double amcl_error_theta = amcl_poses_[i].orientation.z - ground_truth_poses_[i].orientation.z;

                amcl_rmse_x += amcl_error_x * amcl_error_x;
                amcl_rmse_y += amcl_error_y * amcl_error_y;
                amcl_rmse_theta += amcl_error_theta * amcl_error_theta;
            }
        }

        rmse_x = sqrt(rmse_x / estimated_poses_.size());
        rmse_y = sqrt(rmse_y / estimated_poses_.size());
        rmse_theta = sqrt(rmse_theta / estimated_poses_.size());

        if (!amcl_poses_.empty())
        {
            amcl_rmse_x = sqrt(amcl_rmse_x / amcl_poses_.size());
            amcl_rmse_y = sqrt(amcl_rmse_y / amcl_poses_.size());
            amcl_rmse_theta = sqrt(amcl_rmse_theta / amcl_poses_.size());
        }

        if (!std::isnan(rmse_x) && !std::isnan(rmse_y) && !std::isnan(rmse_theta))
        {
            RCLCPP_INFO(this->get_logger(), "Scan Matching RMSE -> X: %f, Y: %f, Theta: %f", rmse_x, rmse_y, rmse_theta);
            publish_rmse(rmse_x, rmse_y, rmse_theta);
        }

        if (!amcl_poses_.empty() && !std::isnan(amcl_rmse_x) && !std::isnan(amcl_rmse_y) && !std::isnan(amcl_rmse_theta))
        {
            RCLCPP_INFO(this->get_logger(), "AMCL RMSE -> X: %f, Y: %f, Theta: %f", amcl_rmse_x, amcl_rmse_y, amcl_rmse_theta);
            publish_amcl_rmse(amcl_rmse_x, amcl_rmse_y, amcl_rmse_theta);
        }

        estimated_poses_.clear();
        ground_truth_poses_.clear();
        amcl_poses_.clear();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleLocalizer>());
    rclcpp::shutdown();
    return 0;
}
