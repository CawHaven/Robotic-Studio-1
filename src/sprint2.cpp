#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <numeric>

class ScanMatchingLocalizer : public rclcpp::Node
{
public:
    ScanMatchingLocalizer() : Node("scan_matching_localizer")
    {
        // Set the initial pose to a known location on the map
        initial_pose_.position.x = 0.0;
        initial_pose_.position.y = 0.0;
        initial_pose_.orientation.z = 0.0;
        initial_pose_.orientation.w = 1.0;

        // Load the static map
        map_image_ = cv::imread("/home/student/ros2_ws/src/package/src/my_map2.pgm", cv::IMREAD_GRAYSCALE);
        if (map_image_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not load map image.");
            rclcpp::shutdown();
        }

        // Subscribers
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanMatchingLocalizer::laser_scan_callback, this, std::placeholders::_1));
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ScanMatchingLocalizer::odometry_callback, this, std::placeholders::_1));

        // Publishers
        estimated_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/estimated_pose", 10);
        rmse_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/rmse_values", 10);

        // Set the initial estimated pose
        estimated_pose_ = initial_pose_;
    }

private:
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimated_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rmse_pub_;

    cv::Mat map_image_;
    geometry_msgs::msg::Pose initial_pose_;
    geometry_msgs::msg::Pose estimated_pose_;
    nav_msgs::msg::Odometry latest_odometry_;

    std::vector<geometry_msgs::msg::Pose> estimated_poses_;
    std::vector<geometry_msgs::msg::Pose> ground_truth_poses_;

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Extract map section around the corrected global estimated pose (Image A)
        cv::Mat map_section = extract_map_section(map_image_, estimated_pose_);
        cv::imshow("Map Section", map_section);
        cv::waitKey(1);

        // Extract edges from Image A (Image B)
        cv::Mat map_edges = detect_edges(map_section);
        cv::imshow("Map Edges", map_edges);
        cv::waitKey(1);

        // Convert laser scan to an image and transform it to the global frame (Image C)
        cv::Mat laser_image = laser_scan_to_global_image(msg, estimated_pose_);
        cv::imshow("Laser Scan Image (Transformed to Global Frame)", laser_image);
        cv::waitKey(1);

        // Estimate rotation using Image B and Image C
        double rotation_diff = estimate_rotation(map_edges, laser_image);

        // Update the estimated pose using scan matching estimate only if rotation is reliable
        if (rotation_diff != 0.0)
        {
            update_global_pose(rotation_diff);
        }

        // Publish the estimated pose
        publish_estimated_pose();

        // Store poses for RMSE calculation
        estimated_poses_.push_back(estimated_pose_);
        ground_truth_poses_.push_back(latest_odometry_.pose.pose);

        // Calculate RMSE if we have enough samples
        if (estimated_poses_.size() >= 10)
        {
            calculate_rmse();
            estimated_poses_.clear();
            ground_truth_poses_.clear();
        }
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odometry_ = *msg;
        // Update pose based on odometry data
        update_pose_from_odometry();
    }

    void update_pose_from_odometry()
    {
        // Update the estimated pose using odometry information
        estimated_pose_.position.x = latest_odometry_.pose.pose.position.x;
        estimated_pose_.position.y = latest_odometry_.pose.pose.position.y;
        estimated_pose_.orientation = latest_odometry_.pose.pose.orientation;
    }

    // Extract map section around the robot
    cv::Mat extract_map_section(const cv::Mat &map, const geometry_msgs::msg::Pose &pose)
    {
        const double resolution = 0.05; // meters per pixel
        const int size = 300;           // size of the ROI in pixels

        // Convert robot position to pixel coordinates
        int x = static_cast<int>((pose.position.x / resolution) + (map.cols / 2));
        int y = static_cast<int>((-pose.position.y / resolution) + (map.rows / 2));

        // Ensure the ROI is within map boundaries
        int roi_x = std::max(0, x - size / 2);
        int roi_y = std::max(0, y - size / 2);
        int roi_width = std::min(size, map.cols - roi_x);
        int roi_height = std::min(size, map.rows - roi_y);

        return map(cv::Rect(roi_x, roi_y, roi_width, roi_height));
    }

    // Edge detection of an image
    cv::Mat detect_edges(const cv::Mat &input_image)
    {
        cv::Mat edges;
        cv::Canny(input_image, edges, 50, 150);
        return edges;
    }

    // Convert laser scan to an OpenCV image and transform to global frame
    cv::Mat laser_scan_to_global_image(const sensor_msgs::msg::LaserScan::SharedPtr &scan, const geometry_msgs::msg::Pose &robot_pose)
{
    int img_size = 300;
    cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);
    float max_range = scan->range_max;

    double theta_robot = 2 * atan2(robot_pose.orientation.z, robot_pose.orientation.w);

    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        float range = scan->ranges[i];
        if (range > scan->range_min && range < max_range)
        {
            float angle = scan->angle_min + i * scan->angle_increment + theta_robot;
            int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
            int y = img_size / 2 - static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)); // Adjusted line
            if (x >= 0 && x < img_size && y >= 0 && y < img_size)
            {
                image.at<uchar>(y, x) = 255;
            }
        }
    }

    return image;
}


    // Estimate rotation between two edge images
    double estimate_rotation(const cv::Mat &img1, const cv::Mat &img2)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::Mat(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::Mat(), keypoints2, descriptors2);

        if (descriptors1.empty() || descriptors2.empty())
        {
            return 0.0;
        }

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(descriptors1, descriptors2, knn_matches, 2);

        // Apply ratio test
        const float ratio_thresh = 0.75f;
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); ++i)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        if (good_matches.empty())
        {
            return 0.0;
        }

        double sum_angle_diff = 0.0;
        int valid_matches = 0;

        for (const auto &match : good_matches)
        {
            cv::Point2f pt1 = keypoints1[match.queryIdx].pt;
            cv::Point2f pt2 = keypoints2[match.trainIdx].pt;

            double angle1 = atan2(pt1.y - img1.rows / 2, pt1.x - img1.cols / 2);
            double angle2 = atan2(pt2.y - img2.rows / 2, pt2.x - img2.cols / 2);

            double angle_diff = angle2 - angle1;
            sum_angle_diff += angle_diff;
            valid_matches++;
        }

        return (valid_matches > 0) ? (sum_angle_diff / valid_matches) * 180.0 / CV_PI : 0.0;
    }

    // Update the estimated pose using global rotation estimate
    void update_global_pose(double angle_diff)
    {
        double dtheta = angle_diff * (CV_PI / 180.0);
        double current_theta = 2 * atan2(estimated_pose_.orientation.z, estimated_pose_.orientation.w);
        current_theta += dtheta;

        // Update orientation
        estimated_pose_.orientation.z = sin(current_theta / 2.0);
        estimated_pose_.orientation.w = cos(current_theta / 2.0);
    }

    // Publish the estimated pose
    void publish_estimated_pose()
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.pose = estimated_pose_;
        estimated_pose_pub_->publish(msg);
    }

    // Calculate RMSE between estimated and ground truth poses
    void calculate_rmse()
    {
        double rmse_x = 0.0, rmse_y = 0.0, rmse_theta = 0.0;

        for (size_t i = 0; i < estimated_poses_.size(); ++i)
        {
            double error_x = estimated_poses_[i].position.x - ground_truth_poses_[i].position.x;
            double error_y = estimated_poses_[i].position.y - ground_truth_poses_[i].position.y;
            double error_theta = estimated_poses_[i].orientation.z - ground_truth_poses_[i].orientation.z;

            rmse_x += error_x * error_x;
            rmse_y += error_y * error_y;
            rmse_theta += error_theta * error_theta;
        }

        rmse_x = sqrt(rmse_x / estimated_poses_.size());
        rmse_y = sqrt(rmse_y / estimated_poses_.size());
        rmse_theta = sqrt(rmse_theta / estimated_poses_.size());

        // Print only X, Y, and RMSE in a single line
        RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f, RMSE: %f", estimated_pose_.position.x, estimated_pose_.position.y, rmse_theta);

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatchingLocalizer>());
    rclcpp::shutdown();
    return 0;
}
