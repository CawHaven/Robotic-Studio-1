/**     Localisation System Idea
 * Write a subscriber node to receive laser scans
 * Convert incoming laser scans to images and display them using cv::imshow
 * Keep 2 instances of laser scans and convert them to images ??????
 * Estimate the relative rotation angle between 2 laser scans ??????
 */

/**     SLO 3.5
 * Use ROS2 nav2_amcl package to localise the robot
 */

/**     SLO 3.6
 * Set the initial robot pose to a known location of the map
 * Extract section of the map around the robot (let's call this Image A)
 * Extract edges from Image A using opencv and create and edge image (let's call this Image B)
 * Receive a laser scan and convert it to an image (let's call this Image C)
 * Similar to WK5 lab activity use Image B and Image C to estimate the rotation of robot relative to the map
 * Using odometry, propagate the robot to the new location
 */

/**     SLO 3.7
 * Compare the localisation accuracies against the ground truth
 */




            // Map_Processor_node

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
    : Node("map_processor_node")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapProcessorNode::scanCallback, this, std::placeholders::_1));

        cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::cout << "scanCallback" << std::endl;
        // cv::Mat tmp_col_img = m_MapColImage.clone();
        // cv::rotate(tmp_col_img, tmp_col_img, cv::ROTATE_90_COUNTERCLOCKWISE);
        // cv::imshow(WINDOW1, tmp_col_img);
        cv::waitKey(1);    
    }
    

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::cout << "mapCallback" << std::endl;

        occupancyGridToImage(mapMsg);

        cv::Mat tmp_col_img = m_MapColImage.clone();

        cv::rotate(tmp_col_img, tmp_col_img, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imshow(WINDOW1, tmp_col_img);
        cv::waitKey(1);
    }

    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
{
    int grid_data;
    unsigned int row, col, val;

    m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

    std::cout << "DataParse started for map: " << grid->header.stamp.sec << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;

    for (row = 0; row < grid->info.height; row++) {
        for (col = 0; col < grid->info.width; col++) {
            grid_data = grid->data[row * grid->info.width + col];
            if (grid_data != -1) {
                val = 255 - (255 * grid_data) / 100;
                val = (val == 0) ? 255 : 0;
                m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
            } else {
                m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
            }
        }
    }

    map_scale_ = grid->info.resolution;
    origin_x = grid->info.origin.position.x;
    origin_y = grid->info.origin.position.y;
    size_x = grid->info.width;
    size_y = grid->info.height;

    cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                0, 1, 0,
                                0, 0, 0);
    cv::erode(m_temp_img, m_MapBinImage, kernel);

    m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
    cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);

    std::cout << "Occupancy grid map converted to a binary image\n";

    // // Display the image to verify
    // cv::imshow("Occupancy Grid", m_MapColImage);
    // cv::waitKey(1);
}


    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;


    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;
    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int size_x;
    unsigned int size_y;

    const std::string WINDOW1 = "Map Image";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();
    return 0;
}





                // LAB 5



#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ScanToImageNode : public rclcpp::Node {
public:
    ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientaion_(0.0) {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
        cv::Mat img = laserScanToMat(msg);

        if (!first_image_captured_) {
            first_image_ = img.clone();
            first_image_captured_ = true;
            // Display the first image
            cv::imshow("First Image", first_image_);
            cv::waitKey(1);  // Add this to process GUI events and update the window
            // Rotate the robot by publishing to cmd_vel
            // rotateRobot();
        } else if (!second_image_captured_) {
            second_image_ = img.clone();
            second_image_captured_ = true;
            // Display the second image
            cv::imshow("Second Image", second_image_);
            cv::waitKey(1);  // Add this to process GUI events and update the window
            // Calculate the change in yaw using cv::transform
        } else {
            first_image_ = second_image_.clone();
            second_image_ = img.clone();
            // Display the new second image
            cv::imshow("Second Image", second_image_);
            cv::waitKey(1);  // Add this to process GUI events and update the window

            calculateYawChange();
            relative_orientaion_ = relative_orientaion_ + angle_difference_;
            RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f", relative_orientaion_);
        }
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        // Parameters
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        return image;
    }




    // void rotateRobot() {
    //     auto twist_msg = geometry_msgs::msg::Twist();
    //     twist_msg.angular.z = 1.0;  // Rotate with some angular velocity
    //     cmd_publisher_->publish(twist_msg);

    //     // Sleep for a while to allow the robot to rotate
    //     rclcpp::sleep_for(std::chrono::seconds(2));

    //     // Stop rotation
    //     twist_msg.angular.z = 0.0;
    //     cmd_publisher_->publish(twist_msg);
    // }

    void calculateYawChange() {
        // Detect and match features between the first and second images
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            } else {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }

    }

                                    // SWIFT AND SURF
    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : matches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat first_image_, second_image_;
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;

    double angle_difference_;
    double relative_orientaion_ = 0.0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToImageNode>());
    rclcpp::shutdown();
    return 0;
}
