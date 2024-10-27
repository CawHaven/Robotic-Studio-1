#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>  // Added for waypoint (clicked point)
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/float64.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <vector>
#include <optional>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;


class System: public rclcpp::Node{
    public:
    System() : Node("System"){
        laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&System::laser_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&System::odomCallback, this, std::placeholders::_1));
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&System::mapCallback, this, std::placeholders::_1));

        publish_timer_ = this->create_wall_timer(10ms, std::bind(&System::system, this));

        this->declare_parameter<double>("Scale", -60);
        this->declare_parameter<double>("OffsetX", 215);
        this->declare_parameter<double>("OffsetY", -225);
        this->declare_parameter<int>("Image", 410);
        this->declare_parameter<int>("Print", 0);
        this->declare_parameter<double>("mapscale", 3);
        this->declare_parameter<int>("xoff", -25);
        this->declare_parameter<int>("yoff", -25);
    }

    void system(){
        if(!((latest_laserscan_ == nullptr)||(latest_odom_ == nullptr)||(latest_map_ == nullptr))){
            auto ground_truth_map_ = cv::imread("/home/ovo/robotics-studio-1/src/ground_truth_map.pgm", cv::IMREAD_COLOR);
            if(ground_truth_map_.empty()){
                RCLCPP_ERROR(this->get_logger(), "Could not load the ground truth map.");
                rclcpp::shutdown();
                return;
            }

            int print;
            this->get_parameter("Print", print);
            if(print){
                double a, b, c;
                int d;
                this->get_parameter("Scale", a);
                this->get_parameter("OffsetX", b);
                this->get_parameter("OffsetY", c);
                this->get_parameter("Image", d);
                std::cout << "Scale: " << a << "  OffsetX: " << b << "  OffsetY:" << c << "  Image: "  << d << std::endl;
                this->set_parameter(rclcpp::Parameter("Print", 0));
            }
    
            // Generating Map
            auto image = GenerateBaseImage(ground_truth_map_, 3);
    
            double roll, pitch, yaw;
            QuaternionToRPY(latest_odom_->pose.pose.orientation, roll, pitch, yaw);

            cv::Mat LaserScan = laserScanToMat(latest_laserscan_, convertToZeroToTwoPi(yaw) + 0);
            cv::flip(LaserScan, LaserScan, 0);

            cv::Mat Buffed = image.clone();
            Buffed.setTo(cv::Scalar(0,0,0));

            double scale;
            this->get_parameter("Scale", scale);
            double offsetX = 225;
            this->get_parameter("OffsetX", offsetX);
            double offsetY = -215;
            this->get_parameter("OffsetY", offsetY);
            double x = (image.rows/2) - (LaserScan.rows/2) + offsetX + latest_odom_->pose.pose.position.x * -scale;
            double y = (image.cols/2) - (LaserScan.cols/2) + offsetY + latest_odom_->pose.pose.position.y * scale;
            cv::Mat Circs = cv::Mat::zeros(100, 100, CV_8UC3);
            cv::Point center(50, 50);
            int radius = 5;
            cv::Scalar color(0, 0, 255); // Red color
            int thickness = -1; // Filled circle
            cv::circle(Circs, center, radius, color, thickness);
            cv::Mat Circle, Circled;
            blendImages(LaserScan, pasteOverlay(convertChannels(LaserScan, Circs), Circs, (LaserScan.cols/2) - 50, (LaserScan.rows/2) - 50), Circled);
            
            // Rubble Detection
            auto imagecopy = image.clone();
            auto blur = applyGaussianBlur(imagecopy, 29, 9);
            // cv::imshow("temp", blur);
                
            cv::Point seed_point(blur.cols/2, blur.rows/2);
            cv::Scalar colour(100, 100, 100);
            cv::floodFill(blur, seed_point, colour);
            // cv::imshow("temp2", blur);
    
            cv::Mat mask;
            cv::inRange(blur, colour, colour, mask);
            cv::Mat output;
            cv::bitwise_and(blur, blur, output, mask);
            cv::Mat edges;
    
            cv::Canny(output, edges, 20, 10);
            cv::Scalar white(255, 255, 255);
            cv::floodFill(output, seed_point, white);
            cv::bitwise_not(output.clone(), output);
            // cv::imshow("temp3", output);
                
            // auto tempval = latest_odom_->twist.twist.angular;
            // std::cout << "X: " << tempval.x << "  Y: " << tempval.y << "  Z: " << tempval.z << std::endl;
            // int ThreshH, ThreshL;
            // this->get_parameter("ThreshH", ThreshH);
            // this->get_parameter("ThreshL", ThreshL);
            if(initialiseMap){rubbledetected = cv::Mat::zeros(image.size(), CV_8UC3); initialiseMap = 0;}
            if(latest_odom_->twist.twist.angular.z < 0.2){
                cv::Mat whitemask;
                cv::inRange(output, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), whitemask);
                output.setTo(cv::Scalar(100, 100, 100), whitemask);

                cv::Mat mapimage = occupancyGridToMat(latest_map_);
                double tempval;
                this->get_parameter("mapscale", tempval);
                cv::Size newSize(static_cast<int>(mapimage.cols * tempval), static_cast<int>(mapimage.rows * tempval));
                cv::Mat resizedImage;
                cv::resize(mapimage, resizedImage, newSize);
                cv::flip(resizedImage.clone(), resizedImage, 0);

                int xoff, yoff;
                this->get_parameter("xoff", xoff);
                this->get_parameter("yoff", yoff);
                cv::Mat positioned = pasteOverlay(output, resizedImage, xoff, yoff);
                blendImages(positioned.clone(), output, positioned);
                // cv::imshow("Slam", positioned);
                
                cv::inRange(positioned, cv::Scalar(100,100,100), cv::Scalar(100,100,100), whitemask);
                positioned.setTo(cv::Scalar(255,255,255), whitemask);
                cv::floodFill(positioned, cv::Point(1,1), cv::Scalar(255,255,128));
                cv::inRange(positioned, cv::Scalar(255,255,128), cv::Scalar(255,255,128), whitemask);
                cv::Mat rubblebase = cv::Mat::zeros(image.size(), CV_8UC3);
                cv::floodFill(rubblebase, cv::Point(1,1), cv::Scalar(0, 128, 255));
                rubblebase.setTo(cv::Scalar(0, 0, 0), whitemask);
                // cv::imshow("test", rubblebase);

                rubbledetected = rubblebase;

            }

            blendImages(image.clone(), rubbledetected, image);

            cv::Mat Output;
            blendImages(image, pasteOverlay(Buffed, Circled, x, y), Output);

            // cv::imshow("Base", ground_truth_map_);
            cv::imshow("Image", Output);
            cv::waitKey(1);
        }else{
            if(latest_laserscan_ == nullptr){RCLCPP_ERROR(this->get_logger(), "Laserscan Not Available");}
            if(latest_odom_ == nullptr){RCLCPP_ERROR(this->get_logger(), "Odometry Not Available");}
            if(latest_map_ == nullptr){RCLCPP_ERROR(this->get_logger(), "Map Not Available");}
        }
    }

    void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg){latest_odom_ = msg;}

    void laser_callback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg){latest_laserscan_ = msg;}

    void mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg){latest_map_ = msg;}

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan, float rotation) {
        // Parameters
        int img_size; // = 430; // Was 450
        this->get_parameter("Image", img_size);
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = (scan->angle_min + i * scan->angle_increment) + rotation;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }

        return image;
    }

    double convertToZeroToTwoPi(double angle) {
        // Normalize the angle to the range [0, 2π]
        if (angle < 0) {
            angle += 2 * M_PI;  // Add 2π if the angle is negative
        }
        return angle;
    }

    void QuaternionToRPY(geometry_msgs::msg::Quaternion q, double& roll, double& pitch, double& yaw) {
        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll = std::atan2(sinr_cosp, cosr_cosp);
    
        // Pitch (y-axis rotation)
        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        if (std::fabs(sinp) >= 1.0)
            pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
        else
            pitch = std::asin(sinp);
    
        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    // Generate an image from a given image with rainbow edges
    cv::Mat GenerateBaseImage(cv::Mat image, double scale){
        cv::Size newSize(static_cast<int>(image.cols * scale), static_cast<int>(image.rows * scale));
        cv::Mat resizedImage;
        cv::resize(image, resizedImage, newSize);

        cv::Mat processing = resizedImage.clone();
        cv::Point seed_point(processing.cols/2, processing.rows/2);
        cv::Scalar colour(255, 255, 128); // Blue, Green, Red
        cv::floodFill(processing, seed_point, colour);

        cv::Mat mask;
        cv::inRange(processing, colour, colour, mask);
        cv::Mat output;
        cv::bitwise_and(processing, processing, output, mask);
        cv::Mat edges;
        cv::Canny(output, edges, 10, 20);
        cv::Scalar white(80, 80, 80);
        cv::floodFill(output, seed_point, white);

        cv::Mat base_image = output.clone();
        cv::Mat bgr_image;
        cv::cvtColor(edges, bgr_image, cv::COLOR_GRAY2BGR);
        cv::Mat hsv_image;
        cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_image, cv::Scalar(0, 0, 200), cv::Scalar(180, 30, 255), mask);
        hsv_image.setTo(cv::Scalar(target_hue, 255, 255), mask);
        target_hue += 10;
        if(target_hue > 180){target_hue = 0;}
        cv::cvtColor(hsv_image, base_image, cv::COLOR_HSV2BGR);

        blendImages(output, base_image, base_image);

        return base_image;
    }

    // Image Processing: Blend 2 images over each other (Blend mode: Screen) // Note that this is Exact Image Sizes
    void blendImages(const cv::Mat& baseImage, const cv::Mat& overlayImage, cv::Mat& outputImage){
        cv::Mat overlayImageResized;
        cv::resize(overlayImage, overlayImageResized, baseImage.size());
        cv::Mat baseImageConverted, overlayImageConverted;

        if(baseImage.channels() == 3){baseImageConverted = baseImage;}
            else if(baseImage.channels() == 1){cv::cvtColor(baseImage, baseImageConverted, cv::COLOR_GRAY2RGB);}
            else{cv::cvtColor(baseImage, baseImageConverted, cv::COLOR_RGBA2RGB);
        }

        if(overlayImageResized.channels() == 3){overlayImageConverted = overlayImageResized;}
            else if(overlayImageResized.channels() == 1){cv::cvtColor(overlayImageResized, overlayImageConverted, cv::COLOR_GRAY2RGB);}
            else{cv::cvtColor(overlayImageResized, overlayImageConverted, cv::COLOR_RGBA2RGB);
        }

        cv::Mat mask;
        cv::inRange(overlayImageConverted, cv::Scalar(0, 0, 0), cv::Scalar(10, 10, 10), mask);

        cv::Mat invMask;
        cv::bitwise_not(mask, invMask);

        outputImage = baseImageConverted.clone();

        cv::Mat foreground, background;
        baseImageConverted.copyTo(background, mask);
        overlayImageConverted.copyTo(foreground, invMask);

        cv::add(background, foreground, outputImage);
    }

    cv::Mat pasteOverlay(const cv::Mat& baseImage, const cv::Mat& overlay, int xOffset, int yOffset) {
        // Create a copy of the base image to work on
        cv::Mat result = baseImage.clone();

        // Check if the overlay needs to be converted to match base image channels
        cv::Mat overlayAdjusted;
        if (overlay.channels() != baseImage.channels()) {
            // Convert overlay to the same number of channels as baseImage
            cv::cvtColor(overlay, overlayAdjusted, (overlay.channels() == 1) ? cv::COLOR_GRAY2BGR : cv::COLOR_BGR2GRAY);
        } else {
            overlayAdjusted = overlay;
        }

        // Calculate the region of interest (ROI) in the base image
        int xStart = std::max(0, xOffset);
        int yStart = std::max(0, yOffset);
        int xEnd = std::min(baseImage.cols, xOffset + overlayAdjusted.cols);
        int yEnd = std::min(baseImage.rows, yOffset + overlayAdjusted.rows);

        // Check if the overlay is within the bounds of the base image
        if (xStart < xEnd && yStart < yEnd) {
            // Define the ROI in the base image
            cv::Rect roi(xStart, yStart, xEnd - xStart, yEnd - yStart);

            // Define the corresponding ROI in the overlay
            cv::Rect overlayRoi(xStart - xOffset, yStart - yOffset, xEnd - xStart, yEnd - yStart);

            // Paste the overlay onto the base image within the defined ROI
            overlayAdjusted(overlayRoi).copyTo(result(roi));
        }

        return result;
    }

    cv::Mat convertChannels(const cv::Mat& source, const cv::Mat& target) {
        cv::Mat converted;

        // Check if the number of channels is different
        if (source.channels() != target.channels()) {
            if (target.channels() == 3 && source.channels() == 1) {
                // Convert grayscale to BGR
                cv::cvtColor(source, converted, cv::COLOR_GRAY2BGR);
            } else if (target.channels() == 1 && source.channels() == 3) {
                // Convert BGR to grayscale
                cv::cvtColor(source, converted, cv::COLOR_BGR2GRAY);
            } else if (target.channels() == 4 && source.channels() == 3) {
                // Convert BGR to BGRA
                cv::cvtColor(source, converted, cv::COLOR_BGR2BGRA);
            } else if (target.channels() == 3 && source.channels() == 4) {
                // Convert BGRA to BGR
                cv::cvtColor(source, converted, cv::COLOR_BGRA2BGR);
            } else {
                // Other conversions can be added as needed
                throw std::invalid_argument("Unsupported channel conversion.");
            }
        } else {
            // No conversion needed, return the original
            converted = source.clone();
        }

        return converted;
    }

    cv::Mat applyGaussianBlur(const cv::Mat& inputImage, int kernelSize, double sigmaX) {
        // Check if the input image is empty
        if (inputImage.empty()) {
            std::cerr << "Input image is empty!" << std::endl;
            return cv::Mat();
        }

        // Ensure kernel size is odd and greater than 1
        if (kernelSize % 2 == 0 || kernelSize <= 1) {
            std::cerr << "Kernel size must be an odd number greater than 1!" << std::endl;
            return cv::Mat();
        }

        // Output image
        cv::Mat outputImage;

        // Apply Gaussian blur
        cv::GaussianBlur(inputImage, outputImage, cv::Size(kernelSize, kernelSize), sigmaX);

        return outputImage;
    }

    // cv::Mat findMiddleLine(const cv::Mat& inputImage) {
    //     // Check if the image is empty
    //     if (inputImage.empty()) {
    //         std::cerr << "Input image is empty!" << std::endl;
    //         return cv::Mat();
    //     }

    //     // Convert to grayscale
    //     cv::Mat gray;
    //     cv::cvtColor(inputImage, gray, cv::COLOR_BGR2GRAY);

    //     // Apply Gaussian blur
    //     cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    //     // Edge detection
    //     cv::Mat edges;
    //     cv::Canny(gray, edges, 50, 150);

    //     // Hough Transform to detect lines
    //     std::vector<cv::Vec2f> lines;
    //     cv::HoughLines(edges, lines, 1, CV_PI / 180, 100);

    //     // Convert the lines to a Mat for easier manipulation
    //     cv::Mat linesMat(lines.size(), 2, CV_32F, lines.data());

    //     // Calculate average rho and theta
    //     cv::Scalar avgRhoTheta = cv::mean(linesMat);

    //     // Extract average rho and theta
    //     float avgRho = avgRhoTheta[0];
    //     float avgTheta = avgRhoTheta[1];

    //     // Calculate the end points of the average line
    //     double a = cos(avgTheta);
    //     double b = sin(avgTheta);
    //     double x0 = a * avgRho;
    //     double y0 = b * avgRho;

    //     cv::Point pt1(std::round(x0 + 1000 * (-b)), std::round(y0 + 1000 * (a)));
    //     cv::Point pt2(std::round(x0 - 1000 * (-b)), std::round(y0 - 1000 * (a)));

    //     // Create a copy of the input image to draw the middle line
    //     cv::Mat outputImage = inputImage.clone();
    //     cv::line(outputImage, pt1, pt2, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

    //     return outputImage;
    // }

    cv::Mat occupancyGridToMat(const std::shared_ptr<nav_msgs::msg::OccupancyGrid>& latest_map) {
        // Check if the occupancy grid pointer is valid
        if (!latest_map) {
            std::cerr << "Occupancy grid is null!" << std::endl;
            return cv::Mat();
        }

        // Get the dimensions of the occupancy grid
        int width = latest_map->info.width;
        int height = latest_map->info.height;

        // Create a cv::Mat of type CV_8U to hold the occupancy values
        cv::Mat mat(height, width, CV_8U);

        // Populate the cv::Mat with values from the occupancy grid
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                int index = i * width + j;
                // Map occupancy values to 0-255 range
                if (latest_map->data[index] == -1) {
                    mat.at<uchar>(i, j) = 127; // Unknown
                } else if (latest_map->data[index] == 0) {
                    mat.at<uchar>(i, j) = 255; // Free
                } else {
                    mat.at<uchar>(i, j) = 0;   // Occupied
                }
            }
        }

        return mat;
    }

    private:
    // Variables
    rclcpp::TimerBase::SharedPtr publish_timer_;
    // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr slam_map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
    std::shared_ptr<sensor_msgs::msg::LaserScan> latest_laserscan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<nav_msgs::msg::Odometry> latest_odom_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> latest_map_;

    int target_hue = 0;
    bool initialiseMap = 1;
    cv::Mat rubbledetected;



};

int main(int argc, char**argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<System>());
    rclcpp::shutdown();
    return 0;
}

