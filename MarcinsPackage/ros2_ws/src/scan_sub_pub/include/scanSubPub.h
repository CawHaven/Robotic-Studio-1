#ifndef SCANSUBPUB_H
#define SCANSUBPUB_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono_literals;

class System : public rclcpp::Node {
public:
    System();

private:
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);
    void publishCallback();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::shared_ptr<sensor_msgs::msg::LaserScan> latest_scan_;
};

#endif // SCANSUBPUB_H
