#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>

using namespace std::chrono_literals;

class System : public rclcpp::Node{
public:
    System() : Node("system_node"){
        laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&System::laserCallback, this, std::placeholders::_1));

        laserpub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/nth_point", 10);

        publish_timer_ = this->create_wall_timer(50ms, std::bind(&System::publishCallback, this));

        this->declare_parameter("nthpoint_", 10);
    }

private:
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg){latest_laserscan_ = msg;}

    void publishCallback(){
        if (latest_laserscan_ != nullptr) {
            auto message = std::make_shared<sensor_msgs::msg::LaserScan>();

            message->header = latest_laserscan_->header;
            message->header.stamp = latest_laserscan_->header.stamp;
            message->angle_min = latest_laserscan_->angle_min;
            message->angle_max = latest_laserscan_->angle_max;
            message->angle_increment = latest_laserscan_->angle_increment*this->get_parameter("nthpoint_").as_int();
            message->time_increment = latest_laserscan_->time_increment*this->get_parameter("nthpoint_").as_int();
            message->scan_time = latest_laserscan_->scan_time;
            message->range_min = latest_laserscan_->range_min;
            message->range_max = latest_laserscan_->range_max; 

            int pointcount = (latest_laserscan_->angle_max - latest_laserscan_->angle_min) / latest_laserscan_->angle_increment;
            message->angle_increment = latest_laserscan_->angle_increment * (this->get_parameter("nthpoint_").as_int());
            for(int i = 0; i <= pointcount; i++){
                if(!(i%this->get_parameter("nthpoint_").as_int())){
                    message->ranges.push_back(latest_laserscan_->ranges.at(i));
                }
            }

            laserpub_->publish(*message);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserpub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::shared_ptr<sensor_msgs::msg::LaserScan> latest_laserscan_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<System>());
    rclcpp::shutdown();
    return 0;
}
