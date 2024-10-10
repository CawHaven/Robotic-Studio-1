#include <scanSubPub.h>

System::System() : Node("system_node") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(50).keep_last(50), 
        std::bind(&System::laserCallback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/nth_point", 10);

    publish_timer_ = this->create_wall_timer(200ms, std::bind(&System::publishCallback, this));

    this->declare_parameter("nthpoint_", 7);
}

void System::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg) {
    latest_scan_ = msg;
}

void System::publishCallback() {
    if (latest_scan_ != nullptr) {
    
        auto message = std::make_shared<sensor_msgs::msg::LaserScan>();
        int nthPoint = this->get_parameter("nthpoint_").as_int();

        	message->header = latest_scan_->header;
        	message->header.stamp = latest_scan_->header.stamp;
        	message->angle_min = latest_scan_->angle_min;
        	message->angle_max = latest_scan_->angle_max;
        	message->angle_increment = latest_scan_->angle_increment*nthPoint;
        	message->time_increment = latest_scan_->time_increment*nthPoint;
        	message->scan_time = latest_scan_->scan_time;
        	message->range_min = latest_scan_->range_min;
        	message->range_max = latest_scan_->range_max;

        int pointcount = (latest_scan_->angle_max - latest_scan_->angle_min) / latest_scan_->angle_increment;
        

        for (int i = 0; i <= pointcount; i++) {
            if (i < latest_scan_->ranges.size() && !(i % nthPoint)) {
            	
        	
                message->ranges.push_back(latest_scan_->ranges.at(i));
            }
        }

        scan_pub_->publish(*message);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<System>());
    rclcpp::shutdown();
    return 0;
}
