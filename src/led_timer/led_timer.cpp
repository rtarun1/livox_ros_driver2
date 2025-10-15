#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>
using std::placeholders::_1;

class LidarTimestepPub : public rclcpp::Node {
public:
    LidarTimestepPub() : Node("lidar_timestep_publisher") {
        
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&t);
        
        
        char filename[64];
        std::strftime(filename, sizeof(filename), "%Y-%m-%d-%H-%M-%S.bin", &tm);

        logfile_.open(filename, std::ios::binary | std::ios::out);
        if (!logfile_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file!");
        }

        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/led_timestep", 10);

        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar",
            rclcpp::SensorDataQoS(),
            std::bind(&LidarTimestepPub::timer_callback, this, _1));

    }
    ~LidarTimestepPub() {
        if (logfile_.is_open()) logfile_.close();
    }
private:
    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        uint64_t sec = msg->header.stamp.sec;
        uint64_t nanosec = msg->header.stamp.nanosec;

        // Build 18-bit value: [sec LSB10 | nsec LSB8]
        uint32_t sec_lsb10  = static_cast<uint32_t>(sec & 0x3FFu);   
        uint32_t nsec_lsb8  = static_cast<uint32_t>(nanosec & 0xFFu); 
        uint32_t unix_led   = (sec_lsb10 << 8) | nsec_lsb8;         

        std_msgs::msg::Int32 out_msg;
        out_msg.data = static_cast<int32_t>(unix_led);
        publisher_->publish(out_msg);

        RCLCPP_INFO_ONCE(this->get_logger(),
                         "First PointCloud2 received; publishing to /led_timestep");

        if (logfile_.is_open()) {
            logfile_.write(reinterpret_cast<const char*>(&sec), sizeof(sec));
            logfile_.write(reinterpret_cast<const char*>(&nanosec), sizeof(nanosec));
            logfile_.write(reinterpret_cast<const char*>(&unix_led), sizeof(unix_led));
        }
    }
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

        std::ofstream logfile_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarTimestepPub>());
    rclcpp::shutdown();
    return 0;
}