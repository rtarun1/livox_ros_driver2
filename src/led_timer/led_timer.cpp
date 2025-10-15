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
        uint32_t nanosec = msg->header.stamp.nanosec;

        // 1. Get the 11 Least Significant Bits (LSBs) of the seconds
        uint32_t sec_lsb11 = static_cast<uint32_t>(sec & 0x7FFu); // 0x7FF is the mask for 11 bits

        // 2. Get the 7 Most Significant Bits (MSBs) of the nanoseconds
        // A full nanosecond value uses ~30 bits, so we shift right by (30-7)=23
        uint32_t nsec_msb7 = nanosec >> 23;

        // 3. Combine them into an 18-bit value: [11 SEC BITS | 7 NANOSEC BITS]
        int32_t unix_led = (sec_lsb11 << 7) | nsec_msb7;         

        std_msgs::msg::Int32 out_msg;
        out_msg.data = static_cast<int32_t>(unix_led);
        publisher_->publish(out_msg);

        RCLCPP_INFO_ONCE(this->get_logger(),
                         "First PointCloud2 received; publishing to /led_timestep");

        if (logfile_.is_open()) {
            logfile_.write(reinterpret_cast<const char*>(&sec), sizeof(sec));
            uint64_t nanosec_64 = nanosec; 
            logfile_.write(reinterpret_cast<const char*>(&nanosec_64), sizeof(nanosec_64));
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