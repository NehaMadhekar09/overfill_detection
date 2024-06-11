// #include <chrono>
#include <string>

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class LidarListener
 * @brief A ROS 2 node for subscribing to Lidar data and detect overfill based
 * on that.
 */
class LidarListener : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the LidarListener class.
   */
  LidarListener() : Node("lidar_listener") {

    // Subscribe to LaserScan topic
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_plugin/out", 10,
        std::bind(&LidarListener::laser_callback, this, std::placeholders::_1));
  }

private:
  /**
   * @brief Callback function for processing laser scan data.
   * @param msg Laser scan data message.
   */
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Simulate overfill detection based on LaserScan data
    auto laser_data = msg->ranges;

    // size_t data_length = laser_data.size();
    // RCLCPP_INFO(this->get_logger(), "Length of laser data: %zu",
    // data_length);

    // LaserScan message provides an array of ranges at different angles
    // We need to provide minimum and maximum angles for laserscan. As per the
    // selected Sick sensor for this application, minimum angle is -100 degrees
    // and maximum angle is 100 degrees Size of data is determined by Sample
    // field in world file. In this case it is set to 400. So, laser scan
    // samples are taken at 0.5 degrees gap. Limits of for loop are decided
    // based on number of samples and distance of the tote from sensor.

    for (int i = 100; i < 300; i++) {
      // 0.85 distance is decided based on tote size and distance of the tote
      // from sensor.
      if (laser_data[i] < 0.85) {
        RCLCPP_INFO(this->get_logger(), "Overfilled ...");
        return;
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscriber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarListener>());
  rclcpp::shutdown();
  return 0;
}
