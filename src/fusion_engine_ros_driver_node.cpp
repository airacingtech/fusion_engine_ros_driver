#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "gps_msgs/msg/gps_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/string.hpp"

#include "fusion_engine_ros_driver/fusion_engine/core.h"
#include "fusion_engine_ros_driver/fusion_engine_ros_driver_node.hpp"

/*
 * Fusion Engine Driver Node publishes realtime GPS/IMU messages using the Fusion Engine Protocol
 */
class FusionEngineDriverNode : public fusion_engine::MessageListener,
                               public rclcpp::Node {
 public:
  FusionEngineDriverNode()
      : Node("fusion_engine_ros_driver_node"),
        gps(FusionEngineDriver::getInstance()),
        connection_type_("udp"),
        tcp_ip_(""),
        tcp_port_(30201),
        udp_port_(23456),
        frame_id_("") {
    // Allow dynamic parameterization.
    connection_type_ =
        this->declare_parameter("connection_type", connection_type_);
    tcp_ip_ = this->declare_parameter("tcp_ip", tcp_ip_);
    tcp_port_ = this->declare_parameter("tcp_port", tcp_port_);
    udp_port_ = this->declare_parameter("udp_port", udp_port_);
    frame_id_ = this->declare_parameter("frame_id", frame_id_);

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "pose", rclcpp::SensorDataQoS());
    gps_fix_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>(
        "gps_fix", rclcpp::SensorDataQoS());
    nav_sat_fix_publisher_ =
        this->create_publisher<sensor_msgs::msg::NavSatFix>(
            "nav_sat_fix", rclcpp::SensorDataQoS());
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS());

    timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&FusionEngineDriverNode::serviceLoopCb, this));

    gps.initialize(this, connection_type_, tcp_ip_, tcp_port_, udp_port_);
    gps.addMessageListener(*this);
  }

  /**
   * Callback function triggered by the FusionEngineClient receiving a complete message.
   * @param evt GPS/IMU data.
   * @return Nothing.
   */
  void receivedFusionEngineMessage(fusion_engine::MessageEvent& evt) {
    auto time = now();
    if (evt.message_type == fusion_engine::MessageType::GPS_FIX) {
      evt.gps_fix.header.frame_id = frame_id_;
      evt.gps_fix.header.stamp = time;
      gps_fix_publisher_->publish(evt.gps_fix);
    } else if (evt.message_type == fusion_engine::MessageType::NAV_SAT_FIX) {
      evt.nav_sat_fix.header.frame_id = frame_id_;
      evt.nav_sat_fix.header.stamp = time;
      nav_sat_fix_publisher_->publish(evt.nav_sat_fix);
    } else if (evt.message_type == fusion_engine::MessageType::IMU) {
      evt.imu.header.frame_id = frame_id_;
      evt.imu.header.stamp = time;
      imu_publisher_->publish(evt.imu);
    } else if (evt.message_type == fusion_engine::MessageType::POSE) {
      evt.pose.header.frame_id = frame_id_;
      evt.pose.header.stamp = time;
      pose_publisher_->publish(evt.pose);
    }
  }

 private:
  FusionEngineDriver& gps;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      nav_sat_fix_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string connection_type_;
  std::string tcp_ip_;
  int tcp_port_;
  int udp_port_;
  std::string frame_id_;

  /**
   * Initiate gps unit to read data.
   */
  void serviceLoopCb() {
    RCLCPP_INFO(this->get_logger(), "Service");
    timer_->cancel(); // one-time enrty into service loop
    gps.service();
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionEngineDriverNode>());
  rclcpp::shutdown();
  return 0;
}