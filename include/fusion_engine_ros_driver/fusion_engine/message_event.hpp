#ifndef FUSION_ENGINE_MESSAGE_EVENT_HPP
#define FUSION_ENGINE_MESSAGE_EVENT_HPP

#include "fusion_engine_ros_driver/fusion_engine/message_type.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace fusion_engine {

/**
 * @brief Data class that wraps message data in a generic object.
 */
class MessageEvent {
 public:
  gps_msgs::msg::GPSFix gps_fix;
  sensor_msgs::msg::NavSatFix nav_sat_fix;
  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::PoseStamped pose;
  MessageType message_type;

  /**
   * @brief Construct a new Message Event object
   * 
   * @param gps_fix_ GPS Fix Message
   */
  MessageEvent(gps_msgs::msg::GPSFix gps_fix_)
      : gps_fix(gps_fix_), message_type(MessageType::GPS_FIX) {}

  /**
   * @brief Construct a new Message Event object
   * 
   * @param nav_sat_fix_ NavSat Fix Message
   */
  MessageEvent(sensor_msgs::msg::NavSatFix nav_sat_fix_)
      : nav_sat_fix(nav_sat_fix_), message_type(MessageType::NAV_SAT_FIX) {}

  /**
   * @brief Construct a new Message Event object
   * 
   * @param imu_ IMU Message
   */
  MessageEvent(sensor_msgs::msg::Imu imu_)
      : imu(imu_), message_type(MessageType::IMU) {}

  /**
   * @brief Construct a new Message Event object
   * 
   * @param pose_ Pose Message
   */
  MessageEvent(geometry_msgs::msg::PoseStamped pose_)
      : pose(pose_), message_type(MessageType::POSE) {}
};

} // namespace fusion_engine

#endif