#ifndef FUSION_ENGINE_MESSAGE_TYPE_HPP
#define FUSION_ENGINE_MESSAGE_TYPE_HPP

namespace fusion_engine {

/**
 * @brief  FusionEngine message type definitions.
 */
enum class MessageType {
  /** 
   * @brief Standard ROS GPSFix message.
   * @see http://docs.ros.org/api/gps_common/html/msg/GPSFix.html
   */
  GPS_FIX = 0,
  /** 
   * @brief Standard ROS NavSatFix message.
   * @see http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
   */
  NAV_SAT_FIX = 1,
  /** 
   * @brief Standard ROS IMU message.
   * @see http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
   */
  IMU = 2,
  /** 
   * @brief Standard ROS POSE message.
   * @see https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
   */
  POSE = 3,
};

} // namespace fusion_engine

#endif