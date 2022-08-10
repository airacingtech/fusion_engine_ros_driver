#ifndef FUSION_ENGINE_BYTE_FRAME_LISTENER_HPP
#define FUSION_ENGINE_BYTE_FRAME_LISTENER_HPP

#include "fusion_engine_ros_driver/fusion_engine/byte_frame_event.hpp"

namespace fusion_engine {

/**
 * @brief Abstract interface for clients listening to byte frames.
 */
class ByteFrameListener {
 public:
  /**
   * @brief Triggers when Point One Gps internal port receives a byte frame.
   * @param evt Event that wraps the message data received.
   */
  virtual void receivedFusionEngineByteFrame(ByteFrameEvent& evt) = 0;
};

} // namespace fusion_engine

#endif