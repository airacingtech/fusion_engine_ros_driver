#ifndef FUSION_ENGINE_MESSAGE_LISTENER_HPP
#define FUSION_ENGINE_MESSAGE_LISTENER_HPP

#include "fusion_engine_ros_driver/fusion_engine/message_event.hpp"

namespace fusion_engine {
/**
 * @brief Abstract interface for listening to asynchronous data from a Point One Nav FusionEngine.
 */
class MessageListener {
 public:
  /**
   * @brief Triggers when FusionEngine receives a complete message.
   * @param evt Event that wraps the message data received.
   */
  virtual void receivedFusionEngineMessage(MessageEvent& evt) = 0;
};

} // namespace fusion_engine

#endif