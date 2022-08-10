#ifndef FUSION_ENGINE_BYTE_FRAME_EVENT_HPP
#define FUSION_ENGINE_BYTE_FRAME_EVENT_HPP

#include <arpa/inet.h>
#include <cstring>

namespace fusion_engine {

/**
 * @brief Data class that wraps a received byte frame into a generic object.
 */
class ByteFrameEvent {
 public:
  uint8_t* frame;
  size_t bytes_read;
  
  /**
   * @brief Construct a new Byte Frame Event object
   * 
   * @param frame Frame used to initialize object
   * @param bytes_read Number of bytes
   */
  ByteFrameEvent(uint8_t* frame, size_t bytes_read)
      : frame{frame}, bytes_read{bytes_read} {}
};

} // namespace fusion_engine

#endif