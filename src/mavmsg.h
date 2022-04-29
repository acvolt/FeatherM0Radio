#include <Arduino.h>
#ifndef mavmsg_h
#define mavmsg_h

struct MavMsg {
  uint8_t buf[281] = {0};
  uint8_t msg_len = 0;
};

struct Mav {
  struct MavMsg buffer;
  uint8_t state = 0;
  uint16_t pos = 0;
  uint16_t len = 0;
  uint16_t seq = 0;
};

#endif