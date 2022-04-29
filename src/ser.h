#ifndef ser_h
#define ser_h
#include <Arduino.h>
#include "lora.h"
#include "mavmsg.h"
/*
struct Mav {
  uint8_t buffer[281] = {0};
  uint8_t state = 0;
  uint16_t pos = 0;
  uint16_t len = 0;
  uint16_t seq = 0;
}; */



class ser {
    public:
        struct Mav serial_buffer[10];
        uint32_t in_msgs = 0;
        uint32_t out_msgs = 0;


        uint8_t node_address;
        uint8_t gcs_address;
        void init();
        void get_message_from_drone();
        void send_message_to_drone(MavMsg, uint16_t);
    private:
        uint8_t buf[281];
        uint16_t bufpos = 0;
        uint16_t buf_length = 0;
        uint16_t reading = false;
        void copy_serial_message();


};



#endif