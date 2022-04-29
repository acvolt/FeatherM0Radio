#include "mavmsg.h"
#define CLIENT_ADDRESS 20

#define SERVER_ADDRESS 17

//Board 1
//#define RFM69_RST 27
//Board 2
#define RFM95_RST 4

//Board 1
//#define RFM69_CS 33
//Board 2
#define RFM95_CS 8

//Board 1
//#define RFM69_INT 15
//Board 2
#define RFM95_INT 3


#define RFM69_FREQ 914.0

#ifndef Lora_h
#define Lora_h
#include <Arduino.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#define HEARTBEAT_INTERVAL 500





class lora {
    public:
        struct Mav radio_buffer[10];
        uint8_t node_address;
        uint8_t gcs_address;
        uint8_t rssi;
        uint8_t last_heartbeat;
        void init();
        void get_message_from_server();
        void send_message_to_server(MavMsg);
        void heartbeat();
    private:
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        RH_RF95* _driver;
        RHReliableDatagram* _manager;
        void copy_radio_message(char *, uint16_t);


};





#endif