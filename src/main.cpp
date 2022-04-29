#include <Arduino.h>

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <RH_RF95.h>
#include <SPI.h>
#include "lora.h"
#include "ardupilotmega/mavlink.h"
#include "ser.h"
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
#define RFM95_CS 32

//Board 1
//#define RFM69_INT 15
//Board 2
#define RFM95_INT 14



lora a = lora();
ser s = ser();
//#define RFM69_FREQ 915.0
//RH_RF95 driver(RFM95_CS, RFM95_INT);
//RHReliableDatagram manager(driver, CLIENT_ADDRESS);
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
Serial.begin(115200);
while(!Serial)
  delay(1);
Serial1.begin(57600);
Serial.println("Init A");
a.init();
Serial.println("Init B");
s.init();
/*
    Serial.println("Beginning Setup Process");

    digitalWrite(RFM95_RST, LOW);
    delay(500);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);

    if(!driver.init()) {
      Serial.println("RFM95 Radio Init Failed");
      while (1);
    }
*/
}
bool once = false;
void loop() {
a.get_message_from_server();
s.get_message_from_drone();
for (int i=0; i<10; i++)
{
  //Serial.printf("Queue %d is: %d \n", i, a.radio_buffer[i].state);
  if (a.radio_buffer[i].state == 1)
  {
    //Serial.printf("Queue %d is: %d \n", i, a.radio_buffer[i].state);
    s.send_message_to_drone(a.radio_buffer[i].buffer, a.radio_buffer[i].len);
    a.radio_buffer[i].state = 0;
  }
}
for (int i=0; i<10; i++)
{
  
  if (s.serial_buffer[i].state == 1)
  {
    //Serial.printf("Serial Queue %d is: %d \n", i, s.serial_buffer[i].state);
    a.send_message_to_server(s.serial_buffer[i].buffer);
    s.serial_buffer[i].state = 0;
  }
}
delay(1);
  
  

}
/*

#define CLIENT_ADDRESS 17
#define SERVER_ADDRESS 20

//Board 1
//#define RFM69_RST 27
//Board 2
#define RFM69_RST 4

//Board 1
//#define RFM69_CS 33
//Board 2
#define RFM69_CS 32

//Board 1
//#define RFM69_INT 15
//Board 2
#define RFM69_INT 14


#define RFM69_FREQ 914.0

mavlink_system_t mavlink_system = {
  230,
  1
};
mavlink_status_t status;
mavlink_message_t msg;

//RH_RF69 driver(RFM69_CS, RFM69_INT);

//RHReliableDatagram manager(driver, CLIENT_ADDRESS);

//Mavlink Message Queue - up to 10 messages that are up to 280 bytes in length
uint8_t mav_buf[10][281]; //The actual messages
uint8_t mav_buf_state[10]  = {0}; //Tells us if the buffer if filled or not
uint16_t mav_buf_size[10] = {0}; //Expected buffer sized based on message length

uint8_t read_buffer[281] = {0};
uint16_t read_pos = 0;
uint8_t radioin_buffer[281] = {0};
uint16_t radioin_pos = 0;
int read_length = -1;

uint16_t read_v1_messages = 0;
uint16_t read_v2_messages = 0;


struct Mav {
  uint8_t buffer[281] = {0};
  uint8_t state = 0;
  uint16_t pos = 0;
  uint16_t len = 0;
};
struct Mav radio_buffer[10];
struct Mav serial_buffer[10];

void setup()
{

  Serial.begin(9600);
  Serial1.begin(57600);
  while(!Serial)
  ;
  Serial.println("Beginning Setup Process");
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(500);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  //if(!driver.init()) {
  //  Serial.println("RFM69 Radio Init Failed");
  //  while (1);
  //}

  

  /*uint8_t send_tries = 200;

  uint8_t data[] = "test broadcast";
  while (send_tries > 0)
  {
    
    bool sent = false;
    sent = driver.send(data, sizeof(data));
    Serial.print("Broadcast Send State is: "); Serial.println(sent);
    send_tries--;
    delay(200);
  }*/
  /*while (!driver.available())
  {
    Serial.println("Still Waiting");
    delay(100);
  }

      uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (driver.recv(buf, &len))
    {
//      RH_RF69::printBuffer("request: ", buf, len);
      Serial.print("got request: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(driver.lastRssi(), DEC);

    }*/
    /*
  driver.setSyncWords();
  if (!manager.init())
    Serial.println("RFM Init Failed");
  driver.setTxPower(14, true);
  if (!driver.setFrequency(RFM69_FREQ))
    Serial.println("Set Frequency Failed");
  Serial.println("So Far So Good");


}

uint8_t data[] = "And hello to somebody";
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void copy_serial_message()
{
  uint8_t qnum=9;
  Serial.println("Copying Serial Message into Radio Q");
  for (int i=9; i>=0; i--)
  {
    if (serial_buffer[i].state == 0)
    {
      Serial.printf("Found Empty serial buffer of %d \n", i);
      qnum = i;
    }
  }
  for(int i=0; i<= read_pos; i++) {
    serial_buffer[qnum].buffer[i] = read_buffer[i];
  }
  serial_buffer[qnum].state = 1;
  serial_buffer[qnum].len = read_pos;
  serial_buffer[qnum].pos = read_pos;
  read_pos = 0;
  return;
}

void copy_radio_message(char *inbuf, uint16_t insize)
{
  uint8_t qnum=9;
  Serial.println("Copying Radio Message into Serial Q");
  for (int i=9; i>=0; i--)
  {
    if (radio_buffer[i].state == 0)
    {
      Serial.printf("Found Empty radio buffer of %d \n", i);
      qnum = i;
    }
  }
  for(int i=0; i<= insize; i++) {
    radio_buffer[qnum].buffer[i] =inbuf[i];
  }
  radio_buffer[qnum].state = 1;
  radio_buffer[qnum].len = insize;
  radio_buffer[qnum].pos = insize;
  return;
}

void copy_message()
{
  Serial.println("Copying Message from read into queue");
  uint8_t qnum = 100;
  for (int i=9; i>=0; i--) {
    Serial.println("Looking for Buffer");
    if (mav_buf_state[i] == 0)
    {
      Serial.printf("Found Empty Buffer %d \n", i);
      qnum = i;
    }
  }
  for (int i = 0; i<= read_pos; i++)
  {
    mav_buf[qnum][i] = read_buffer[i];
  }
  mav_buf_state[qnum] = 1;
  mav_buf_size[qnum] = read_pos;
  read_pos = 0;
      
  return;
  }

void get_serial_traffic()
{
  while(Serial1.available() > 0)
  {
      uint8_t b = Serial1.read();
      if (read_pos == 0)
      {
        if (b == 0xfd || b == 0xfe)
        {
          //Start of a new message
          read_buffer[read_pos] = b;
          read_pos++;

        }
      }
      else if (read_pos == 1) {
        read_buffer[1] = b; //This will be the message length
        read_pos++;
      }
      else if (read_pos == 2) {
        //This is i flags in v2 or seq of v1
        if (read_buffer[0] == 0xfe)
        {
          read_length = read_buffer[1] + 7;
        }
        else if (read_buffer[0] == 0xfd)
        {
          read_length = read_buffer[1];
          if (b & 0x01)
            read_length += 13;
        }
        read_buffer[2] = b;
        read_pos++;
      }
      else if (read_pos <= read_length)
      {
        read_buffer[read_pos] = b; 
        read_pos++;
        
      }
      else
      {
        //This should be the last byte...  Famous last words, eh?
        read_buffer[read_pos] = b;
        copy_serial_message();
        //copy_message();
        

      }
      
  }

}
  
void send_message_to_server()
{
  for (int i=0; i<10; i++)
  {
    if (mav_buf_state[i]==1)
    {
      if(mav_buf_size[i] < 59)
      {
        Serial.printf("Q %d Found Message Small Enough to Send in 1 shot size %d type %x \n", i, mav_buf_size[i], mav_buf[i][0]);
        manager.sendtoWait(mav_buf[i], (uint8_t)mav_buf_size[i]+1, SERVER_ADDRESS);
        
        mav_buf_state[i] = 0;
      }
    }
  }

  return;
}

void get_message_from_server()
{
  if (manager.available())
  {
    //Message is available
    Serial.println("Receiving Something from server");
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.print(" Length->"); Serial.println((len));
      Serial.printf("Type %d MAV %x Payload %d\n", buf[0], buf[1], buf[2]);
      Serial.println((char*)buf);
      copy_radio_message((char*)buf, len);
      if (buf[0] == 0x01)
      {
        Serial1.println((char*)&(buf[1]));
        Serial.println((char*)&(buf[1]));
      }
      

      // Send a reply back to the originator client
      if (!manager.sendtoWait(data, sizeof(data), from))
        Serial.println("sendtoWait failed");
    }
  }


  
  return;
}
void send_message_to_drone()
{
  for (int i=0; i<10; i++)
  {
    if (radio_buffer[i].state == 1)
    {
      for(int j=0; j<radio_buffer[i].pos; j++)
      {
        Serial1.write(radio_buffer[i].buffer[j]);
      }
      radio_buffer[i].state = 0;
    }
  }
  return;
}

void loop()
{
  get_serial_traffic();
  send_message_to_server();
  get_message_from_server();
  send_message_to_drone();
}
*/
/*
void nogo()
{
  while (Serial1.available() > 0)
  {
    
    uint8_t b = Serial1.read();
    Serial.print("Found something on com1"); Serial.print(b, HEX); Serial.println(" Data");
    if (mavlink_parse_char(MAVLINK_COMM_0, b, &msg, &status))
    {
      printf("Received message with ID %d, sequence: %d from component %d of system %d with magic %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid, msg.magic);
      if (msg.len < 50)
      {
        buf[0] = msg.magic;
        buf[1] = msg.len;
        if (msg.magic == 0xfe)
        {
          buf[2] = msg.incompat_flags;
          buf[3] = msg.compat_flags;
          buf[4] = msg.seq;
          buf[5] = msg.sysid;
          buf[6] = msg.compid;
          buf[7] = (msg.msgid & 0xFF0000) >> 16;
          buf[8] = (msg.msgid & 0x00FF00) >> 8;
          buf[9] = (msg.msgid & 0x0000FF);
          if (msg.len > 0)
          {
            for (int j = 10; j < (10+msg.len); j++)
            {
              buf[j] = msg.payload64[j-10];
            }
          }

          
        }
        else {

        }
        manager.sendtoWait((char*)&msg, sizeof(msg), 1);
      }
    }
  }

  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    Serial.println("Receiving Something");
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.print(" Length->"); Serial.println((len));
      Serial.printf("Type %d MAV %x Payload %d\n", buf[0], buf[1], buf[2]);
      Serial.println((char*)buf);
      if (buf[0] == 0x01)
      {
        Serial1.println((char*)&(buf[1]));
        Serial.println((char*)&(buf[1]));
      }
      

      // Send a reply back to the originator client
      if (!manager.sendtoWait(data, sizeof(data), from))
        Serial.println("sendtoWait failed");
    }
  }
  else
    Serial.println("Nothing Available");
  delay(200);
} */

