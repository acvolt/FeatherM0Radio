#include "ser.h"
#include "lora.h"
#include <Arduino.h>

void ser::init()
{

}
void ser::send_message_to_drone(MavMsg msg, uint16_t length)
{
    //Serial.printf("Sending Message to Drone on Serial1 of length %d other length %d \n", msg.msg_len, length);
    digitalWrite(LED_BUILTIN, HIGH);
    for(int i=0; i< length; i++)
    {
        //Serial.print(msg.buf[i] & 0xFF, HEX); Serial.print(" : ");
        Serial1.write(msg.buf[i]);
    }
    //Serial.println(' ');
    digitalWrite(LED_BUILTIN, LOW);
}

void ser::get_message_from_drone()
{
    //Serial.printf("Serial 1 has %d bytes available \n", Serial1.available());
    while (Serial1.available())
    {
        
        uint8_t inbyte;
        Serial1.readBytes(&inbyte, 1);
        if ((inbyte == 0xfe || inbyte == 0xfd) && !reading) {
            //This is the start of a message
            bufpos = 1;
            reading = true;
            buf[0] = inbyte;
        }
        else {
            //Reading in the packet
            switch (bufpos) {
                case 1:
                case 2:
                    buf[bufpos] = inbyte;
                    bufpos++;
                    break;
                case 3:
                    buf[bufpos] = inbyte;
                    bufpos++;
                    switch(buf[0]) {
                        case 0xfe:
                            buf_length = (uint8_t) 8 + buf[1];
                            break;
                        case 0xfd:
                            if (buf[2]&0x01)
                                buf_length = (uint8_t)25 + buf[1];
                            else
                                buf_length = (uint8_t)12 + buf[1]; 
                    }
                    break;
                default:
                    buf[bufpos] = inbyte;
                    bufpos++;
                    if (bufpos >= buf_length)
                        {
                        //Serial.println("Complete Message Received from Serial");
                    
                        copy_serial_message();
                        bufpos = 0;
                        reading = false;
                        }
                    
            }
        }
    }
}

void ser::copy_serial_message()
{
  uint8_t qnum=9;
  //Serial.printf("Copying serial Message into Radio Q (bufpos) %d \n", bufpos);
  for (int i=9; i>=0; i--)
  {
    if (serial_buffer[i].state == 0)
    {
      //Serial.printf("Found Empty radio buffer of %d \n", i);
      qnum = i;
    }
  }
  //Serial.println("Outbound Message (to GCS)");
  for(int i=0; i<= bufpos; i++) {
      //Serial.printf(" %02x :", buf[i]);
    serial_buffer[qnum].buffer.buf[i] =buf[i];
  }
  //Serial.print("\n");
  serial_buffer[qnum].buffer.msg_len = bufpos;
  serial_buffer[qnum].state = 1;
  serial_buffer[qnum].len = bufpos;
  serial_buffer[qnum].pos = bufpos;
  return;
}