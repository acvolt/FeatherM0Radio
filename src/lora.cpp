#include "lora.h"

void lora::init() {
    _driver = new RH_RF95(RFM95_CS, RFM95_INT);
    _manager = new RHReliableDatagram(*_driver, CLIENT_ADDRESS);
    Serial.println("Beginning Setup Process");

    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, LOW);
    delay(500);
    digitalWrite(RFM95_RST, HIGH);
    delay(100);
    if(!_driver->init()) {
      Serial.println("RFM95 Radio Init Failed");
      while (1);
    }
    _driver->setModemConfig(RH_RF95::ModemConfigChoice::Bw500Cr45Sf128);
    if (!_driver->setFrequency(RFM69_FREQ)) {
      Serial.println("Frequency Set Failed");
      while(1);
    }
    _driver->setTxPower(20, false);
    _manager->setThisAddress(CLIENT_ADDRESS);
    _manager->setHeaderFrom(CLIENT_ADDRESS);


}

void lora::get_message_from_server()
{
  if (_manager->available())
  {
    //Message is available
    //Serial.println("Receiving Something from server");
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (_manager->recvfrom(buf, &len, &from))
    {
      //Serial.print("got request from : 0x");
      //Serial.print(from, HEX);
      //Serial.print(": ");
      //Serial.print(" Length->"); Serial.println((len));
      //Serial.printf("Type %d MAV %x Payload %d\n", buf[0], buf[1], buf[2]);
      //Serial.println((char*)buf);
      copy_radio_message((char*)buf, len);
      if (buf[0] == 0x01)
      {
        Serial1.println((char*)&(buf[1]));
        Serial.println((char*)&(buf[1]));
      }
      
    }
    else
      Serial.printf("Didn't actually get anything!\n");
  }

}

void lora::send_message_to_server(MavMsg msg)
{
  bool sent = false;
  //Serial.printf("Drone->GCS Message Type 0x%02x Length %d \n", msg.buf[0], msg.msg_len);
  if (msg.msg_len+1 < RH_MAX_MESSAGE_LEN)
  {
    //Serial.printf("Single Shot Message of length %d type %02x \n ", msg.msg_len, msg.buf[0]);
    sent = _manager->sendto(msg.buf, msg.msg_len, SERVER_ADDRESS);
    //Serial.printf("Send Response was %d \n", sent);

  }
  else {
    Serial.print("Double Shot Message");
    uint8_t new_message1[(msg.msg_len+2)/2];
    uint8_t new_message2[(msg.msg_len+2)/2];
    new_message1[0] = 0xf0;
    new_message2[0] = 0xf1;
    for (int i = 0; i < (int)(msg.msg_len/2); i++)
    {
      new_message1[i+1] = msg.buf[i];
    }
    _manager->sendto(new_message1, (int)(msg.msg_len/2), 20);
    for (int i = 0; i < msg.msg_len; i++)
    {
      new_message2[i+1] = msg.buf[i + 1 + msg.msg_len/2];
    }
    _manager->sendto(new_message2, (int)(msg.msg_len/2), 20);
    Serial.print("Fragmented Message :-(");
  }

}

void lora::copy_radio_message(char *inbuf, uint16_t insize)
{
  uint8_t qnum=9;
  //Serial.printf("Copying Radio Message into Serial Q of length %d \n", insize);
  for (int i=9; i>=0; i--)
  {
    if (radio_buffer[i].state == 0)
    {
      //Serial.printf("Found Empty radio buffer of %d \n", i);
      qnum = i;
    }
  }
  for(int i=0; i<= insize; i++) {
    radio_buffer[qnum].buffer.buf[i] =inbuf[i];
  }
  radio_buffer[qnum].buffer.msg_len = insize;
  radio_buffer[qnum].state = 1;
  radio_buffer[qnum].len = insize;
  radio_buffer[qnum].pos = insize;
  return;
}