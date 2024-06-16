#include "header.h"
#include <RadioLib.h>

CC1101 radio = new Module(PIN_FUNK_CS, PIN_RADIO_GDO0, RADIOLIB_NC, PIN_RADIO_GDO2);

char funk_init = 0;

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;
void setFlag(void);
void radio_setup()
{
  // initialize CC1101 with default settings
  Serial.print(F("[CC1101] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true)
      ;
  }

  // set the function that will be called
  // when new packet is received
  radio.setPacketReceivedAction(setFlag);

  // start listening for packets
  Serial.print(F("[CC1101] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE)
  {
    radio.setFrequency(433);
    Serial.println(F("success!"));
    funk_init = 1;
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true)
      ;
  }

  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // radio.standby()
  // radio.sleep()
  // radio.transmit();
  // radio.receive();
  // radio.readData();
}

// flag to indicate that a packet was received
volatile bool receivedFlag = false;
void setFlag(void)
{
  // we got a packet, set the flag
  receivedFlag = true;
}

void radio_recieve_buffer()
{
  if (!funk_init)
  {
    radio_setup();
  }
  // check if the flag is set
  if (receivedFlag)
  {
    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    String str;
    int state = radio.readData(str);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int numBytes = radio.getPacketLength();
      int state = radio.readData(byteArr, numBytes);
    */

    if (state == RADIOLIB_ERR_NONE)
    {
      // packet was successfully received
      Serial.println(F("[CC1101] Received packet!"));

      // print data of the packet
      Serial.print(F("[CC1101] Data:\t\t"));
      Serial.println(str);

      // print RSSI (Received Signal Strength Indicator)
      // of the last received packet
      Serial.print(F("[CC1101] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));

      // print LQI (Link Quality Indicator)
      // of the last received packet, lower is better
      Serial.print(F("[CC1101] LQI:\t\t"));
      Serial.println(radio.getLQI());
    }
    else if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
      // packet was received, but is malformed
      Serial.println(F("CRC error!"));
    }
    else
    {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);
    }

    // put module back to listen mode
    radio.startReceive();
  }
}



