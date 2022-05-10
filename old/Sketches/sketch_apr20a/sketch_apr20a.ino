#include <OSCBoards.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCMatch.h>
#include <OSCMessage.h>
#include <OSCTiming.h>
#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
#else
#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial);
#endif
#include <string.h>

#include <Encoder.h>

Encoder Enc1(2, 3);
Encoder Enc2(A0, A1);

void sendOscMessage(const String &address, float value)
{
  OSCMessage msg(address.c_str());
  msg.add(value);
  SLIPSerial.beginPacket();
  msg.send(SLIPSerial);
  SLIPSerial.endPacket();
}

void sendWheelMove(WHEEL_TYPE type, float ticks)
{
  String wheelMsg("/eos/active/wheel");


    
 

  if (type == PAN)
    wheelMsg.concat("/3");
  else if (type == TILT)
   wheelMsg.concat("/4");


  
}


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
