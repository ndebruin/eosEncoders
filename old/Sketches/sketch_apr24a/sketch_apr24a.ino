//Libraries
#include "Arduino.h"
#include <OSCMessage.h>
#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
#else
#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial);
#endif
#include "eOS.h"

//Strings for communication
const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";
const String PING_QUERY = "enc_hello";
const String PARAMETER_QUERY = "/eos/out/param/";
const String NO_PARAMETER = "none"; // none is a keyword used when there is no parameter

//Variable definition
#define PING_AFTER_IDLE_INTERVAL    2500
#define TIMEOUT_AFTER_IDLE_INTERVAL 5000
#define PARAMETER_MAX    14 // number of parameters must even
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;
int8_t idx = 0; // start with parameter index 2 must even


struct Parameter {
  String name;
  String displayName;
  float value;
};

struct Parameter parameter[PARAMETER_MAX] = {
  {NO_PARAMETER, "------"},
  {"Intens"},
  {"Pan"},
  {"Tilt"},
  {"Zoom"},
  {"Edge"},
  {"Iris"},
  {"Diffusn"},
  //{"Hue"},
  //{"Saturatn"},
  {"Red"},
  {"Green"},
  {"Blue"},
  {"Cyan"},
  {"Magenta"},
  {"Yellow"},
  //{"cto", "CTO"},
  //{"frame_assembly", "Assembly"},
  //{"thrust_A", "ThrustA"},
  //{"angle_A", "AngleA"},
  //{"thrust_B", "ThrustB"},
  //{"thrust_B", "AngleB"},
  //{"thrust_C", "ThrustC"},
  //{"angle_C", "AngleC"},
  //{"thrust_D", "ThrustD"},
  //{"angle_D", "AngleD"}
};



//Send Handshake Reply
void initEOS()
{
  SLIPSerial.beginPacket();
  SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
  SLIPSerial.endPacket();
}

//Acquire OSC message and send to parser
void grabOSCmsg()
{
  static String curMsg;
  int size;
  size = SLIPSerial.available();
  if (size > 0) {
    while (size--) curMsg += (char)(SLIPSerial.read());
  }
  if (SLIPSerial.endofPacket()) {
    parseOSCmsg(curMsg);
    lastMessageRxTime = millis();
    curMsg = String();
    return curMsg;
  }
}

//Break apart messages from Eos
void parseOSCmsg(String msg)
{
  static String parseMsg;
  if (msg.indexOf(HANDSHAKE_QUERY) != -1) { // Check to see if this is the handshake string
    initEOS();
    return;
  }
  else {
    OSCMessage oscmsg;
    oscmsg.fill((uint8_t*)msg.c_str(), (int)msg.length());
    parseMsg = PARAMETER_QUERY + parameter[idx].name;
    if (msg.indexOf(parseMsg) != -1) {
      parameter[idx].value = oscmsg.getFloat(0);
      parseMsg = String();
      return;
    }
    parseMsg = PARAMETER_QUERY + parameter[idx + 1].name;
    if (msg.indexOf(parseMsg) != -1) {
      parameter[idx + 1].value = oscmsg.getFloat(0);
      parseMsg = String();
      return;
    }
  }
}

void setup()
{
  //Hack from OSC library to deal with a bug
  SLIPSerial.begin(115200);
#ifdef BOARD_HAS_USB_SERIAL
  while (!SerialUSB);
#else
  while (!Serial);
#endif

  initEOS(); //for hotplug on boards like Uno (No native USB)
}

void loop() {
  //grabOSCmsg();
  initEOS();
}
