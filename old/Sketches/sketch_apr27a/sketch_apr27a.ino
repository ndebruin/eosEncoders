/*******************************************************************************
 *
 *   Electronic Theatre Controls
 *
 *   lighthack - USB Test Sketch
 *
 *   (c) 2017 by ETC
 *
 *
 *   This code provides a minimal OSC/USB implementation designed for
 *   troubleshooting issues with the other DIY modules. It does not interface
 *   with any hardware; it simply attempts to connect with Eos. Upon doing so,
 *   it begins periodically sending /eos/ping commands which can then be viewed
 *   in the Eos application to verify that the OSC pipe is working.
 ******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
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

 /*******************************************************************************
  * Macros and Constants
  ******************************************************************************/
const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";

/*******************************************************************************
 * Local Types
 ******************************************************************************/

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*******************************************************************************
 * Local Functions
 ******************************************************************************/

/*******************************************************************************
 * Given an unknown OSC message we check to see if it's a handshake message.
 * The handshake message must be replied to for Eos to recognize this device
 * as an OSC device.
 *
 * Parameters:
 *  msg - The OSC message of unknown importance
 *
 * Return Value: void
 *
 ******************************************************************************/
void parseOSCMessage(String& msg)
{
    // check to see if this is the handshake string
    if (msg.indexOf(HANDSHAKE_QUERY) != -1)
    {
        // handshake string found!
        SLIPSerial.beginPacket();
        SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
        SLIPSerial.endPacket();
    }
}

/*******************************************************************************
 * Here we prepare to communicate OSC with Eos by setting up SLIPSerial. Once
 * we are done with setup() we pass control over to loop() and never call
 * setup() again.
 *
 * Parameters: none
 *
 * Return Value: void
 *
 ******************************************************************************/
void setup()
{
    SLIPSerial.begin(115200);
    // This is a hack around an Arduino bug. It was taken from the OSC library
    // examples
#ifdef BOARD_HAS_USB_SERIAL
    while (!SerialUSB);
#else
    while (!Serial);
#endif

    // this is necessary for reconnecting a device because it needs some time
    // for the serial port to open, but meanwhile the handshake message was
    // sent from Eos
    SLIPSerial.beginPacket();
    SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
    SLIPSerial.endPacket();
}

/*******************************************************************************
 * Main loop: manage OSC I/O. Send a ping command to Eos every second.
 *
 * Parameters: none
 *
 * Return Value: void
 *
 ******************************************************************************/
void loop()
{
    static String curMsg;
    static unsigned long lastTimeSent;
    static int32_t pingNum;
    unsigned long curTime;
    int size;

    // Check to see if any OSC commands have come from Eos that we need to respond to.
    size = SLIPSerial.available();
    if (size > 0)
    {
        // Fill the msg with all of the available bytes
        while (size--)
            curMsg += (char)(SLIPSerial.read());
    }
    if (SLIPSerial.endofPacket())
    {
        parseOSCMessage(curMsg);
        curMsg = String();
    }

    // Send a ping every second.
    curTime = millis();
    if (curTime - lastTimeSent > 1000)
    {
        OSCMessage ping("/eos/ping");
        ping.add(pingNum++);
        SLIPSerial.beginPacket();
        ping.send(SLIPSerial);
        SLIPSerial.endPacket();
        lastTimeSent = curTime;
    }
}
