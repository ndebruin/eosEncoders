/*******************************************************************************
   Includes
 ******************************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <OSCBoards.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCMatch.h>
#include <OSCMessage.h>
#include <OSCTiming.h>
#include <string.h>
#ifdef BOARD_HAS_USB_SERIAL
    #include <SLIPEncodedUSBSerial.h>
    SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
#else
    #include <SLIPEncodedSerial.h>
    SLIPEncodedSerial SLIPSerial(Serial);
#endif
#include <U8x8lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif


/*******************************************************************************
   Macros and Constants
 ******************************************************************************/
#define SHIFT_BTN 1

#define PGUP_BTN 2
#define PGDN_BTN 3

const int sid = 4, sclk = 5, rs = 6, rst = 7, cs = 8;

#define ENC1_BTN 10
#define ENC1_DT 11
#define ENC1_CLK 12

#define ENC2_BTN 17
#define ENC2_DT 18
#define ENC2_CLK 19

#define ENC3_BTN 14
#define ENC3_DT 15
#define ENC3_CLK 16

#define ENC4_BTN 21
#define ENC4_DT 22
#define ENC4_CLK 23

#define SUBSCRIBE 1
#define UNSUBSCRIBE 0

#define EDGE_DOWN 1
#define EDGE_UP 0

#define FORWARD 0
#define REVERSE 1

// Change these values to switch which direction increase/decrease pan/tilt
#define ENC1_DIR FORWARD
#define ENC2_DIR FORWARD
#define ENC3_DIR FORWARD
#define ENC4_DIR FORWARD

// Use these values to make the encoder more coarse or fine. This controls
// the number of wheel "ticks" the device sends to Eos for each tick of the
// encoder. 1 is the default and the most fine setting. Must be an integer.

// changed to an int so they can be modified at run time.
int ENC1_SCALE = 1;
int ENC2_SCALE = 1;
int ENC3_SCALE = 1;
int ENC4_SCALE = 1;

// added variables to set global "coarse" and "normal" encoder modes
int encNorm = 1;
int encCoarse = 8;

#define SIG_DIGITS 2 // Number of significant digits displayed

#define OSC_BUF_MAX_SIZE 512

const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";

const char VERSION_STRING[] = "Version: 0.0.1";

// Change these values to alter how long we wait before sending an OSC ping
// to see if Eos is still there, and then finally how long before we
// disconnect and show the splash screen
// Values are in milliseconds
#define PING_AFTER_IDLE_INTERVAL 5000
#define TIMEOUT_AFTER_IDLE_INTERVAL 15000


/*******************************************************************************
   Local Types
 ******************************************************************************/
enum WHEEL_TYPE
{
    ENC1,
    ENC2,
    ENC3,
    ENC4
};

const char deg = char(176);

enum WHEEL_UNIT
{
    Intens = '%',
    Red = '%',
    Green = '%',
    Blue = '%',
    White = '%',
    Amber = '%',
    Edge = '%',
    Zoom = deg,
    Pan = deg,
    Tilt = deg
};

enum WHEEL_MODE
{
    COARSE,
    FINE
};

struct Encoder
{
    uint8_t pinA;
    uint8_t pinB;
    int pinAPrevious;
    int pinBPrevious;
    float pos;
    uint8_t direction;
    String name;
    int type;
    char unit;
};

struct Encoder enc1Wheel;
struct Encoder enc2Wheel;
struct Encoder enc3Wheel;
struct Encoder enc4Wheel;

/*******************************************************************************
   Global Variables
 ******************************************************************************/

// initialize the display
U8X8_ST7565_LM6059_4W_SW_SPI display(sclk,sid,cs,rs,rst);	// Adafruit ST7565 GLCD

bool updateDisplay = false;
bool connectedToEos = false;
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;

/*******************************************************************************
   Local Functions
 ******************************************************************************/

/*******************************************************************************
   Issues all our subscribes to Eos. When subscribed, Eos will keep us updated
   with the latest values for a given parameter.

   Parameters:  none

   Return Value: void

 ******************************************************************************/
void issueSubscribes()
{
    // Add a filter so we don't get spammed with unwanted OSC messages from Eos
    OSCMessage filter("/eos/filter/add");
    filter.add("/eos/out/param/*");
    filter.add("/eos/out/ping");
    filter.add("/eos/out/active/wheel/*");
    SLIPSerial.beginPacket();
    filter.send(SLIPSerial);
    SLIPSerial.endPacket();

    /*if (true)
    {
        // subscribe to Eos pan & tilt updates
        OSCMessage subOne("/eos/subscribe/param/pan/tilt/zoom/edge");
        subOne.add(SUBSCRIBE);
        SLIPSerial.beginPacket();
        subOne.send(SLIPSerial);
        SLIPSerial.endPacket();

        OSCMessage subTwo("/eos/subscribe/param/cyan/magenta/yellow/cto");
        subTwo.add(UNSUBSCRIBE);
        SLIPSerial.beginPacket();
        subTwo.send(SLIPSerial);
        SLIPSerial.endPacket();

    }*/
}

/*******************************************************************************
   Given a valid OSCMessage (relevant to Pan/Tilt), we update our Encoder struct
   with the new position information.

   Parameters:
    msg - The OSC message we will use to update our internal data
    addressOffset - Unused (allows for multiple nested roots)

   Return Value: void

 ******************************************************************************/
void parseNull(OSCMessage &msg, int addressOffset) {}

void parseEnc1Update(OSCMessage &msg, int addressOffset)
{
    enc1Wheel.pos = msg.getOSCData(0)->getFloat();
    connectedToEos = true; // Update this here just in case we missed the handshake
    updateDisplay = true;
}

void parseEnc2Update(OSCMessage &msg, int addressOffset)
{
    enc2Wheel.pos = msg.getOSCData(0)->getFloat();
    connectedToEos = true; // Update this here just in case we missed the handshake
    updateDisplay = true;
}

void parseEnc3Update(OSCMessage &msg, int addressOffset)
{
    enc3Wheel.pos = msg.getOSCData(0)->getFloat();
    connectedToEos = true; // Update this here just in case we missed the handshake
    updateDisplay = true;
}

void parseEnc4Update(OSCMessage &msg, int addressOffset)
{
    enc4Wheel.pos = msg.getOSCData(0)->getFloat();
    connectedToEos = true; // Update this here just in case we missed the handshake
    updateDisplay = true;
}

void parseParams(OSCMessage &msg, int wheelNum){
    char name[20];
    msg.getOSCData(0)->getString(name);
    int type = msg.getOSCData(1)->getInt();
    float pos = msg.getOSCData(2)->getFloat();
    enc1Wheel.name = name;
    enc1Wheel.pos = pos;
    enc1Wheel.type = type;

}
/*******************************************************************************
   Given an unknown OSC message we check to see if it's a handshake message.
   If it's a handshake we issue a subscribe, otherwise we begin route the OSC
   message to the appropriate function.

   Parameters:
    msg - The OSC message of unknown importance

   Return Value: void

 ******************************************************************************/
void parseOSCMessage(String &msg)
{
    // check to see if this is the handshake string
    if (msg.indexOf(HANDSHAKE_QUERY) != -1)
    {
        // handshake string found!
        SLIPSerial.beginPacket();
        SLIPSerial.write((const uint8_t *)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
        SLIPSerial.endPacket();

        // Let Eos know we want updates on some things
        issueSubscribes();

        // Make our splash screen go away
        connectedToEos = true;
        updateDisplay = true;
    }
    /*else
    {
        // prepare the message for routing by filling an OSCMessage object with our message string
        OSCMessage oscmsg;
        oscmsg.fill((uint8_t *)msg.c_str(), (int)msg.length());
        // route pan/tilt messages to the relevant update function
        if (true)
        {
            oscmsg.route("/eos/out/param/pan", parseEnc1Update);
            oscmsg.route("/eos/out/param/tilt", parseEnc2Update);
            oscmsg.route("/eos/out/param/edge", parseEnc3Update);
            oscmsg.route("/eos/out/param/zoom", parseEnc4Update);
        }
    }*/
}

/*******************************************************************************
   Updates the display with the latest pan and tilt positions.

   Parameters:  none

   Return Value: void

 ******************************************************************************/
void displayStatus()
{
    //  lcd.clear();
    display.setFont(u8x8_font_8x13B_1x2_f);

    if (!connectedToEos)
    {
        display.clear();
        display.drawString(0, 0, "ML Controller");
        display.drawString(0, 2, VERSION_STRING);
        display.drawString(0, 4, "Waiting for");
        display.drawString(0, 6, "connection...");
    }
    else
    {
        display.clear();
        display.drawString(0, 0, enc1Wheel.name.c_str() + ':' + ' ' + char(enc1Wheel.pos) + enc1Wheel.unit);
        display.drawString(0, 2, enc2Wheel.name.c_str() + ':' + ' ' + char(enc2Wheel.pos) + enc2Wheel.unit);
        display.drawString(0, 4, enc3Wheel.name.c_str() + ':' + ' ' + char(enc3Wheel.pos) + enc3Wheel.unit);
        display.drawString(0, 6, enc4Wheel.name.c_str() + ':' + ' ' + char(enc4Wheel.pos) + enc4Wheel.unit);
    }

    updateDisplay = false;
}

/*******************************************************************************
   Initializes a given encoder struct to the requested parameters.

   Parameters:
    encoder - Pointer to the encoder we will be initializing
    pinA - Where the A pin is connected to the Arduino
    pinB - Where the B pin is connected to the Arduino
    direction - Determines if clockwise or counterclockwise is "forward"

   Return Value: void

 ******************************************************************************/
void initEncoder(struct Encoder *encoder, uint8_t pinA, uint8_t pinB, uint8_t direction)
{
    encoder->pinA = pinA;
    encoder->pinB = pinB;
    encoder->pos = 0;
    encoder->direction = direction;

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    encoder->pinAPrevious = digitalRead(pinA);
    encoder->pinBPrevious = digitalRead(pinB);
}

/*******************************************************************************
   Checks if the encoder has moved by comparing the previous state of the pins
   with the current state. If they are different, we know there is movement.
   In the event of movement we update the current state of our pins.

   Parameters:
    encoder - Pointer to the encoder we will be checking for motion

   Return Value:
    encoderMotion - Returns the 0 if the encoder has not moved
                                1 for forward motion
                               -1 for reverse motion

 ******************************************************************************/
int8_t updateEncoder(struct Encoder *encoder)
{
    int8_t encoderMotion = 0;
    int pinACurrent = digitalRead(encoder->pinA);
    int pinBCurrent = digitalRead(encoder->pinB);

    // has the encoder moved at all?
    if (encoder->pinAPrevious != pinACurrent)
    {
        // Since it has moved, we must determine if the encoder has moved forwards or backwards
        encoderMotion = (encoder->pinAPrevious == encoder->pinBPrevious) ? -1 : 1;

        // If we are in reverse mode, flip the direction of the encoder motion
        if (encoder->direction == REVERSE)
            encoderMotion = -encoderMotion;
    }
    encoder->pinAPrevious = pinACurrent;
    encoder->pinBPrevious = pinBCurrent;

    return encoderMotion;
}

/*******************************************************************************
   Sends a message to Eos informing them of a wheel movement.

   Parameters:
    type - the type of wheel that's moving (i.e. pan or tilt)
    ticks - the direction and intensity of the movement

   Return Value: void

 ******************************************************************************/
void sendWheelMove(WHEEL_TYPE type, float ticks)
{
    String wheelMsg("/eos/active/wheel");

    if (digitalRead(SHIFT_BTN) == LOW)
        wheelMsg.concat("/fine");
    else
        wheelMsg.concat("/coarse");

    OSCMessage wheelUpdate(wheelMsg.c_str());
    wheelUpdate.add(ticks);
    SLIPSerial.beginPacket();
    wheelUpdate.send(SLIPSerial);
    SLIPSerial.endPacket();
}

/*******************************************************************************
   Sends a message to Eos informing them of a key press.

   Parameters:
    down - whether a key has been pushed down (true) or released (false)
    key - the key that has moved

   Return Value: void

 ******************************************************************************/
void sendKeyPress(bool down, String key)
{
    key = "/eos/key/" + key;
    OSCMessage keyMsg(key.c_str());

    if (down)
        keyMsg.add(EDGE_DOWN);
    else
        keyMsg.add(EDGE_UP);

    SLIPSerial.beginPacket();
    keyMsg.send(SLIPSerial);
    SLIPSerial.endPacket();
}

/*******************************************************************************
   Sends a message to Eos informing them of a attribute key press.

   Parameters:
    down - whether a key has been pushed down (true) or released (false)
    key - the key that has moved

   Return Value: void

 ******************************************************************************/
void sendParamPress(String key)
{
    String key1 = "/eos/cmd"; // + key;
    String keyMsg(key1.c_str());
    key = key;

    if (key == "ENC1")
    {
        keyMsg.concat("/"+enc1Wheel.type);
    }
    else
        // something has gone very wrong
        return;

    OSCMessage keyMsg1(keyMsg.c_str());

    //    keyMsg1.add(EDGE_DOWN);

    SLIPSerial.beginPacket();
    keyMsg1.send(SLIPSerial);
    SLIPSerial.endPacket();
}

/*******************************************************************************
   Checks the status of all the buttons relevant to Eos (i.e. Next & Last)

   NOTE: This does not check the shift key. The shift key is used in tandem with
   the encoder to determine coarse/fine mode and thus does not report to Eos
   directly.

   Parameters: none

   Return Value: void

 ******************************************************************************/
void checkButtons()
{
    static int enc1KeyState = HIGH;
    static int enc2KeyState = HIGH;
    static int enc3KeyState = HIGH;
    static int enc4KeyState = HIGH;

    static uint32_t debounceTime11 = 0;
    static uint32_t debounceTime12 = 0;
    static uint32_t debounceTime13 = 0;
    static uint32_t debounceTime14 = 0;

    // Has the button state changed
    if (digitalRead(ENC1_BTN) != enc1KeyState)
    {
        enc1KeyState = digitalRead(ENC1_BTN);

        if (enc1KeyState == HIGH)
        {
            debounceTime11 = millis();
            ENC1_SCALE = encNorm; // added for encoder fine/coarse
        }
        else
        {
            debounceTime11 = 0;
            ENC1_SCALE = encCoarse; // added for encoder fine/coarse
        }
    }

    if (debounceTime11 > 0 && (millis() - debounceTime11 > 10))
    {
        // ... set the time stamp to 0 to say we have finished debouncing
        debounceTime11 = 0;
        sendParamPress("ENC1");
    }

    if (digitalRead(ENC2_BTN) != enc2KeyState)
    {
        enc2KeyState = digitalRead(ENC2_BTN);

        if (enc2KeyState == HIGH)
        {
            debounceTime12 = millis();
            ENC2_SCALE = encNorm; // added for encoder fine/coarse
        }
        else
        {
            debounceTime12 = 0;
            ENC2_SCALE = encCoarse; // added for encoder fine/coarse
        }
    }

    if (debounceTime12 > 0 && (millis() - debounceTime12 > 10))
    {
        // ... set the time stamp to 0 to say we have finished debouncing
        debounceTime12 = 0;
        sendParamPress("ENC2");
    }

    if (digitalRead(ENC3_BTN) != enc3KeyState)
    {
        enc3KeyState = digitalRead(ENC3_BTN);

        if (enc3KeyState == HIGH)
        {
            debounceTime13 = millis();
            ENC3_SCALE = encNorm; // added for encoder fine/coarse
        }
        else
        {
            debounceTime13 = 0;
            ENC3_SCALE = encCoarse; // added for encoder fine/coarse
        }
    }

    if (debounceTime13 > 0 && (millis() - debounceTime13 > 10))
    {
        // ... set the time stamp to 0 to say we have finished debouncing
        debounceTime13 = 0;
        sendParamPress("ENC3");
    }

    if (digitalRead(ENC4_BTN) != enc4KeyState)
    {
        enc4KeyState = digitalRead(ENC4_BTN);

        if (enc4KeyState == HIGH)
        {
            debounceTime14 = millis();
            ENC4_SCALE = encNorm; // added for encoder fine/coarse
        }
        else
        {
            debounceTime14 = 0;
            ENC4_SCALE = encCoarse; // added for encoder fine/coarse
        }
    }

    if (debounceTime14 > 0 && (millis() - debounceTime14 > 10))
    {
        // ... set the time stamp to 0 to say we have finished debouncing
        debounceTime14 = 0;
        sendParamPress("ENC4");
    }
}

/*******************************************************************************
   Here we setup our encoder, lcd, and various input devices. We also prepare
   to communicate OSC with Eos by setting up SLIPSerial. Once we are done with
   setup() we pass control over to loop() and never call setup() again.

   NOTE: This function is the entry function. This is where control over the
   Arduino is passed to us (the end user).

   Parameters: none

   Return Value: void

 ******************************************************************************/
void setup()
{
    SLIPSerial.begin(115200);
    // This is a hack around an Arduino bug. It was taken from the OSC library
    // examples
    //#ifdef BOARD_HAS_USB_SERIAL
    //    while (!SerialUSB);
    //#else
    //    while (!Serial);
    //#endif

    // This is necessary for reconnecting a device because it needs some time
    // for the serial port to open, but meanwhile the handshake message was
    // sent from Eos
    SLIPSerial.beginPacket();
    SLIPSerial.write((const uint8_t *)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
    SLIPSerial.endPacket();

    // Let Eos know we want updates on some things
    //issueSubscribes();

    //initEncoder(&enc1Wheel, ENC1_CLK, ENC1_DT, ENC1_DIR);
    //initEncoder(&enc2Wheel, ENC2_CLK, ENC2_DT, ENC2_DIR);
    //initEncoder(&enc4Wheel, ENC4_CLK, ENC4_DT, ENC4_DIR);
    //initEncoder(&enc3Wheel, ENC3_CLK, ENC3_DT, ENC3_DIR);

    display.begin();

    pinMode(SHIFT_BTN, INPUT_PULLUP);

    pinMode(ENC1_BTN, INPUT_PULLUP);
    pinMode(ENC2_BTN, INPUT_PULLUP);
    pinMode(ENC3_BTN, INPUT_PULLUP);
    pinMode(ENC4_BTN, INPUT_PULLUP);

    displayStatus();
}

/*******************************************************************************
   Here we service, monitor, and otherwise control all our peripheral devices.
   First, we retrieve the status of our encoders and buttons and update Eos.
   Next, we check if there are any OSC messages for us.
   Finally, we update our display (if an update is necessary)

   NOTE: This function is our main loop and thus this function will be called
   repeatedly forever

   Parameters: none

   Return Value: void

 ******************************************************************************/
void loop()
{
    static String curMsg;
    int size;
    // get the updated state of each encoder
    int32_t enc1Motion = updateEncoder(&enc1Wheel);
    int32_t enc2Motion = updateEncoder(&enc2Wheel);
    int32_t enc3Motion = updateEncoder(&enc3Wheel);
    int32_t enc4Motion = updateEncoder(&enc4Wheel);

    // Scale the result by a scaling factor
    enc1Motion *= ENC1_SCALE;
    enc2Motion *= ENC2_SCALE;
    enc3Motion *= ENC3_SCALE;
    enc4Motion *= ENC4_SCALE;

    // check for next/last updates
    checkButtons();

    // now update our wheels
    if (enc1Motion != 0)
        sendWheelMove(ENC1, enc1Motion);

    if (enc2Motion != 0)
        sendWheelMove(ENC2, enc2Motion);

    if (enc3Motion != 0)
        sendWheelMove(ENC3, enc3Motion);

    if (enc4Motion != 0)
        sendWheelMove(ENC4, enc4Motion);

    // check to see if any OSC commands have come from Eos
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
        lastMessageRxTime = millis();
        // We only care about the ping if we haven't heard recently
        // Clear flag when we get any traffic
        timeoutPingSent = false;
        curMsg = String();
    }

    // if there is the possibility of a timeout
    if (lastMessageRxTime > 0)
    {
        unsigned long diff = millis() - lastMessageRxTime;
        // We first check if it's been too long and we need to time out
        if (diff > TIMEOUT_AFTER_IDLE_INTERVAL)
        {
            connectedToEos = false;
            lastMessageRxTime = 0;
            updateDisplay = true;
            timeoutPingSent = false;
        }

        // It could be the console is sitting idle. Send a ping once to
        //  double check that it's still there, but only once after 5s have passed
        if (!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL)
        {
            OSCMessage ping("/eos/ping");
            ping.add("4Enc_hello"); // This way we know who is sending the ping
            SLIPSerial.beginPacket();
            ping.send(SLIPSerial);
            SLIPSerial.endPacket();
            timeoutPingSent = true;
        }
    }

    // update the display if needed
    if (updateDisplay)
        displayStatus();
}
