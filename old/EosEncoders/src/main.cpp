//Included Libraries
#include "Arduino.h"
#include <OSCMessage.h>
#include <eOS.h>

#ifdef BOARD_HAS_USB_SERIAL
	#include <SLIPEncodedUSBSerial.h>
	SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
#else
	#include <SLIPEncodedSerial.h>
	SLIPEncodedSerial SLIPSerial(Serial);
#endif


//Enable eOS 'eos' constructor
EOS eos;

//Global strings for beginning communication with Eos
const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";
const String PING_QUERY = "encoder_hello";

// Change these values to alter how long we wait before sending an OSC ping
// to see if Eos is still there, and then finally how long before we
// disconnect and show the splash screen
// Values are in milliseconds
#define PING_AFTER_IDLE_INTERVAL	2500
#define TIMEOUT_AFTER_IDLE_INTERVAL	5000

//Global Variables
int idx1 = 1;
int idx2 = 2;
int idx3 = 3;
int idx4 = 4;

bool connectedToEos = false;
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;



//Wheel Objects
Wheel wheel1(1, 2, FORWARD);
Wheel wheel2(3, 4, FORWARD);
Wheel wheel3(5, 6, FORWARD);
Wheel wheel4(7, 8, FORWARD);

//Local Functions

//@brief Init the console, gives back a handshake reply and send the filters.
void initEOS() {
	SLIPSerial.print(HANDSHAKE_REPLY);
	filter("/eos/out/ping");
	connectedToEos = true;
}

void wheelidxup(){
	idx1 = idx1 + 4;
	idx2 = idx2 + 4;
	idx3 = idx3 + 4;
	idx4 = idx4 + 4;
	wheel1.index(idx1);
	wheel2.index(idx2);
	wheel3.index(idx3);
	wheel4.index(idx4);
}

void wheelidxdown(){
	idx1 = idx1 - 4;
	idx2 = idx2 - 4;
	idx3 = idx3 - 4;
	idx4 = idx4 - 4;
	wheel1.index(idx1);
	wheel2.index(idx2);
	wheel3.index(idx3);
	wheel4.index(idx4);
}

void wheelupdate(){
	wheel1.update();
	wheel2.update();
	wheel3.update();
	wheel4.update();
}

void parseOSCMessage(String& msg) {
	if (msg.indexOf(HANDSHAKE_QUERY) != -1) { // Check to see if this is the handshake string
		initEOS();
		connectedToEos = true;
		return;
	}

	else{return;}
}


/**
 * @brief Here we setup our encoder, lcd, and various input devices. We also prepare
 * to communicate OSC with Eos by setting up SLIPSerial. Once we are done with
 * setup() we pass control over to loop() and never call setup() again.
 *
 * NOTE: This function is the entry function. This is where control over the
 * Arduino is passed to us (the end user).
 * 
 */
void setup()
{
	SLIPSerial.begin(115200);
	// This is a hack around an Arduino bug. It was taken from the OSC library examples
	#ifdef BOARD_HAS_USB_SERIAL
		#ifndef TEENSYDUINO
			while (!SerialUSB);
		#endif
	#else
		while (!Serial);
	#endif


	pinMode(13, OUTPUT);
	pinMode(11, INPUT_PULLUP);
	pinMode(12, INPUT_PULLUP);
	digitalWrite(13, HIGH);

	wheel1.index(idx1);
	wheel2.index(idx2);
	wheel3.index(idx3);
	wheel4.index(idx4);

	initEOS();
}


/**
 * @brief Here we service, monitor, and otherwise control all our peripheral devices.
 * First, we retrieve the status of our encoders and buttons and update Eos.
 * Next, we check if there are any OSC messages for us.
 * Finally, we update our display (if an update is necessary)
 *
 * NOTE: This function is our main loop and thus this function will be called
 * repeatedly forever
 * 
 */
void loop()
{
	static String curMsg;
	int size;

	size = SLIPSerial.available();
	if (size > 0) {
		// Fill the msg with all of the available bytes
		while (size--) curMsg += (char)(SLIPSerial.read());
		}
	if (SLIPSerial.endofPacket()) {
		parseOSCMessage(curMsg);
		lastMessageRxTime = millis();
		// We only care about the ping if we haven't heard recently
		// Clear flag when we get any traffic
		timeoutPingSent = false;
		curMsg = String();
	}

	if(digitalRead(11) == LOW){
		wheelidxdown();
		delay(200);
		return;
	}

	if(digitalRead(12) == LOW){
		wheelidxup();
		delay(200);
		return;
	}

	wheelupdate();

	if (lastMessageRxTime > 0) {
		unsigned long diff = millis() - lastMessageRxTime;
		//We first check if it's been too long and we need to time out
		if (diff > TIMEOUT_AFTER_IDLE_INTERVAL) {
			connectedToEos = false;
			lastMessageRxTime = 0;
			timeoutPingSent = false;
			}
		//It could be the console is sitting idle. Send a ping once to
		// double check that it's still there, but only once after 2.5s have passed
		if (!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL) {
			ping(PING_QUERY);
			timeoutPingSent = true;
			}

	}
}