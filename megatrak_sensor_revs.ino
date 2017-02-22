// rf69_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem
// configuration
// Tested on Moteino with RFM69 http://lowpowerlab.com/moteino/
// Tested on miniWireless with RFM69 www.anarduino.com/miniwireless
#include <RHDatagram.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

// Singleton instance of the radio driver
// RH_RF69 rf69(slave_select_pin, interrupt_pin);
RH_RF69 rf69(15, 14);

#define CLIENT_ADDRESS	3
#define SERVER_ADDRESS	1

RHDatagram manager(rf69, CLIENT_ADDRESS);

//Display
// Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();


void setup()
{
	Serial.begin(9600);
	while (!Serial && (millis() <= 8000));
	Serial.print("rfm69 sender startup nr: ");
	Serial.println(CLIENT_ADDRESS);

	if (!manager.init())
		Serial.println("init rfm69 failed");
	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	// No encryption
	if (!rf69.setFrequency(868.0))
		Serial.println("setFrequency failed");

	// If you are using a high power RF69, you *must* set a Tx power in the
	// range 14 to 20 like this:
	rf69.setTxPower(14);

}


uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint16_t count = 0;

uint8_t data[2];


void loop() {

	data[0] = lowByte(count);
	data[1] = highByte(count);
	count++;

	if (!manager.sendto(data, sizeof(data), SERVER_ADDRESS))
		Serial.println("sendto failed");

	// if (manager.available()) {
	// 	// Now wait for a reply

	// 	uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
	// 	uint8_t len = sizeof(buf);
	// 	uint8_t from;

	// 	// Should be a reply message for us now
	// 	if (manager.recvfrom(buf, &len, &from)) {
	// 		Serial.print("got reply: ");
	// 		Serial.print(from, DEC);
	// 		Serial.print(": ");
	// 		Serial.println((char*)buf);
	// 	} else {
	// 		Serial.println("recv failed");
	// 	}
	// }	else {
	// 	Serial.println("No reply, is rf69_server running?");
	// }
	delay(500);
}

