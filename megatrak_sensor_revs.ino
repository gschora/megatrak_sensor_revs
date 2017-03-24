#include <RHDatagram.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <EEPROM.h>
#include <SerialCommand.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

#define EEPROM_NODE_ADDRESS 0
#define EEPROM_SERVER_ADDRESS 1
uint8_t SERVER_ADDRESS = EEPROM.read(EEPROM_SERVER_ADDRESS);
uint8_t NODE_ADDRESS = EEPROM.read(EEPROM_NODE_ADDRESS);

SerialCommand cmd;

RH_RF69 rf69(15, 14);
RHDatagram manager(rf69, NODE_ADDRESS);
uint8_t msg[RH_RF69_MAX_MESSAGE_LEN];

#define SEND_INTERVALL 1000

unsigned long time = 0;

void setup() {
	Serial.begin(57600);
	while (!Serial && (millis() <= 8000));
	Serial.print("rfm69 sender startup nr: ");
	Serial.println(NODE_ADDRESS);

	//###################################################
	cmd.addCommand("sna", sc_setNodeAddress); //set node address
	cmd.addCommand("ssa", sc_setServerAddress); //set server address
	cmd.addCommand("cfg", sc_printCfg); //print the current config
	cmd.addCommand("scn", sc_sndCmdNode); //send a command to a node over rf
	//###################################################

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

uint8_t from;
uint16_t count = 0;

uint8_t data[3];

void loop() {
	cmd.readSerial();
	chkMsg();

	if (millis() - time > SEND_INTERVALL) {
		data[0] = 'u';
		data[1] = lowByte(count);
		data[2] = highByte(count);
		//Serial.println((char*)data);
		sendMsg((char*)data, SERVER_ADDRESS);
		//Serial.println(count);

		count++;
		time = millis();
	}
}

void chkMsg() {
	if (manager.available()) {
		uint8_t len = sizeof(msg);
		uint8_t from;
		memset(msg, 0, sizeof(msg)); //clearing the msg array IMPORTANT!!!!! otherwise old values from previous messages stay in there!!!

		if (manager.recvfrom(msg, &len, &from)) {
			if (from == SERVER_ADDRESS) {
				parseSrvCmd();
			}
			//Serial.print("got reply: ");
			//Serial.print(from, DEC);
			//Serial.print(": ");
			//Serial.println((char*)msg);
		}
		else {
			Serial.println("recv failed");
		}
	}
}

void sendMsg(char* data, uint8_t rcvr) {
	// Send a reply back to the originator client
	if (!manager.sendto((uint8_t*)data, strlen(data), rcvr)) {
		Serial.println("sendto failed");
	}
	manager.waitPacketSent(); //wichtig!!!
}

void sendOKMsg() {
	char* ok = "iok";
	sendMsg(ok,SERVER_ADDRESS);
}

void parseSrvCmd() {
	
	Serial.print("msg from server: ");
	Serial.println((char*)msg);
	//Serial.println(sizeof(msg));
	if (msg[0] == 's') {
		if (msg[1] == 'n' && msg[2] == 'a') {
			uint8_t addr[3] = {};
			addr[0] = msg[3];
			addr[1] = msg[4];
			addr[2] = msg[5];

			if (atoi((char*)addr) < 255) {
				sendOKMsg();
				setEEPROMNodeAddress(atoi((char*)addr));
			}
					
		}
	}

}
//###################################################
void setEEPROMNodeAddress(uint8_t address) {
	Serial.print("setting node address from ");
	Serial.print(EEPROM.read(EEPROM_NODE_ADDRESS), DEC);
	Serial.print(" to ");
	Serial.println(address);
	EEPROM.write(EEPROM_NODE_ADDRESS, address);
	NODE_ADDRESS = address;
	Serial.println("done!");
	WRITE_RESTART(0x5FA0004);
}
uint8_t getEEPROMNodeAddress() {
	return EEPROM.read(EEPROM_NODE_ADDRESS);
}
void sc_setNodeAddress() {
	uint8_t address;
	char *arg;
	arg = cmd.next();
	if (arg != NULL) {
		address = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("sna - address is missing! aborting...");
		return;
	}
	setEEPROMNodeAddress(address);
}

void setEEPROMServerAddress(uint8_t address) {
	Serial.print("setting server address from ");
	Serial.print(EEPROM.read(EEPROM_SERVER_ADDRESS), DEC);
	Serial.print(" to ");
	Serial.println(address);
	EEPROM.write(EEPROM_SERVER_ADDRESS, address);
	SERVER_ADDRESS = address;
	Serial.println("done!");
}

uint8_t getEEPROMServerAddress() {
	return EEPROM.read(EEPROM_SERVER_ADDRESS);
}

void sc_setServerAddress() {
	uint8_t address;
	char *arg;
	arg = cmd.next();
	if (arg != NULL) {
		address = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("ssa - address is missing! aborting...");
		return;
	}
	setEEPROMServerAddress(address);
}

void sc_sndCmdNode() {
	uint8_t node_adress;
	char *ncmd;
	uint8_t ncmdval;
	char *arg;

	arg = cmd.next();
	if (arg != NULL) {
		node_adress = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("scn - node address is missing! aborting...");
		return;
	}
	arg = cmd.next();
	if (arg != NULL) {
		ncmd = arg;
	}
	else {
		Serial.println("scn - node cmd is missing! aborting...");
		return;
	}
	arg = cmd.next();
	if (arg != NULL) {
		ncmdval = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("scn - cmd val is missing! aborting...");
		return;
	}

	Serial.print("rf command is to node: ");
	Serial.print(node_adress);
	Serial.print(" cmd: ");
	Serial.print(ncmd);
	Serial.print(" val: ");
	Serial.println(ncmdval);
}

void sc_printCfg() {
	Serial.print("node address: ");
	Serial.println(getEEPROMNodeAddress());
	Serial.print("server address: ");
	Serial.println(getEEPROMServerAddress());
}