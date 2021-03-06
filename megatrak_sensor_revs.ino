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
#define EEPROM_DIV1_ADDRESS 2
#define EEPROM_DIV2_ADDRESS 3
#define EEPROM_DIV3_ADDRESS 4
#define EEPROM_MOD1_ADDRESS 5
#define EEPROM_MOD2_ADDRESS 6
#define EEPROM_MOD3_ADDRESS 7
#define EEPROM_PREC1_ADDRESS 8
#define EEPROM_PREC2_ADDRESS 9
#define EEPROM_PREC3_ADDRESS 10
#define EEPROM_DIAM1_ADDRESS 11
#define EEPROM_DIAM2_ADDRESS 12
#define EEPROM_DIAM3_ADDRESS 13
#define EEPROM_SEND_INTERVAL_ADDRESS 14
#define EEPROM_REVNR_EN_ADDRESS 15

bool DEBUG = 0;

uint8_t SERVER_ADDRESS = EEPROM.read(EEPROM_SERVER_ADDRESS);
uint8_t NODE_ADDRESS = EEPROM.read(EEPROM_NODE_ADDRESS);

SerialCommand cmd;

RH_RF69 rf69(15, 14);
RHDatagram manager(rf69, NODE_ADDRESS);
uint8_t msg[RH_RF69_MAX_MESSAGE_LEN];


uint8_t SEND_INTERVAL = EEPROM.read(EEPROM_SEND_INTERVAL_ADDRESS);

typedef union {
	struct	{
		unsigned char bit1 : 1;
		unsigned char bit2 : 1;
		unsigned char bit3 : 1;
		unsigned char bit4 : 1;
		unsigned char bit5 : 1;
		unsigned char bit6 : 1;
		unsigned char bit7 : 1;
		unsigned char bit8 : 1;
	}bits;
	unsigned char revnren;
}RevEn;

RevEn reven;

unsigned long time = 0;

const uint8_t revPin1 = 10;
unsigned long revTimeOld1 = 0;
unsigned long revTimeNew1 = 0;
unsigned long revTimeDiff1 = 0;
uint16_t revs1 = 0;
uint8_t divider1 = EEPROM.read(EEPROM_DIV1_ADDRESS); //Divider for one revision
uint16_t precision1 = pow(10, EEPROM.read(EEPROM_PREC1_ADDRESS)); //precision used for calculating revs per min
uint8_t revMode1 = EEPROM.read(EEPROM_MOD1_ADDRESS); // Mode 0 = umin, 1 = kmh
uint8_t diam1 = EEPROM.read(EEPROM_DIAM1_ADDRESS);
//float speed1 = 0.0;
union sp{ // see http://forum.arduino.cc/index.php?topic=246654.msg1763405#msg1763405
	float f = 0.0;
	unsigned char b[4];
} speed1;

const uint8_t revPin2 = 3;
unsigned long revTimeOld2 = 0;
unsigned long revTimeNew2 = 0;
unsigned long revTimeDiff2 = 0;
uint16_t revs2 = 0;
uint8_t divider2 = EEPROM.read(EEPROM_DIV2_ADDRESS); //Divider for one revision
uint16_t precision2 = pow(10, EEPROM.read(EEPROM_PREC2_ADDRESS)); //precision used for calculating revs per min
uint8_t revMode2 = EEPROM.read(EEPROM_MOD2_ADDRESS); // Mode 0 = umin, 1 = kmh
uint8_t diam2 = EEPROM.read(EEPROM_DIAM2_ADDRESS);
//float speed2 = 0.0;
union sp speed2; //https://forum.arduino.cc/index.php?topic=462747.msg3176384#msg3176384

const uint8_t revPin3 = 4;
unsigned long revTimeOld3 = 0;
unsigned long revTimeNew3 = 0;
unsigned long revTimeDiff3 = 0;
uint16_t revs3 = 0;
uint8_t divider3 = EEPROM.read(EEPROM_DIV3_ADDRESS); //Divider for one revision
uint16_t precision3 = pow(10, EEPROM.read(EEPROM_PREC3_ADDRESS)); //precision used for calculating revs per min
uint8_t revMode3 = EEPROM.read(EEPROM_MOD3_ADDRESS); // Mode 0 = umin, 1 = kmh
uint8_t diam3 = EEPROM.read(EEPROM_DIAM3_ADDRESS);
union sp speed3; //https://forum.arduino.cc/index.php?topic=462747.msg3176384#msg3176384


void setup() {
	Serial.begin(57600);
	while (!Serial && (millis() <= 8000));
	Serial.print("rfm69 sender startup nr: ");
	Serial.println(NODE_ADDRESS);

	reven.revnren = EEPROM.read(EEPROM_REVNR_EN_ADDRESS);

	//###################################################
	cmd.setDefaultHandler(sc_unrecognized);
	cmd.addCommand("sna", sc_setNodeAddress); //set node address
	cmd.addCommand("ssa", sc_setServerAddress); //set server address
	cmd.addCommand("cfg", sc_printCfg); //print the current config
	cmd.addCommand("scn", sc_sndCmdNode); //send a command to a node over rf
	cmd.addCommand("srd", sc_setRevDivider);
	cmd.addCommand("srp", sc_setRevPrecision);
	cmd.addCommand("srm", sc_setRevMode);
	cmd.addCommand("srdia", sc_setRevDiameter);
	cmd.addCommand("sni", sc_setInterval);
	cmd.addCommand("snd", sc_setDebug);
	cmd.addCommand("sre", sc_setRevEnable);
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

	pinMode(revPin1, INPUT);
	attachInterrupt(revPin1, countRevs1, FALLING);
	pinMode(revPin2, INPUT);
	attachInterrupt(revPin2, countRevs2, FALLING);
	pinMode(revPin3, INPUT);
	attachInterrupt(revPin3, countRevs3, FALLING);
}

uint8_t from;
float count = 0.0;

uint8_t data[6];

void loop() {
	cmd.readSerial();
	chkMsg();
	if(DEBUG){
		if (millis() - time > 100*SEND_INTERVAL) {
			sendOKMsg();
			time = millis();
		}
	} else {
	if (millis() - time > 100*SEND_INTERVAL) {
		if (reven.bits.bit1) { sendSpeed(1); };
		if (reven.bits.bit2) { sendSpeed(2); };
		if (reven.bits.bit3) { sendSpeed(3); };
		
		time = millis();
		//count++;
		}
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

void sendMsg(uint8_t* mes, uint8_t rcvr, uint8_t len) {
	//Serial.print(len);
	//Serial.print(" ");
	//for (int i = 0; i < len; i++) {
	//	Serial.print(mes[i], DEC);
	//	Serial.print(" ");
	//}
	//Serial.println("");

	// Send a reply back to the originator client
	if (!manager.sendto((uint8_t*)mes, len, rcvr)) {
		Serial.println("sendto failed");
	}
	manager.waitPacketSent(); //wichtig!!!
}

void sendOKMsg() {
	char* ok = "iok";
	sendMsg((uint8_t*)ok,SERVER_ADDRESS,strlen(ok));
}

void sendSpeed(uint8_t revNr) {
	data[1] = revNr;
	switch (revNr) {
	case 1:
		revMode1 ? data[0] = 's' : data[0] = 'u';
		data[2] = speed1.b[0];
		data[3] = speed1.b[1];
		data[4] = speed1.b[2];
		data[5] = speed1.b[3];
		sendMsg(data, SERVER_ADDRESS, sizeof(data));
		speed1.f = 0.0;
		break;
	case 2:
		revMode2 ? data[0] = 's' : data[0] = 'u';
		data[2] = speed2.b[0];
		data[3] = speed2.b[1];
		data[4] = speed2.b[2];
		data[5] = speed2.b[3];
		sendMsg(data, SERVER_ADDRESS, sizeof(data));
		speed2.f = 0.0;
		break;
	case 3:
		revMode3 ? data[0] = 's' : data[0] = 'u';
		data[2] = speed3.b[0];
		data[3] = speed3.b[1];
		data[4] = speed3.b[2];
		data[5] = speed3.b[3];
		sendMsg(data, SERVER_ADDRESS, sizeof(data));
		speed3.f = 0.0;
		break;
	}
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
		else if (msg[1] == 'r') {
			if (msg[2] == 'p') {
				// set precision
				setEEPROMRevPrecision(msg[3]-'0', msg[4]-'0');  // -'0' ist trick um aus ascii-zahl einen int zu machen
				sendOKMsg();
				
			}
			else if (msg[2] == 'e') {
				// enable revNr
				setEEPROMRevEnable(msg[3] - '0', msg[4] - '0');  // -'0' ist trick um aus ascii-zahl einen int zu machen
				sendOKMsg();

			}else if (msg[2] == 'm') {
				// set revmode
				setEEPROMRevMode(msg[3] - '0', msg[4] - '0');  // -'0' ist trick um aus ascii-zahl einen int zu machen
				sendOKMsg();

			}else if (msg[2] == 'd') {
				if (msg[3] == 'i' && msg[4] == 'a') {
					uint8_t revNr = msg[5] - '0';
					//set diameter
					uint8_t val[3] = {};
					val[0] = msg[6];
					val[1] = msg[7];
					val[2] = msg[8];

					sendOKMsg();
					setEEPROMRevLength(revNr,atoi((char*)val));
					
				}
				else {
					//set divider
					uint8_t revNr = msg[3] - '0';
					//set diameter
					uint8_t val[3] = {};
					val[0] = msg[4];
					val[1] = msg[5];
					val[2] = msg[6];

					setEEPROMRevDivider(revNr, atoi((char*)val));  // -'0' ist trick um aus ascii-zahl einen int zu machen
					sendOKMsg();
				}
			}
		}
		else if (msg[1] == 'n' && msg[2] == 'i') {
			//set send interval
			uint8_t val[3];
			val[0] = msg[3];//todo
			val[1] = msg[4];
			val[2] = msg[5];
			Serial.println(atoi((char*)val));
			if (atoi((char*)val) < 255) {
				setEEPROMSendInterval(atoi((char*)val));  // -'0' ist trick um aus ascii-zahl einen int zu machen
				sendOKMsg();
			}
		}
	}
}
//###################################################
void sc_unrecognized(const char *command) {
	Serial.println("command not understood! try something like...");
	Serial.println("sna - set node adress: sna [1-255]");
	Serial.println("ssa - set server adress: ssa [1-255]");
	Serial.println("cfg - show current config");
	Serial.println("scn - send command to node: scn [rcv adress] [cmd] - scn 2 ssa3");
	Serial.println("srd - set divider: srd [1|2|3] [1-255]");
	Serial.println("srp - set precision: srd [0|1|2|3] - 1 10 100 1000" );
	Serial.println("srm - set mode: srm [1|2|3] [0|1] - 0 umin 1kmh");
	Serial.println("srdia - set wheel diameter in cm: srdia [1-255]");
	Serial.println("sni - set send interval (*100ms): sni [1-255]");
	Serial.println("snd - set debug mode: snd [0|1]");
	Serial.println("sre - en|disable revnr: sre [1|2|3] [0|1]");
}

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

void sc_setDebug() {
	bool d;
	char *arg;
	arg = cmd.next();
	if (arg != NULL) {
		d = (bool)atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("snd - debug value is missing! aborting...");
		return;
	}
	if (d == 1) {
		Serial.println("debug on...");
		DEBUG = d;
	} else if (d == 0){
		Serial.println("debug off...");
		DEBUG = d;
	}
	else {
		Serial.println("wrong value for debug only 0|1");
	}
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

	Serial.print("divider: ");
	Serial.print(divider1);
	Serial.print("|");
	Serial.print(divider2);
	Serial.print("|");
	Serial.println(divider3);

	Serial.print("precision: ");
	Serial.print(precision1);
	Serial.print("|");
	Serial.print(precision2);
	Serial.print("|");
	Serial.println(precision3);

	Serial.print("revMode: ");
	Serial.print(revMode1);
	Serial.print("|");
	Serial.print(revMode2);
	Serial.print("|");
	Serial.println(revMode3);

	Serial.print("diameter: ");
	Serial.print(diam1);
	Serial.print("|");
	Serial.print(diam2);
	Serial.print("|");
	Serial.println(diam3);

	Serial.print("interval (*100ms): ");
	Serial.println(SEND_INTERVAL);

	Serial.print("revEn: ");
	Serial.print(reven.bits.bit8);
	Serial.print("|");
	Serial.print(reven.bits.bit7);
	Serial.print("|");
	Serial.print(reven.bits.bit6);
	Serial.print("|");
	Serial.print(reven.bits.bit5);
	Serial.print("|");
	Serial.print(reven.bits.bit4);
	Serial.print("|");
	Serial.print(reven.bits.bit3);
	Serial.print("|");
	Serial.print(reven.bits.bit2);
	Serial.print("|");
	Serial.println(reven.bits.bit1);

}

void sc_setRevDiameter() {
	uint8_t revNr;
	uint16_t len;
	char *arg;
	arg = cmd.next();
	if (arg != NULL) {
		revNr = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("srdia - revNr is missing! aborting...");
		return;
	}
	arg = cmd.next();
	if (arg != NULL) {
		len = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("srdia - diameter is missing! aborting...");
		return;
	}
	setEEPROMRevLength(revNr, len); 
}
void setEEPROMRevLength(uint8_t revNr, uint16_t dia) {
	uint16_t olen;
	Serial.print("setting revdiameter ");
	switch (revNr) {
	case 1:
		Serial.print(revNr);
		Serial.print(" from ");
		olen = (int)EEPROM.read(EEPROM_DIAM1_ADDRESS); //diameter in cm used for calculating kmh
		Serial.print(olen, DEC);
		Serial.print(" to ");
		Serial.println(dia);
		EEPROM.write(EEPROM_DIAM1_ADDRESS, dia);
		diam1 = dia;
		break;
	case 2:
		Serial.print(revNr);
		Serial.print(" from ");
		olen = (int)EEPROM.read(EEPROM_DIAM2_ADDRESS); //diameter in cm used for calculating kmh
		Serial.print(olen, DEC);
		Serial.print(" to ");
		Serial.println(dia);
		EEPROM.write(EEPROM_DIAM2_ADDRESS, dia);
		diam2 = dia;
		break;
	case 3:
		Serial.print(revNr);
		Serial.print(" from ");
		olen = (int)EEPROM.read(EEPROM_DIAM3_ADDRESS); //diameter in cm used for calculating kmh
		Serial.print(olen, DEC);
		Serial.print(" to ");
		Serial.println(dia);
		EEPROM.write(EEPROM_DIAM3_ADDRESS, dia);
		diam3 = dia;
		break;
	}
	Serial.println("done!");
}

void sc_setRevMode() {
	uint8_t revNr;
	uint8_t mod;
	char *arg;
	arg = cmd.next();
	if (arg != NULL) {
		revNr = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("srm - revNr is missing! aborting...");
		return;
	}
	arg = cmd.next();
	if (arg != NULL) {
		mod = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("srm - mode is missing! aborting...");
		return;
	}
	setEEPROMRevMode(revNr, mod);
}
void setEEPROMRevMode(uint8_t revNr, uint8_t rmode) {
	Serial.print("setting revmode ");
	switch (revNr) {
	case 1:
		Serial.print(revNr);
		Serial.print(" from ");
		Serial.print(EEPROM.read(EEPROM_MOD1_ADDRESS), DEC);
		Serial.print(" to ");
		Serial.println(rmode);
		EEPROM.write(EEPROM_MOD1_ADDRESS, rmode);
		revMode1 = rmode;
		break;
	case 2:
		Serial.print(revNr);
		Serial.print(" from ");
		Serial.print(EEPROM.read(EEPROM_MOD2_ADDRESS), DEC);
		Serial.print(" to ");
		Serial.println(rmode);
		EEPROM.write(EEPROM_MOD2_ADDRESS, rmode);
		revMode2 = rmode;
		break;
	case 3:
		Serial.print(revNr);
		Serial.print(" from ");
		Serial.print(EEPROM.read(EEPROM_MOD3_ADDRESS), DEC);
		Serial.print(" to ");
		Serial.println(rmode);
		EEPROM.write(EEPROM_MOD3_ADDRESS, rmode);
		revMode3 = rmode;
		break;
	}
	Serial.println("done!");
}
void sc_setRevPrecision() {
	uint8_t revNr;
	uint8_t prec;
	char *arg;
	arg = cmd.next();
	if (arg != NULL) {
		revNr = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("srp - revNr is missing! aborting...");
		return;
	}
	arg = cmd.next();
	if (arg != NULL) {
		prec = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("srp - precision is missing! aborting...");
		return;
	}
	setEEPROMRevPrecision(revNr, prec);
}
void setEEPROMRevPrecision(uint8_t revNr, uint8_t prec) {
	Serial.print("setting precision ");
	switch (revNr) {
	case 1:
		Serial.print(revNr);
		Serial.print(" from ");
		Serial.print(EEPROM.read(EEPROM_PREC1_ADDRESS), DEC);
		Serial.print(" to ");
		if (prec < 0) prec = 0;
		Serial.println(prec);
		EEPROM.write(EEPROM_PREC1_ADDRESS, prec);
		precision1 = pow(10, prec);
		break;
	case 2:
		Serial.print(revNr);
		Serial.print(" from ");
		Serial.print(EEPROM.read(EEPROM_PREC2_ADDRESS), DEC);
		Serial.print(" to ");
		if (prec < 0) prec = 0;
		Serial.println(prec);
		EEPROM.write(EEPROM_PREC2_ADDRESS, prec);
		precision2 = pow(10, prec);
		break;
	case 3:
		Serial.print(revNr);
		Serial.print(" from ");
		Serial.print(EEPROM.read(EEPROM_PREC3_ADDRESS), DEC);
		Serial.print(" to ");
		if (prec < 0) prec = 0;
		Serial.println(prec);
		EEPROM.write(EEPROM_PREC3_ADDRESS, prec);
		precision3 = pow(10, prec);
		break;
	}
	Serial.println("done!");
}

void sc_setRevDivider() {
	uint8_t revNr;
	uint8_t div;
	char *arg;
	arg = cmd.next();
	if (arg != NULL) {
		revNr = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("srd - revNr is missing! aborting...");
		return;
	}
	arg = cmd.next();
	if (arg != NULL) {
		div = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("srd - divider is missing! aborting...");
		return;
	}
	setEEPROMRevDivider(revNr, div);
}
void setEEPROMRevDivider(uint8_t revNr, uint8_t div) {
	Serial.print("setting divider ");
	switch (revNr) {
	case 1:
		Serial.print(revNr);
		Serial.print(" from ");
		Serial.print(EEPROM.read(EEPROM_DIV1_ADDRESS), DEC);
		Serial.print(" to ");
		if (div < 1) { div = 1; }
		Serial.println(div);
		EEPROM.write(EEPROM_DIV1_ADDRESS, div);
		divider1 = div;
		break;
	case 2:
		Serial.print(revNr);
		Serial.print(" from ");
		Serial.print(EEPROM.read(EEPROM_DIV2_ADDRESS), DEC);
		Serial.print(" to ");
		if (div < 1) { div = 1; }
		Serial.println(div);
		EEPROM.write(EEPROM_DIV2_ADDRESS, div);
		divider2 = div;
		break;
	case 3:
		Serial.print(revNr);
		Serial.print(" from ");
		Serial.print(EEPROM.read(EEPROM_DIV3_ADDRESS), DEC);
		Serial.print(" to ");
		if (div < 1) { div = 1; }
		Serial.println(div);
		EEPROM.write(EEPROM_DIV3_ADDRESS, div);
		divider3 = div;
		break;
	}
	Serial.println("done!");
}

void sc_setInterval() {
	char *arg;
	uint16_t iv;
	arg = cmd.next();
	if (arg != NULL) {
		iv = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("si - interval is missing! aborting...");
		return;
	}
	setEEPROMSendInterval(iv);
}
//todo
void setEEPROMSendInterval(uint8_t iv) {
	Serial.print("setting interval from ");
	Serial.print(EEPROM.read(EEPROM_SEND_INTERVAL_ADDRESS)*100, DEC);
	Serial.print(" to ");
	Serial.println(iv * 100);
	if (iv < 1) { iv = 1; }
	SEND_INTERVAL = iv;
	EEPROM.write(EEPROM_SEND_INTERVAL_ADDRESS, iv);
	Serial.println("...done");

}

void sc_setRevEnable() {
	char *arg;
	uint16_t rNr;
	uint16_t val;
	arg = cmd.next();
	if (arg != NULL) {
		rNr = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("sre - revNr is missing! aborting...");
		return;
	}
	arg = cmd.next();
	if (arg != NULL) {
		val = atoi(arg);    // Converts a char string to an integer
	}
	else {
		Serial.println("sre - val is missing! aborting...");
		return;
	}
	setEEPROMRevEnable(rNr,val);
}
void setEEPROMRevEnable(uint8_t revNr, bool val) {
	Serial.println("setting revnr ");
	Serial.print(revNr);
	Serial.print(" to ");
	Serial.println(val);

	switch (revNr) {
	case 1:
		reven.bits.bit1 = val;
		break;
	case 2:
		reven.bits.bit2 = val;
		break;
	case 3:
		reven.bits.bit3 = val;
		break;
	case 4:
		reven.bits.bit4 = val;
		break;
	case 5:
		reven.bits.bit5 = val;
		break;
	case 6:
		reven.bits.bit6 = val;
		break;
	case 7:
		reven.bits.bit7 = val;
		break;
	case 8:
		reven.bits.bit8 = val;
		break;
	}
	EEPROM.write(EEPROM_REVNR_EN_ADDRESS, reven.revnren);
	Serial.println("...done");
}

void countRevs1() {
	revTimeNew1 = millis();
	revTimeDiff1 = revTimeNew1 - revTimeOld1;

	float s = 0.0;
	if (revMode1) {
		revs1 = 1000 * 60 / revTimeDiff1 / divider1;
		s = revs1 * diam1 *3.142 / 100 * 60; //m pro h
		if (s > 1000) {
			speed1.f = s / 1000;
		}
		else {
			speed1.f = (int)s / precision1* precision1;
		}
	}
	else {
		revs1 = 1000 * 60 / revTimeDiff1 / divider1 / precision1 * precision1;
		speed1.f = revs1;
	}
	revTimeOld1 = revTimeNew1;
}

void countRevs2() {
	revTimeNew2 = millis();
	revTimeDiff2 = revTimeNew2 - revTimeOld2;
	float s = 0.0;
	if (revMode2) {
		revs2 = 1000 * 60 / revTimeDiff2 / divider2;
		s = revs2 * diam2 *3.142 / 100 * 60; //m pro h
		if (s > 1000) {
			speed2.f = s / 1000;
		}
		else {
			speed2.f = (int)s / precision2* precision2;
		}
	}
	else {
		revs2 = 1000 * 60 / revTimeDiff2 / divider2 / precision2 * precision2;
		speed2.f = revs2;
	}
	revTimeOld2 = revTimeNew2;
}

void countRevs3() {
	revTimeNew3 = millis();
	revTimeDiff3 = revTimeNew3 - revTimeOld3;
	float s = 0.0;
	if (revMode3) {
		revs3 = 1000 * 60 / revTimeDiff3 / divider3;
		s = revs3 * diam3 *3.142 / 100 * 60; //m pro h
		if (s > 1000) {
			speed3.f = s / 1000;
		}
		else {
			speed3.f = (int)s / precision3* precision3;
		}
	}
	else {
		revs3 = 1000 * 60 / revTimeDiff3 / divider3 / precision3 * precision3;
		speed3.f = revs3;
	}
	revTimeOld3 = revTimeNew3;
}
