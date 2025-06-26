/*
	By Matiere&lumiere
	https://github.com/PM04290/

	Version 1.1 : initial
	Version 2.0 : packet release (config)
	Version 2.1 : DEFAULT pin user configurable
	
	Supported hardware radio:
	- SX1278 (eg. Ra-02)
	
*/
#pragma once

#include <Arduino.h>

#include "MLcompacket.h"

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#include <tinySPI.h>
#define SPIClass tinySPI
#define digitalPinToInterrupt(p)  ((p) == 8 ? 0 : NOT_AN_INTERRUPT)
#else
#include <SPI.h>
#endif

#define CRC_PRELOAD 0x5A

#if defined(__AVR_ATtiny84__)
	const uint8_t RL_NEW_MISO           = 0;
	const uint8_t RL_NEW_MOSI           = 0;
	const uint8_t RL_NEW_SCLK           = 0;
	const uint8_t RL_NEW_SS             = 0;
	SPIClass* RL_DEFAULT_SPI            = &SPI;
	const long RL_DEFAULT_SPI_FREQUENCY = 8E6; // not used
	const uint8_t RL_DEFAULT_SS_PIN     = 3;
	const uint8_t RL_DEFAULT_RESET_PIN  = 2;
	const uint8_t RL_DEFAULT_DINT_PIN   = 8;
#elif defined(__AVR_ATtiny814__) || defined(__AVR_ATtiny1614__)
	const uint8_t RL_NEW_MISO           = 0;
	const uint8_t RL_NEW_MOSI           = 0;
	const uint8_t RL_NEW_SCLK           = 0;
	const uint8_t RL_NEW_SS             = 0;
	SPIClass* RL_DEFAULT_SPI            = &SPI;
	const long RL_DEFAULT_SPI_FREQUENCY = 8E6; // not used
	const uint8_t RL_DEFAULT_SS_PIN     = PIN_PA4;
	const uint8_t RL_DEFAULT_RESET_PIN  = PIN_PA7;
	const uint8_t RL_DEFAULT_DINT_PIN   = PIN_PA6;
#elif defined(__AVR_ATtiny1616__) || defined(__AVR_ATtiny3216__) || defined(__AVR_ATtiny1617__) || defined(__AVR_ATtiny3217__)
	const uint8_t RL_NEW_MISO           = 0;
	const uint8_t RL_NEW_MOSI           = 0;
	const uint8_t RL_NEW_SCLK           = 0;
	const uint8_t RL_NEW_SS             = 0;
	SPIClass* RL_DEFAULT_SPI            = &SPI;
	const long RL_DEFAULT_SPI_FREQUENCY = 8E6; // not used
	const uint8_t RL_DEFAULT_SS_PIN     = PIN_PA4;
	const uint8_t RL_DEFAULT_RESET_PIN  = PIN_PA7;
	const uint8_t RL_DEFAULT_DINT_PIN   = PIN_PA6;
#elif defined(ESP32)
	#if defined(ARDUINO_LOLIN_S2_MINI)
		// defining NEW spi pin
		//#define RL_DEFAULT_SPI         not defined for specific SPI
		const uint8_t RL_NEW_MISO           = 9;
		const uint8_t RL_NEW_MOSI           = 11;
		const uint8_t RL_NEW_SCLK           = 7;
		const uint8_t RL_NEW_SS             = 5;
		SPIClass* RL_DEFAULT_SPI            = new SPIClass(HSPI);
		const long RL_DEFAULT_SPI_FREQUENCY = 8E6;
		const uint8_t RL_DEFAULT_SS_PIN     =  5;
		const uint8_t RL_DEFAULT_RESET_PIN  = 12;
		const uint8_t RL_DEFAULT_DINT_PIN   =  3;
	#elif defined(ARDUINO_LOLIN_S3_MINI) 
		const uint8_t RL_NEW_MISO           = 0;
		const uint8_t RL_NEW_MOSI           = 0;
		const uint8_t RL_NEW_SCLK           = 0;
		const uint8_t RL_NEW_SS             = 0;
		SPIClass* RL_DEFAULT_SPI            = &SPI;
		const long RL_DEFAULT_SPI_FREQUENCY = 8E6;
		const uint8_t RL_DEFAULT_SS_PIN     = 10;
		const uint8_t RL_DEFAULT_RESET_PIN  =  5;
		const uint8_t RL_DEFAULT_DINT_PIN   =  3;
	#elif defined(ARDUINO_ESP32_POE_ISO) || defined(ARDUINO_ESP32_POE)
		// defining NEW spi pin
		//#define RL_DEFAULT_SPI         not defined for specific SPI
		const uint8_t RL_NEW_MISO           = 15;
		const uint8_t RL_NEW_MOSI           = 2;
		const uint8_t RL_NEW_SCLK           = 14;
		const uint8_t RL_NEW_SS             = 5;
		SPIClass* RL_DEFAULT_SPI            = new SPIClass(HSPI);
		const long RL_DEFAULT_SPI_FREQUENCY = 8E6;
		const uint8_t RL_DEFAULT_SS_PIN     =  5;
		const uint8_t RL_DEFAULT_RESET_PIN  =  4;
		const uint8_t RL_DEFAULT_DINT_PIN   =  36;
	#elif defined(ARDUINO_WT32_ETH01)
		// defining NEW spi pin
		//#define RL_DEFAULT_SPI         not defined for specific SPI
		const uint8_t RL_NEW_MISO           = 15;
		const uint8_t RL_NEW_MOSI           = 2;
		const uint8_t RL_NEW_SCLK           = 14;
		const uint8_t RL_NEW_SS             = 12;
		SPIClass* RL_DEFAULT_SPI            = new SPIClass(HSPI);
		const long RL_DEFAULT_SPI_FREQUENCY = 8E6;
		const uint8_t RL_DEFAULT_SS_PIN     =  12;
		const uint8_t RL_DEFAULT_RESET_PIN  =  4;
		const uint8_t RL_DEFAULT_DINT_PIN   =  35;
	#elif defined(ARDUINO_TTGO_LoRa32_v21new)
		const uint8_t RL_NEW_MISO           = 0;
		const uint8_t RL_NEW_MOSI           = 0;
		const uint8_t RL_NEW_SCLK           = 0;
		const uint8_t RL_NEW_SS             = 0;
		SPIClass* RL_DEFAULT_SPI            = &SPI;
		const long RL_DEFAULT_SPI_FREQUENCY = 8E6;
		const uint8_t RL_DEFAULT_SS_PIN     = 18;
		const uint8_t RL_DEFAULT_RESET_PIN  = 23;
		const uint8_t RL_DEFAULT_DINT_PIN   = 26;
	#else
		const uint8_t RL_NEW_MISO           = 0;
		const uint8_t RL_NEW_MOSI           = 0;
		const uint8_t RL_NEW_SCLK           = 0;
		const uint8_t RL_NEW_SS             = 0;
		SPIClass* RL_DEFAULT_SPI            = &SPI;
		const long RL_DEFAULT_SPI_FREQUENCY = 8E6;
		const uint8_t RL_DEFAULT_SS_PIN     =  5;
		const uint8_t RL_DEFAULT_RESET_PIN  =  4;
		const uint8_t RL_DEFAULT_DINT_PIN   =  2;
	#endif
#elif defined(ESP32S2)
	const uint8_t RL_NEW_MISO           = 0;
	const uint8_t RL_NEW_MOSI           = 0;
	const uint8_t RL_NEW_SCLK           = 0;
	const uint8_t RL_NEW_SS             = 0;
	SPIClass* RL_DEFAULT_SPI            = &SPI;
	const long RL_DEFAULT_SPI_FREQUENCY = 8E6;
	const uint8_t RL_DEFAULT_SS_PIN     = 12;
	const uint8_t RL_DEFAULT_RESET_PIN  =  4;
	const uint8_t RL_DEFAULT_DINT_PIN   = 35;
#elif defined(__AVR_ATmega32U4__)
	const uint8_t RL_NEW_MISO           = 0;
	const uint8_t RL_NEW_MOSI           = 0;
	const uint8_t RL_NEW_SCLK           = 0;
	const uint8_t RL_NEW_SS             = 0;
	SPIClass* RL_DEFAULT_SPI            = &SPI;
	const long RL_DEFAULT_SPI_FREQUENCY = 8E6;
	const uint8_t RL_DEFAULT_SS_PIN     = 10;
	const uint8_t RL_DEFAULT_RESET_PIN  = 9;
	const uint8_t RL_DEFAULT_DINT_PIN   = 7;
#else
	const uint8_t RL_NEW_MISO           = 0;
	const uint8_t RL_NEW_MOSI           = 0;
	const uint8_t RL_NEW_SCLK           = 0;
	const uint8_t RL_NEW_SS             = 0;
	SPIClass* RL_DEFAULT_SPI            = &SPI;
	const long RL_DEFAULT_SPI_FREQUENCY = 8E6;
	const uint8_t RL_DEFAULT_SS_PIN     = 10;
	const uint8_t RL_DEFAULT_RESET_PIN  = 9;
	const uint8_t RL_DEFAULT_DINT_PIN   = 2;
#endif

//#define RL_SX1278 1
//#define RL_CC1101 1
//#define RL_NRF24 1

#include "helpers/MLcomhelper.hpp"

#if defined(ML_SX1278)
	#include "helpers/ML_SX1278.hpp"
	#define RADIOHELPER MLhelper_SX1278
#endif

#if defined(ML_MAX485_PIN_RX) && defined(ML_MAX485_PIN_TX) && defined(ML_MAX485_PIN_DE)
	#include "helpers/ML_MAX485.hpp"
	#define RADIOHELPER MLhelper_MAX485
#endif

RADIOHELPER RLhelper;

static rl_packet_t currentPacket;

void (*_onRxDone)(uint8_t,rl_packet_t*);
void (*_onTxDone)();
uint8_t _TXdone = 0;

#if (ESP8266 || ESP32)
    #define ISR_PREFIX ICACHE_RAM_ATTR
#else
    #define ISR_PREFIX
#endif

ISR_PREFIX void MLhelper_base::onDintRise()
{
  RLhelper.handleDintRise();
}

class iotCommClass
{
  public:
	iotCommClass() {
	}
#if defined(ML_MAX485_PIN_RX) && defined(ML_MAX485_PIN_TX) && defined(ML_MAX485_PIN_DE)
	bool begin(long baudrate, void(*callbackR)(uint8_t,rl_packet_t*), void(*callbackT)())
	{
	  _waitOnTx = true;
	  _onRxDone = callbackR;
	  _onTxDone = callbackT;
	  if (RLhelper.begin(baudrate))
	  {
		RLhelper.onInternalRxDone(onRxDone);
		RLhelper.onInternalTxDone(onTxDone);
		
		RLhelper.receiveMode();
		return true;
	  }
	  return false;
	}
#endif
	
#if defined(ML_SX1278)
	bool begin(long frequency, void(*callbackR)(uint8_t,rl_packet_t*), void(*callbackT)(), int TxLevel, uint8_t radioDistance)
	{
	  _waitOnTx = true;
	  _onRxDone = callbackR;
	  _onTxDone = callbackT;
	  if (RLhelper.begin(frequency))
	  {
		RLhelper.onInternalRxDone(onRxDone);
		RLhelper.onInternalTxDone(onTxDone);
		RLhelper.setTxPower(TxLevel);
		setRadioDistance(radioDistance);
		
		RLhelper.receiveMode();
		return true;
	  }
	  return false;
	}
#endif

	void end()
	{
		RLhelper.end();
	}

	static void onRxDone(int packetSize)
	{
	  if (packetSize == 0) return;
	  if ((size_t)packetSize > sizeof(currentPacket)) packetSize = sizeof(currentPacket);
	  byte* raw = (byte*)&currentPacket;
	  RLhelper.read(raw, packetSize);
	  uint8_t crc = CRC_PRELOAD;
	  for (uint8_t i = 0; i < packetSize-1; i++)
	  {
		  crc += raw[i];
	  }
	  if (crc != currentPacket.crc)
	  {
		  return;
	  }
	  if (_onRxDone)
	  {
		_onRxDone(packetSize, &currentPacket);
	  }
	}

	static void onTxDone()
	{
	  _TXdone = true;
	  RLhelper.receiveMode();
	  if (_onTxDone)
	  {
		_onTxDone();
	  }
	}

	int lqi() {
	  return RLhelper.lqi();
	}

	void sleep() {
	  return RLhelper.sleep();
	}

	void idle() {
	  return RLhelper.idle();
	}

	void setDint(uint8_t pin) {
		RLhelper.setDint(pin);
	}

	void setRst(uint8_t pin) {
		RLhelper.setRst(pin);
	}

	void setWaitOnTx(bool state)
	{
		_waitOnTx = state;
	}

	void setRadioDistance(uint8_t n) // 0 .. 3
	{
		if (n == 3) {
			RLhelper.setSpreadingFactor(11);   // 7 .. 12
			RLhelper.setSignalBandwidth(6);    // 0 .. 9
			RLhelper.setCodingRate4(4);        // 1 .. 4
			return;
		}
		if (n == 2) {
			RLhelper.setSpreadingFactor(10);   // 7 .. 12
			RLhelper.setSignalBandwidth(6);    // 0 .. 9
			RLhelper.setCodingRate4(3);        // 1 .. 4
			return;
		}
		if (n == 1) {
			RLhelper.setSpreadingFactor(9);   // 7 .. 12
			RLhelper.setSignalBandwidth(7);   // 0 .. 9
			RLhelper.setCodingRate4(2);       // 1 .. 4
			return;
		}
		// Default : SF=7, BW = 7, CR=1
		RLhelper.setSpreadingFactor(7);   // 7 .. 12
		RLhelper.setSignalBandwidth(8);   // 0 .. 9
		RLhelper.setCodingRate4(1);       // 1 .. 4
	}

	void setOCP(uint8_t mA)
	{
		RLhelper.setOCP(mA);
	}

	void publishPaquet(rl_packets* packet, byte version = 0)
	{
	  if (version == 0 || version == RL_CURRENT_VERSION)
	  {
		packet->current.crc = CRC_PRELOAD;
		for (uint8_t i = 0; i < sizeof(rl_packets)-1; i++)
		{
		  packet->current.crc += ((uint8_t *)&packet->current)[i];
		}
		_TXdone = false;
		RLhelper.write((uint8_t *)packet, sizeof(packet->current));
		while (_waitOnTx && !_TXdone) { delay(2); };
		return;
	  }
	}

	void publishBool(byte destinationid, byte senderid, byte childid, const uint8_t value, byte version = 0)
	{
	  rl_packets packet;
	  switch (version) {
		  case 1:
		  default:
			  packet.current.destinationID = destinationid;
			  packet.current.senderID = senderid;
			  packet.current.childID = childid;
			  packet.current.sensordataType = (E_BINARYSENSOR << 3) + D_BOOL;
			  packet.current.data.num.value = value;
		  break;
	  }
	  publishPaquet(&packet, version);
	}

	void publishNum(byte destinationid, byte senderid, byte childid, const long value, byte version = 0)
	{
	  rl_packets packet;
	  switch (version) {
		  case 1:
		  default:
			  packet.current.destinationID = destinationid;
			  packet.current.senderID = senderid;
			  packet.current.childID = childid;
			  packet.current.sensordataType = (E_NUMERICSENSOR << 3) + D_NUM;
			  packet.current.data.num.value = value;
			  packet.current.data.num.divider = 1;
		  break;
	  }
	  publishPaquet(&packet, version);
	}

	void publishFloat(byte destinationid, byte senderid, byte childid, const long value, const int divider, byte version = 0)
	{
	  rl_packets packet;
	  switch (version) {
		  case 1:
		  default:
			  packet.current.destinationID = destinationid;
			  packet.current.senderID = senderid;
			  packet.current.childID = childid;
			  packet.current.sensordataType =  (E_NUMERICSENSOR << 3) + D_FLOAT;
			  packet.current.data.num.value = value;
			  packet.current.data.num.divider = divider;
		  break;
	  }
	  publishPaquet(&packet, version);
	}

	void publishSwitch(byte destinationid, byte senderid, byte childid, const uint8_t value, byte version = 0)
	{
	  rl_packets packet;
	  switch (version) {
		  case 1:
		  default:
			  packet.current.destinationID = destinationid;
			  packet.current.senderID = senderid;
			  packet.current.childID = childid;
			  packet.current.sensordataType = (E_SWITCH << 3) + D_NUM;
			  packet.current.data.num.value = value;
			  packet.current.data.num.divider = 1;
		  break;
	  }
	  publishPaquet(&packet, version);
	}

	void publishText(byte destinationid, byte senderid, byte childid, const char* text, byte version = 0)
	{
	  rl_packets packet;
	  byte l = strlen(text);
	  if (l > MAX_PACKET_DATA_LEN) {
		l = MAX_PACKET_DATA_LEN;
	  }
	  switch (version) {
		  case 1:
		  default:
			  packet.current.destinationID = destinationid;
			  packet.current.senderID = senderid;
			  packet.current.childID = childid;
			  packet.current.sensordataType =  (E_TEXTSENSOR << 3) + D_TEXT;
			  strncpy(packet.current.data.text, text, l);
			  if (l < MAX_PACKET_DATA_LEN) {
				packet.current.data.text[l] = 0;
			  }
		  break;
	  }
	  publishPaquet(&packet, version);
	}

	void publishTag(byte destinationid, byte senderid, byte childid, const uint32_t tagH, const uint32_t tagL, const byte readerID, const byte readerType, byte version = 0)
	{
	  rl_packets packet;
	  switch (version) {
		  case 1:
		  default:
			  packet.current.destinationID = destinationid;
			  packet.current.senderID = senderid;
			  packet.current.childID = childid;
			  packet.current.sensordataType =  (E_TAG << 3) + D_TAG;
			  packet.current.data.tag.tagH = tagH;
			  packet.current.data.tag.tagL = tagL;
			  packet.current.data.tag.readerID = readerID;
			  packet.current.data.tag.readerType = readerType;
		  break;
	  }
	  publishPaquet(&packet, version);
	}

	void publishRaw(byte destinationid, byte senderid, byte childid, const uint8_t* data, const byte len, byte version = 0)
	{
	  rl_packets packet;
	  switch (version) {
		  case 1:
		  default:
			  packet.current.destinationID = destinationid;
			  packet.current.senderID = senderid;
			  packet.current.childID = childid;
			  packet.current.sensordataType =  (E_CUSTOM << 3) + D_RAW;
			  for (byte b = 0; b < len && b < MAX_PACKET_DATA_LEN; b++) {
				packet.current.data.rawByte[b] = data[b];
			  }
		  break;
	  }
	  publishPaquet(&packet, version);
	}

	void publishLight(byte destinationid, byte senderid, byte childid, uint8_t state, uint8_t brightness, uint16_t temperature, uint8_t red, uint8_t green, uint8_t blue, byte version = 0)
	{
	  rl_packets packet;
	  switch (version) {
		  case 1:
		  default:
			  packet.current.destinationID = destinationid;
			  packet.current.senderID = senderid;
			  packet.current.childID = childid;
			  packet.current.sensordataType =  (E_LIGHT << 3) + D_RAW;
			  packet.current.data.light.state = state;
			  packet.current.data.light.brightness = brightness;
			  packet.current.data.light.temperature = temperature;
			  packet.current.data.light.red = red;
			  packet.current.data.light.green = green;
			  packet.current.data.light.blue = blue;
		  break;
	  }
	  publishPaquet(&packet, version);
	}

	void publishCover(byte destinationid, byte senderid, byte childid, uint8_t command, uint8_t position, byte version = 0)
	{
	  rl_packets packet;
	  switch (version) {
		  case 1:
		  default:
			  packet.current.destinationID = destinationid;
			  packet.current.senderID = senderid;
			  packet.current.childID = childid;
			  packet.current.sensordataType =  (E_COVER << 3) + D_RAW;
			  packet.current.data.cover.state = 0;
			  packet.current.data.cover.command = command;
			  packet.current.data.cover.position = position;
		  break;
	  }
	  publishPaquet(&packet, version);
	}

	void publishConfig(byte destinationid, byte senderid, rl_configs_t* cnf, rl_conf_t cnfIdx, byte version = 0)
	{
	  rl_packets packet;
	  switch (version) {
		  case 1:
		  default:
			  packet.current.destinationID = destinationid;
			  packet.current.senderID = senderid;
			  packet.current.childID = RL_ID_CONFIG;
			  packet.current.sensordataType =  (E_CONFIG << 3) + (cnfIdx&0x7);
			  memcpy(&packet.current.data.configs, cnf, sizeof(rl_configs_t));
		  break;
	  }
	  publishPaquet(&packet, version);
	}
  private:
    uint8_t _waitOnTx;
};
