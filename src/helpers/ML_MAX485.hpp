#pragma once

#include <Arduino.h>

#define BUFFER_SIZE 64

volatile char rxBuffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;
volatile bool packetReady = false;


#if defined(ESP32)
#include "driver/uart.h"
#include "soc/uart_struct.h"
#define UART_NUM 1

void IRAM_ATTR uart_intr_handle(void* arg) {
    size_t rx_fifo_len;
    uint16_t i;

    uart_get_buffered_data_len(UART_NUM, &rx_fifo_len);
    uint8_t data[rx_fifo_len];
    uart_read_bytes(UART_NUM, data, rx_fifo_len, portMAX_DELAY);

    for (i = 0; i < rx_fifo_len; i++) {
        char c = data[i];

        // Paquet ASCII entre STX et ETX
        if (c == 0x02) {  // STX
            bufferIndex = 0;
        }

        if (bufferIndex < BUFFER_SIZE) {
            rxBuffer[bufferIndex++] = c;
DSerial.print(c);
        }

        if (c == 0x03) {  // ETX
            packetReady = true;
        }
    }
}
#else
void serialEvent() {
  char c = USART0.RXDATAL;
  static bool receiving = false;
  if (c == 0x02) { // STX
    bufferIndex = 0;
    receiving = true;
    return;
  }
  if (receiving) {
DSerial.print(c);
    if (bufferIndex < BUFFER_SIZE) {
      rxBuffer[bufferIndex++] = c;
    }
    if (c == 0x03) { // ETX
      packetReady = true;
      receiving = false;
    }
  }
}
#endif
class MLhelper_MAX485 : public MLhelper_base {
    private:
        HardwareSerial* rs485;
        uint8_t deRePin;
        int spreadingFactor;
        long _baudrate;

    public:
        // Constructeur
        MLhelper_MAX485()
        {
            deRePin = ML_MAX485_PIN_DE;
            spreadingFactor = 20;
            _baudrate = 115200;
#if defined(ESP32)
            rs485 = &Serial1; //new HardwareSerial(ML_MAX485_PIN_RX, ML_MAX485_PIN_TX);
#else
            Serial.swap(1);
            rs485 = &Serial; //new HardwareSerial(ML_MAX485_PIN_RX, ML_MAX485_PIN_TX);
#endif
            pinMode(deRePin, OUTPUT);
            receiveMode();
        }
        
        // Initialisation
        int begin(long baudRate) {
            _baudrate = baudRate;
#if defined(ESP32)
            rs485->begin(_baudrate, SERIAL_8N1, ML_MAX485_PIN_RX, ML_MAX485_PIN_TX);
            uart_isr_register(UART_NUM, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, NULL);
            uart_enable_rx_intr(UART_NUM);
#else
            rs485->begin(_baudrate);
            USART0.CTRLA |= USART_RXCIE_bm;
#endif
            return true;
        }
        void end()
        {
            rs485->end();
        }

        void idle()
        {
        }

        void sleep()
        {
        }
        
        // Contrôle direction MAX485
        int transmitMode()
        {
DSerial.println("TXmode");
            digitalWrite(deRePin, HIGH);
            delayMicroseconds(10); // Stabilisation
            return true;
        }
        
        int receiveMode()
        {
DSerial.println("RXmode");
            digitalWrite(deRePin, LOW);
            delayMicroseconds(10); // Stabilisation
            return true;
        }

        void setSpreadingFactor(int sf)
        {
            spreadingFactor = (sf%9) * 10;
        }

        void setSignalBandwidth(long baudrate)
        {
            _baudrate = baudrate;
#if defined(ESP32)
            rs485->begin(_baudrate, SERIAL_8N1, ML_MAX485_PIN_RX, ML_MAX485_PIN_TX);
#else
            rs485->begin(_baudrate);
#endif
        }        

        void setCodingRate4(int denominator)
        {
            // not used
        }

        void setOCP(uint8_t mA)
        {
            // not used
        }

#if defined(ESP32)
        void printHex (const uint8_t b)
        {
          char x=(b>>4)|'0';
          if (x > '9')
            x += 7;
          rs485->write(x);
          x=(b&0x0F)|'0';
          if (x > '9')
            x += 7;
          rs485->write(x);
        }
        void printHex(uint8_t* p,uint8_t len)
        {
          for (byte i=0;i<len;i++) {
            printHex(*p++);
          }
        }
#endif
        // Méthode principale d'envoi (données binaires)
        int write(byte* data, uint8_t len)
        {
            transmitMode();
            // Envoi des données binaires
            rs485->write(2);
#if defined(ESP32)
            printHex(data, len); //rs485->write(data, len);
#else
            rs485->printHex(data, len); //rs485->write(data, len);
#endif
            rs485->write(3);
            rs485->flush(); // Attendre fin transmission

            receiveMode();
            if (_onInternalTxDone)
            {
                _onInternalTxDone();
            }
            return true;
        }


        // Réception et traitement des messages binaires
        int read(byte* buffer, uint8_t maxLength)
        {
            receiveMode();
            int bytesRead = 0;

            while (bytesRead < maxLength && rs485->available())
            {
                buffer[bytesRead++] = rs485->read();
            }
            return bytesRead;
        }

        void setDint(uint8_t pin)
        {
            //
        }

        void setRst(uint8_t pin)
        {
            //
        }
};

/*
// Exemple d'utilisation
RS485SmartComm comm(1, RX_PIN, TX_PIN, DE_RE_PIN); // Node ID = 1

void setup() {
    Serial.begin(9600);
    comm.begin(9600);
    
    Serial.println("RS485 Smart Communication Test");
    delay(1000);
    
}

void loop() {

    // Écouter les messages entrants
    String receivedMsg = comm.receiveMessage();
    if(receivedMsg.length() > 0) {
        Serial.println("Processing: " + receivedMsg);
    }
    
    delay(100);
}
*/