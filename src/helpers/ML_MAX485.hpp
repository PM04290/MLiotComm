#pragma once

#include <Arduino.h>

#define BUFFER_SIZE 64


#if defined(ESP32)

volatile char rxBuffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;
volatile bool frameReady = false;


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
        }

        if (c == 0x03) {  // ETX
            frameReady = true;
        }
    }
}

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
            rs485 = &Serial1; //new HardwareSerial(ML_MAX485_PIN_RX, ML_MAX485_PIN_TX);
            pinMode(deRePin, OUTPUT);
            receiveMode();
        }
        
        // Initialisation
        int begin(long baudRate) {
            _baudrate = baudRate;
            rs485->begin(_baudrate, SERIAL_8N1, ML_MAX485_PIN_RX, ML_MAX485_PIN_TX);
            uart_isr_register(UART_NUM, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, NULL);
            uart_enable_rx_intr(UART_NUM);
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
            digitalWrite(deRePin, HIGH);
            delayMicroseconds(10); // Stabilisation
            return true;
        }
        
        int receiveMode()
        {
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
            rs485->begin(_baudrate, SERIAL_8N1, ML_MAX485_PIN_RX, ML_MAX485_PIN_TX);
        }        

        void setCodingRate4(int denominator)
        {
            // not used
        }

        void setOCP(uint8_t mA)
        {
            // not used
        }

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

        // Méthode principale d'envoi (données binaires)
        int write(byte* data, uint8_t len)
        {
            transmitMode();
            // Envoi des données binaires
            rs485->write(2);
            printHex(data, len); //rs485->write(data, len);
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

        void process() {
            if (frameReady && _onInternalRxDone)
            {
                _onInternalRxDone(bufferIndex);
            }
        }
};
#endif

#if defined(__AVR_ATtiny1616__) || defined(__AVR_ATtiny3216__) || defined(__AVR_ATtiny1617__) || defined(__AVR_ATtiny3217__)

// États de réception
enum RxState {
    WAITING_STX,
    RECEIVING_DATA,
    FRAME_COMPLETE
};

#define STX 0x02  // Start of Text
#define ETX 0x03  // End of Text

class MLhelper_MAX485 : public MLhelper_base {
private:
    // Buffer pour les données décodées
    uint8_t dataBuffer[BUFFER_SIZE];
    uint8_t dataHead, dataTail;
    uint8_t dataLength;
    
    // État de réception de trame
    RxState rxState;
    uint8_t hexBuffer[2];
    uint8_t hexIndex;
    bool frameReady;
    
    // Configuration des pins
    uint8_t dePin;
    
    // Fonctions utilitaires
    // Conversion caractère hex en nibble
    uint8_t hexCharToNibble(char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return 0;
    }

    // Vérification caractère hex valide
    bool isValidHexChar(char c) {
        return ((c >= '0' && c <= '9') || 
                (c >= 'A' && c <= 'F') || 
                (c >= 'a' && c <= 'f'));
    }

    // Traitement des caractères reçus pour décoder les trames
    void processReceivedChar(uint8_t c) {
        switch (rxState) {
            case WAITING_STX:
                if (c == STX) {
                    rxState = RECEIVING_DATA;
                    hexIndex = 0;
                    dataLength = 0;
                }
                break;
                
            case RECEIVING_DATA:
                if (c == ETX) {
                    // Fin de trame
                    if (hexIndex == 1) {
                        // Caractère hex orphelin, on l'ignore
                        hexIndex = 0;
                    }
                    rxState = FRAME_COMPLETE;
                    frameReady = true;
                } else if (isValidHexChar(c)) {
                    // Caractère hexadécimal valide
                    hexBuffer[hexIndex] = c;
                    hexIndex++;
                    
                    if (hexIndex == 2) {
                        // Paire complète, conversion en octet
                        uint8_t b = (hexCharToNibble(hexBuffer[0]) << 4) | hexCharToNibble(hexBuffer[1]);
                        
                        if (dataLength < BUFFER_SIZE) {
                            dataBuffer[dataLength] = b;
                            dataLength++;
                        }
                        hexIndex = 0;
                    }
                } else {
                    // Caractère invalide, abandon de la trame
                    rxState = WAITING_STX;
                    hexIndex = 0;
                    dataLength = 0;
                }
                break;
                
            case FRAME_COMPLETE:
                // Ne devrait pas arriver, mais au cas où...
                if (c == STX) {
                    rxState = RECEIVING_DATA;
                    hexIndex = 0;
                    dataLength = 0;
                }
                break;
        }
    }
    
public:
    // Constructeur
    MLhelper_MAX485() {
        dePin = ML_MAX485_PIN_DE;
    }
    
    // Initialisation
    int begin(long baudrate = 115200) {
        // Configuration des pins de contrôle
        pinMode(dePin, OUTPUT);
        receiveMode(); // Mode réception par défaut
       
        Serial.swap(1); // Use Alternate MISO/MOSI for TX/RX
        Serial.begin(baudrate);
        // Initialisation des buffers et états
        dataHead = 0;
        dataTail = 0;
        dataLength = 0;
        rxState = WAITING_STX;
        hexIndex = 0;
        frameReady = false;
        
        return true;
    }

    void end() {
        Serial.end();
    }

    // Vérifier si une trame complète est disponible
    bool frameAvailable() {
        return frameReady;
    }

    // Lire une trame complète
    int read(byte* buffer, uint8_t maxLength) {
        if (!frameReady) return 0;
        
        cli(); // Désactiver les interruptions
        
        uint8_t length = min(dataLength, maxLength);
        memcpy( buffer, dataBuffer, length);
        
        // Réinitialiser pour la prochaine trame
        frameReady = false;
        rxState = WAITING_STX;
        dataLength = 0;
        hexIndex = 0;
        
        sei(); // Réactiver les interruptions
        
        return length;
    }
    
    // Obtenir la longueur de la trame prête
    uint8_t getFrameLength() {
        return frameReady ? dataLength : 0;
    }

    // Vider les trames en attente
    void flushFrames() {
        cli();
        frameReady = false;
        rxState = WAITING_STX;
        dataLength = 0;
        hexIndex = 0;
        sei();
    }
    
    // Envoi d'une trame avec données binaires
    int write(byte* data, uint8_t length) {
        transmitMode();
        
        // Envoi STX
        Serial.write(STX);
        
        // Envoi des données en ASCII hexadécimal
        for (uint8_t i = 0; i < length; i++) {
            uint8_t highNibble = (data[i] >> 4) & 0x0F;
            uint8_t lowNibble = data[i] & 0x0F;
            
            // Conversion en caractères ASCII
            char highChar = (highNibble < 10) ? ('0' + highNibble) : ('A' + highNibble - 10);
            char lowChar = (lowNibble < 10) ? ('0' + lowNibble) : ('A' + lowNibble - 10);
            
            Serial.write(highChar);
            Serial.write(lowChar);
        }
        
        // Envoi ETX
        Serial.write(ETX);
        
        receiveMode();
        
        return true;
    }
    
    // Passage en mode transmission
    int transmitMode() {
        digitalWrite(dePin, HIGH);
        delayMicroseconds(1); // Petit délai pour la commutation
        return true;
    }

    // Passage en mode réception
    int receiveMode() {
        digitalWrite(dePin, LOW);
        delayMicroseconds(1); // Petit délai pour la commutation
        return true;
    }
    
    void process() {
        while (Serial.available()) {
            uint8_t data = Serial.read();
            processReceivedChar(data);
        }
        if (frameReady && _onInternalRxDone)
        {
            _onInternalRxDone(dataLength);
        }
    }
    
    //
	void setDint(uint8_t pin) {};
	void setRst(uint8_t pin) {};
	void idle()	{};
	void sleep() {};
};

#endif
