#include <Arduino.h>
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include "CommPacketCommon.h"

#define NRF24_CE_PIN 7
#define NRF24_CSN_PIN 8

void writePacketToSerial(uint8_t* data, size_t size);

RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);
uint32_t lastStatusUpdate = 0;

void setup() 
{
    Serial.begin(115200);
    while (!Serial);

    if (!radio.begin()) 
    {
      Serial.println("Radio Module Not Responding");
      while (true);
    }

    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_2MBPS);
    radio.setRetries(1, 5);
    radio.openWritingPipe(ROBOT_ADDRESS);
    radio.openReadingPipe(1, PC_ADDRESS); 
    radio.startListening();
}

void loop() 
{
    static bool radioAck = false;
    static bool serialAck = false;
    static uint32_t lastTx = 0;

    static EncoderDataPacket encoderRX;
    static AnchorRangePacket rangeRX;
    static StatusPacket status;

    while (Serial.available())
    {
        if (Serial.peek() == PACKET_ACK)
        {
            Serial.read();
            serialAck = true;
        }
        
        else if (Serial.peek() == PACKET_START_BYTE)
        {
            CommandPacket rxSerialPacket;
            Serial.readBytes((uint8_t*)&rxSerialPacket, sizeof(CommandPacket));

            if (rxSerialPacket.header == PACKET_HEADER && rxSerialPacket.packetID == COMMAND_PACKET_ID)
            {
                radio.stopListening();
                radioAck = radio.write((uint8_t*)&rxSerialPacket, sizeof(CommandPacket));
                radio.startListening();
            }
        }

        else
        {
            Serial.read();
        }
    }
    
    if (radio.available())
    {
        uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
        radio.read((uint8_t*)&encoderRX, sizeof(encoderRX));
        writePacketToSerial((uint8_t*)&encoderRX, sizeof(encoderRX));
    }

    if ((millis() - lastStatusUpdate) > STATUS_INTERVAL_MS)
    {
        status.connected = radioAck;
        writePacketToSerial((uint8_t*)&status, sizeof(StatusPacket));
        lastStatusUpdate = millis();
    }
    
}

void writePacketToSerial(uint8_t* data, size_t size)
{
    Serial.write(data, size);
    for (int i = size; i < 32; i++)
    {
        Serial.write(0x00);
    }
}


    //if (true)
    //{
    //    EncoderTX = {PACKET_HEADER, ENCODER_PACKET_ID, 0.0, 0.0, 0.0, 0.0};
    //    SendPacket((uint8_t*)&EncoderTX, sizeof(EncoderDataPacket));
    //
    //    RangeTX = {PACKET_HEADER, RANGE_PACKET_ID, 'A', 9.73};
    //    SendPacket((uint8_t*)&RangeTX, sizeof(AnchorRangePacket));
//
    //    serialAck = false;
    //    lastTx = millis();
    //    digitalWrite(LED_BUILTIN, LOW);
    //}

    //if (serialAck == false)
    //{
    //    if ((millis() - lastTx) > SERIAL_MAX_WAIT_MS)
    //    {
    //        serialAck = true;
    //        digitalWrite(LED_BUILTIN, HIGH);
    //    }
    //}