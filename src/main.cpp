/*
    This file is resonponsible for forwarding packets 
    received from the mobile robot, over the NRF24 radio, 
    to the PC via Serial. It also forwards packets received 
    from the PC to the mobile robot.
*/

#include <Arduino.h>
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include "CommProtocal.hpp"

#define NRF24_CE_PIN 7
#define NRF24_CSN_PIN 8

#define PC_ADDRESS 0xA5
#define ROBOT_ADDRESS 0xB5
#define SERIAL_MAX_WAIT_MS 10
#define STATUS_INTERVAL_MS 100

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
    radio.setRetries(1, 0);
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
    static LandmarkPacket rangeRX;
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
            uint16_t checksum = calculateChecksum((uint8_t*)&rxSerialPacket, sizeof(CommandPacket));
            
            if (rxSerialPacket.header == PACKET_HEADER && rxSerialPacket.packetID == COMMAND_PACKET_ID && rxSerialPacket.Checksum == checksum)
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
        uint8_t rxBuffer[32];
        uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
        radio.read(rxBuffer, bytes); // read the payload into the rangeRX struct

        if (rxBuffer[2] == LANDMARK_PACKET_ID)
        {
            memcpy(&rangeRX, rxBuffer, sizeof(LandmarkPacket)); // read the payload into the rangeRX struct
            uint16_t checksum = calculateChecksum((uint8_t*)&rangeRX, sizeof(LandmarkPacket)); // read the payload into the rangeRX struct);

            if (rangeRX.Checksum == checksum)
            {
                writePacketToSerial((uint8_t*)&rangeRX, sizeof(LandmarkPacket)); // read the payload into the rangeRX struct);
            } 
        }
        
        else if (rxBuffer[2] == ENCODER_PACKET_ID)
        {
            memcpy(&encoderRX, rxBuffer, sizeof(EncoderDataPacket)); // read the payload into the rangeRX struct
            uint16_t checksum = calculateChecksum((uint8_t*)&encoderRX, sizeof(EncoderDataPacket)); // read the payload into the rangeRX struct);

            if (encoderRX.Checksum == checksum)
            {
                writePacketToSerial((uint8_t*)&encoderRX, sizeof(EncoderDataPacket)); // read the payload into the rangeRX struct);
            } 
        }
    }

    if ((millis() - lastStatusUpdate) > STATUS_INTERVAL_MS)
    {
        status.connected = radioAck;
        status.Checksum = calculateChecksum((uint8_t*)&status, sizeof(StatusPacket));
        writePacketToSerial((uint8_t*)&status, sizeof(StatusPacket));
        lastStatusUpdate = millis();
    }
}