#include <Arduino.h>

#define PC_ADDRESS 0xA5
#define ROBOT_ADDRESS 0xB5

#define PACKET_ACK 0x01
#define PACKET_START_BYTE 0x55
#define PACKET_HEADER 0xAA55

#define ENCODER_PACKET_ID 0x01
#define COMMAND_PACKET_ID 0x02
#define RANGE_PACKET_ID 0x03
#define STATUS_PACKET_ID 0x04

#define SERIAL_MAX_WAIT_MS 10
#define STATUS_INTERVAL_MS 100

struct StatusPacket
{
    uint16_t header = PACKET_HEADER;
    uint8_t packetID = STATUS_PACKET_ID;
    bool connected = false;
};

struct EncoderDataPacket 
{
    uint16_t header;
    uint8_t packetID;
    float encA;
    float encB;
    float velA;
    float velB;
}; // 19 Bytes Total (In theory)

struct CommandPacket 
{
    uint16_t header;
    uint8_t packetID;
    float VelA;
    float VelB;
};

struct AnchorRangePacket 
{
    uint16_t header;
    uint8_t packetID;
    uint8_t anchorID;
    float range;
};