#pragma once

#include <iostream>

#include "serial/serial.h"

#define MAX_PACKET 2048

#define ACTION_ON (1 << 0)
#define ACTION_KICK1 (1 << 1)
#define ACTION_KICK2 (1 << 2)
#define ACTION_DRIBBLE (1 << 3)
#define ACTION_CHARGE (1 << 5)
#define ACTION_TARE_ODOM (1 << 7)

typedef struct {
    uint8_t id;
    uint8_t actions;

    int16_t x_speed;  // Kinematic orders [mm/s]
    int16_t y_speed;
    int16_t t_speed;  // Rotation in [mrad/s]

    uint8_t kickPower;  // Kick power (this is a duration in [x25 uS])
} __attribute__((packed)) packet_mainboard;

class Mainboard {
   public:
    Mainboard(std::string port, unsigned int baudrate);
    serial::Serial* mySerial;

    void addPacket(uint8_t* packet);
    void appendToPacket(uint8_t packet);
    void send();

   private:
    uint8_t tmp_packet_[2048];
    size_t tmp_packet_size = 0;
};