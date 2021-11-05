#include "rostron_sender/com/mainboard.h"

Mainboard::Mainboard(std::string port, unsigned int baudrate) {
    serial::Timeout to(serial::Timeout::max(), 1000, 1, 10, 10);
    mySerial = new serial::Serial(port, baudrate, to);
}

void Mainboard::appendToPacket(uint8_t c) {
    if (tmp_packet_size < sizeof(tmp_packet_)) {
        tmp_packet_[tmp_packet_size] = c;
        tmp_packet_size += 1;
    }
}

void Mainboard::addPacket(uint8_t* packet) {
    appendToPacket(0xaa);
    appendToPacket(0x55);
    for (size_t i = 0; i < sizeof(packet_mainboard); ++i) {
        appendToPacket(packet[i]);
    }
    appendToPacket(0xFF);
}

void Mainboard::send() {
    mySerial->write(tmp_packet_, tmp_packet_size);
    tmp_packet_size = 0;
}
