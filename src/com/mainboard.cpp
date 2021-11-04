#include "rostron_sender/com/mainboard.h"

Mainboard::Mainboard(std::string port, unsigned int baudrate) {
    serial::Timeout to(serial::Timeout::max(), 1000, 1, 10, 10);
    mySerial = new serial::Serial(port, baudrate, to);
}
