#include "SerialConnector.h"
//-----------------------------//
int main() {
    SerialConnector Connector("/dev/ttyACM2", B115200, 1000, 4);

    uint8_t Buff[4];

    for (int i = 0; i < 20; i++) {
        Connector.readSerial(Buff);

        std::cout << reinterpret_cast <char*>(Buff) << std::endl;
    }

    return 0;
}
