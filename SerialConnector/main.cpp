#include "SerialConnector.h"
//-----------------------------//
int main() {
    SerialConnector Connector("/dev/ttyACM1", B115200, 1000, 4);

    uint8_t Buff[4];
    uint32_t Bytes;

    uint8_t Test[32];

    while (true) {
        Connector.writeSerial(Test);
        Connector.readSerial(Buff);

        memcpy(&Bytes, Buff, 4);

        std::cout << Bytes << std::endl;
    }

    return 0;
}
