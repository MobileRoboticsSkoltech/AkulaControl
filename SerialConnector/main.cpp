#include "SerialConnector.h"
//-----------------------------//
#define PACKET_SIZE			32

#define CONN_REQUEST_TAG 	0x0000AAAA
#define SPEED_TAG 			0x0000AAAB
#define PING_TAG			0x0000AAAC
//-----------------------------//
int main() {
    SerialConnector Connector("/dev/ttyACM1", B115200, 1000, PACKET_SIZE);

    bool Connected = false;

    uint8_t ReadBuffer[PACKET_SIZE];
    uint8_t WriteBuffer[PACKET_SIZE];
    uint32_t TotalSpeed = 0;
    uint32_t DeltaSpeed = 1;

    uint32_t Tag;

    memcpy(WriteBuffer, &DeltaSpeed, PACKET_SIZE);

    while (true) {
        Connector.readSerial(ReadBuffer);
        memcpy(&Tag, ReadBuffer, 4);

        std::cout << "Request? " << CONN_REQUEST_TAG << std::endl;
        std::cout << "Request :" << Tag << std::endl;

//        if (Connector.readSerial(ReadBuffer) > 0) {
//            memcpy(&Tag, ReadBuffer, 4);
//
//
//
//            if (Tag == CONN_REQUEST_TAG) {
//                std::cout << "Request" << std::endl;
//                Tag = PING_TAG;
//                memcpy(WriteBuffer, &Tag, 4);
//                Connector.writeSerial(WriteBuffer);
//                Connected = true;
//                break;
//            }
//        }
    }

    while (true) {
        Connector.writeSerial(WriteBuffer);
        Connector.readSerial(ReadBuffer);

        memcpy(&Tag, ReadBuffer, 4);

        std::cout << Tag << std::endl;
    }

    return 0;
}
