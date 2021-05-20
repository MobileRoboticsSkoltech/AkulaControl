#include <chrono>
//-----------------------------//
#include "SerialConnector.h"
//-----------------------------//
#define PACKET_SIZE			32

#define CONN_REQUEST_TAG 	0x0000AAAA
#define SPEED_TAG 			0x0000AAAB
#define PING_TAG			0x0000AAAC
//-----------------------------//

int main() {
    SerialConnector Connector("/dev/ttyACM1", B115200, 100, PACKET_SIZE);

    uint32_t PingInterval = 1000;
    std::chrono::system_clock::time_point LastPing;
    std::chrono::duration <double> Duration {};

    bool Connected = false;
    bool Running = true;

    uint8_t ReadBuffer[PACKET_SIZE];
    uint8_t WriteBuffer[PACKET_SIZE];
    uint32_t TotalSpeed = 0;
    uint32_t DeltaSpeed = 1;

    uint32_t Time;

    uint32_t Tag;

    memcpy(WriteBuffer, &DeltaSpeed, PACKET_SIZE);

    while (Running) {
        if (!Connected) {
            if (Connector.readSerial(ReadBuffer) > 0) {
                memcpy(&Tag, ReadBuffer, 4);

                if (Tag == CONN_REQUEST_TAG) {
                    std::cout << "Request" << std::endl;
                    Tag = PING_TAG;
                    memcpy(WriteBuffer, &Tag, 4);
                    Connector.writeSerial(WriteBuffer);
                    Connected = true;
                }
            } else {
                std::cerr << "Timeout" << std::endl;
            }
        } else {
            Duration = std::chrono::system_clock::now() - LastPing;

            if (Duration.count() > PingInterval) {
                Tag = PING_TAG;
                memcpy(WriteBuffer, &Tag, 4);
                Connector.writeSerial(WriteBuffer);

                LastPing = std::chrono::system_clock::now();

                std::cout << "Ping" << std::endl;
            }

            if (Connector.readSerial(ReadBuffer) > 0) {
                memcpy(&Tag, ReadBuffer, 4);

                switch (Tag) {
                    case SPEED_TAG:
                        break;
                    case PING_TAG:
                        memcpy(&Time, ReadBuffer + 4, 4);

                        std::cout << "Response!" << std::endl;
                        std::cout << Time << std::endl;
                        break;
                    default:
                        std::cerr << "Something wend wrong!" << std::endl;
                        Running = false;
                }
            } else {
                std::cerr << "Timeout" << std::endl;
            }



        }
    }

    return 0;
}
