#include <chrono>
//-----------------------------//
#include "SerialConnector.h"
//-----------------------------//
#define PACKET_SIZE			32

#define CONN_REQUEST_TAG 	0x0000AAAA
#define SPEED_TAG 			0x0000AAAB
#define PING_TAG			0x0000AAAC
#define UNDEFINED           0XFFFFFFFE
#define SHUTDOWN            0XFFFFFFFF
//-----------------------------//

int main() {
    SerialConnector Connector("/dev/ttyACM1", B115200, 2000, PACKET_SIZE);

    uint32_t PingInterval = 1000;
    std::chrono::system_clock::time_point LastPing;
    std::chrono::duration <double> Duration {};

    bool Connected = false;
    bool Running = true;

    uint8_t ReadBuffer[PACKET_SIZE];
    uint8_t WriteBuffer[PACKET_SIZE];

    uint32_t Time;

    uint32_t WriteTag           = UNDEFINED;
    uint32_t ReadTag            = UNDEFINED;

    while (Running) {
        if (!Connected) {
            if (Connector.readSerial(ReadBuffer) > 0) {
                memcpy(&ReadTag, ReadBuffer, 4);

                if (ReadTag == CONN_REQUEST_TAG) {
                    std::cout << "Request" << std::endl;
                    WriteTag = PING_TAG;
                    memcpy(WriteBuffer, &WriteTag, 4);
                    Connector.writeSerial(WriteBuffer);
                    Connected = true;

                    WriteTag = UNDEFINED;
                }

                ReadTag = UNDEFINED;
            } else {
                std::cerr << "Timeout" << std::endl;
            }
        } else {
            Duration = std::chrono::system_clock::now() - LastPing;

            if (Duration.count() * 1000 > PingInterval) {
                WriteTag = PING_TAG;
                memcpy(WriteBuffer, &WriteTag, 4);
                Connector.writeSerial(WriteBuffer);

                LastPing = std::chrono::system_clock::now();

                std::cout << "Ping" << std::endl;

                WriteTag = UNDEFINED;
            }

            if (Connector.readSerial(ReadBuffer) > 0) {
                memcpy(&ReadTag, ReadBuffer, 4);

                switch (ReadTag) {
                    case SPEED_TAG:
                        break;
                    case PING_TAG:
                        memcpy(&Time, ReadBuffer + 4, 4);

                        std::cout << "Response!" << std::endl;
                        std::cout << Time << std::endl;
                        break;
                    default:
                        std::cerr << "Something wend wrong: " << ReadTag << std::endl;
                        Running = false;
                }

                ReadTag = UNDEFINED;
            } else {
                std::cerr << "Timeout" << std::endl;
            }
        }
    }

    return 0;
}
