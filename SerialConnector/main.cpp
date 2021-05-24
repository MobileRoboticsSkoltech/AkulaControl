#include <chrono>
#include <thread>
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
    SerialConnector Connector("/dev/ttyACM5", B115200, 10, PACKET_SIZE);

    uint32_t PingInterval = 20;
    std::chrono::system_clock::time_point LastPing;
    std::chrono::duration <double> Duration {};

    std::chrono::system_clock::time_point Start;
    std::chrono::system_clock::time_point Stop;
    std::chrono::duration <double> WaitDuration {};

    bool Connected = false;
    bool Running = true;

    uint8_t ReadBuffer[PACKET_SIZE];
    uint8_t WriteBuffer[PACKET_SIZE];

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

                //----------//

                WriteTag = SPEED_TAG;
                uint32_t DeltaSpeed = 2;

                memcpy(WriteBuffer, &WriteTag, 4);
                memcpy(WriteBuffer + 4, &DeltaSpeed, 4);
                Connector.writeSerial(WriteBuffer);

                LastPing = std::chrono::system_clock::now();

                WriteTag = UNDEFINED;
            }

            Start = std::chrono::system_clock::now();

            if (Connector.readSerial(ReadBuffer) > 0) {
                memcpy(&ReadTag, ReadBuffer, 4);

                switch (ReadTag) {
                    case SPEED_TAG:
                        uint32_t Speed;
                        memcpy(&Speed, ReadBuffer + 4, 4);

                        std::cout << "Speed" << std::endl;

                        break;
                    case PING_TAG:
                        std::cout << "Response!" << std::endl;
                        break;
                    case CONN_REQUEST_TAG:
                        std::cout << "Request: skip" << std::endl;
                        break;
                    default:
                        std::cerr << "Something wend wrong: " << ReadTag << std::endl;
                        Running = false;
                }

                ReadTag = UNDEFINED;
            }

            Stop = std::chrono::system_clock::now();

            WaitDuration = Stop - Start;

//            std::cout << WaitDuration.count() << std::endl;

//            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    return 0;
}
