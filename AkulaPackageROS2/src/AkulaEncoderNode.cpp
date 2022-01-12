//
// Created by mrob on 1/12/22.
//
//-----------------------------//
#include <rclcpp/rclcpp.hpp>
//-----------------------------//
#include "dSocket/dSocket.h"
#include "akula_package/msg/encoders.hpp"
//-----------------------------//
class AkulaEncoderNode : public rclcpp::Node {
public:
    AkulaEncoderNode() : Node("AkulaEncoderNode") {
        dSocketResult SocketRes;

        mEncoders   = create_publisher <akula_package::msg::Encoders>("encoders", 1000);
        mSocketUDP  = new dSocket(true);

        if ((SocketRes = mSocketUDP -> init(dSocketProtocol::UDP)) != dSocketResult::SUCCESS) {
            switch (SocketRes) {
                case dSocketResult::WRONG_PROTOCOL:
                    throw std::runtime_error("Server::Server: WRONG_PROTOCOL");
                case dSocketResult::CREATE_FAILURE:
                    throw std::runtime_error("Server::Server: CREATE_FAILURE, " + mSocketUDP -> getLastError());
                default:
                    throw std::runtime_error("Server::Server: this should have not occurred!");
            }
        }

        if ((SocketRes = mSocketUDP -> finalize(dSocketType::SERVER, 50001)) != dSocketResult::SUCCESS) {
            switch (SocketRes) {
                case dSocketResult::WRONG_PROTOCOL:
                    throw std::runtime_error("Server::Server: WRONG_PROTOCOL");
                case dSocketResult::NO_SOCKET_TYPE:
                    throw std::runtime_error("Server::Server: NO_SOCKET_TYPE");
                case dSocketResult::BIND_FAILURE:
                    throw std::runtime_error("Server::Server: BIND_FAILURE, " + mSocketUDP -> getLastError());
                case dSocketResult::LISTEN_FAILURE:
                    throw std::runtime_error("Server::Server: LISTEN_FAILURE, " + mSocketUDP -> getLastError());
                case dSocketResult::ADDRESS_CONVERSION_FAILURE:
                    throw std::runtime_error("Server::Server: ADDRESS_CONVERSION_FAILURE, " + mSocketUDP -> getLastError());
                default:
                    throw std::runtime_error("Server::Server: this should have not occurred!");
            }
        }
    }

    ~AkulaEncoderNode() {
        if (mSocketUDP) {
            delete(mSocketUDP);
            mSocketUDP = nullptr;
        }
    }

    dSocketResult startLoop() {
        size_t PacketSize = 32;
        uint8_t Packet[PacketSize];
        sockaddr_in ClientAddr {};
        socklen_t ClientAddrLen = sizeof(ClientAddr);
        ssize_t ReadBytes;
        size_t ReadTotal = 0;
        dSocketResult Result;

        while (rclcpp::ok()) {
            while (ReadTotal < PacketSize && rclcpp::ok()) {
                Result = mSocketUDP -> readUDP(Packet + ReadTotal, PacketSize - ReadTotal, &ReadBytes,
                                               reinterpret_cast <sockaddr*>(&ClientAddr), &ClientAddrLen);

                switch (Result) {
                    case dSocketResult::SUCCESS:
                        ReadTotal += ReadBytes;
                        break;
                    case dSocketResult::RECV_TIMEOUT:
                        ReadTotal = 0;
                        break;
                    default:
                        ///---TODO: skip some particular errors and terminate on others---///
                        return dSocketResult::READ_ERROR;
                }
            }

            ReadTotal = 0;

            if (!rclcpp::ok()) {
                break;
            }

            akula_package::msg::Encoders Msg;

            memcpy(&Msg.left, Packet + 4, 8);
            memcpy(&Msg.right, Packet + 12, 8);

            mEncoders -> publish(Msg);
        }

        return dSocketResult::SUCCESS;
    }
private:
    dSocket*                                                                mSocketUDP                  = nullptr;
    std::shared_ptr <rclcpp::Publisher <akula_package::msg::Encoders>>      mEncoders;
};
//-----------------------------//
void sigintHandler(int tSigNum) {
    std::cout << "Receive signum: " << tSigNum << std::endl;
    rclcpp::shutdown();
}
//-----------------------------//
int main(int argc, char ** argv) {
    signal(SIGINT, sigintHandler);
    rclcpp::init(argc, argv);

    auto EncoderNode = std::make_shared <AkulaEncoderNode>();
    EncoderNode -> startLoop();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
