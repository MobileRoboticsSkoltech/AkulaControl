#include <cmath>
//-----------------------------//
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
//-----------------------------//
#include <yaml-cpp/yaml.h>
//-----------------------------//
#include "AkulaPackage/SerialConnector.h"
#include "dSocket/dSocket.h"
#include "akula_package/msg/encoders.hpp"
//-----------------------------//
constexpr size_t PacketSize = 22;
//-----------------------------//
class AkulaMainNode : public rclcpp::Node {
public:
    AkulaMainNode() : Node("AkulaMainNode") {
        mPubIMU                     = create_publisher <sensor_msgs::msg::Imu>("mcu_imu", 1000);
        mPubTemp                    = create_publisher <sensor_msgs::msg::Temperature>("mcu_imu_temp", 1000);
        mPubCameraTS                = create_publisher <sensor_msgs::msg::TimeReference>("mcu_cameras_ts", 1000);
        mEncoders                   = create_publisher <akula_package::msg::Encoders>("encoders", 1000);

        std::string PackagePath     = ament_index_cpp::get_package_share_directory("AkulaPackage");
        YAML::Node Config           = YAML::LoadFile(PackagePath + "/config/imu.yaml");
        std::string SerialPath      = Config["serial"].as <std::string>();

        mConnector                  = new SerialConnector(SerialPath, B115200, 1000, PacketSize);

        //----------//

        dSocketResult SocketRes;
        mSocketUDP = new dSocket(true);

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

        mServerReadThread = std::async(std::launch::async, &AkulaMainNode::serverReadCallback, this);
    }
    ~AkulaMainNode() {
        if (mConnector) {
            delete(mConnector);
            mConnector = nullptr;
        }

        if (mSocketUDP) {
            delete(mSocketUDP);
            mSocketUDP = nullptr;
        }

        dSocketResult SocketRes;

        if (mServerReadThread.valid()) {
            if ((SocketRes = mServerReadThread.get()) != dSocketResult::SUCCESS) {
                std::cerr << "mServerReadThread error: " << static_cast <uint32_t>(SocketRes) << std::endl;
            }
        }
    }

    //----------//

    void startLoop() {
        while (rclcpp::ok()) {
            mConnector -> readSerial(mBuffer);

            memcpy(&mSeconds, mBuffer, 4);
            memcpy(&mTimeTIM5, mBuffer + 4, 4);

            memcpy(&mTempVal, mBuffer + 8, 2);
            mAccel[0] = (static_cast <double>(mTempVal) + 422.9) / 16361.7 * 9.81;
            memcpy(&mTempVal, mBuffer + 10, 2);
            mAccel[1] = (static_cast <double>(mTempVal) + 174.15) / 16411.45 * 9.81;
            memcpy(&mTempVal, mBuffer + 12, 2);
            mAccel[2] = (static_cast <double>(mTempVal) + 1781.35) / 16533.85 * 9.81;

            for (int i = 0; i < 3; i++) {
                memcpy(&mTempVal, mBuffer + 16 + i * 2, 2);
                mGyro[i] = static_cast <double>(mTempVal) / 131.0 / 180.0 * M_PI;
            }

            memcpy(&mTempVal, mBuffer + 14, 2);
            mTemp = static_cast <double>(mTempVal) / 333.87 + 21;

            //----------//

            publishIMU();
            publishTemperature();
        }
    }
private:
    SerialConnector*                mConnector                  = nullptr;
    dSocket*                        mSocketUDP                  = nullptr;

    std::future <dSocketResult>     mServerReadThread;

    uint8_t                         mBuffer[PacketSize];
    int16_t                         mTempVal;

    uint32_t                        mSeconds;
    uint32_t                        mTimeTIM5;
    double                          mAccel[3];
    double                          mTemp;
    double                          mGyro[3];

    //----------//

    std::shared_ptr <rclcpp::Publisher <sensor_msgs::msg::Imu>>             mPubIMU;
    std::shared_ptr <rclcpp::Publisher <sensor_msgs::msg::Temperature>>     mPubTemp;
    std::shared_ptr <rclcpp::Publisher <sensor_msgs::msg::TimeReference>>   mPubCameraTS;
    std::shared_ptr <rclcpp::Publisher <akula_package::msg::Encoders>>      mEncoders;

    //----------//

    dSocketResult serverReadCallback() {
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

    //----------//

    void publishIMU() {
        sensor_msgs::msg::Imu Msg;

        Msg.header.frame_id = "mcu_imu";
        Msg.header.stamp = rclcpp::Time(mSeconds + mTimeTIM5);

        Msg.linear_acceleration.x = mAccel[0];
        Msg.linear_acceleration.y = mAccel[1];
        Msg.linear_acceleration.z = mAccel[2];

        Msg.angular_velocity.x = mGyro[0];
        Msg.angular_velocity.y = mGyro[1];
        Msg.angular_velocity.z = mGyro[2];

        mPubIMU -> publish(Msg);
    }

    void publishTemperature() {
        sensor_msgs::msg::Temperature Msg;

        Msg.header.frame_id = "mcu_imu_temp";
        Msg.header.stamp = rclcpp::Time(mSeconds + mTimeTIM5);
        Msg.temperature = mTemp;

        mPubTemp -> publish(Msg);
    }
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

    auto AkulaNode = std::make_shared <AkulaMainNode>();
    AkulaNode -> startLoop();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
