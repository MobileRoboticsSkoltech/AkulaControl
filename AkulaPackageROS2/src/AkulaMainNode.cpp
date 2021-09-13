#include <cstdio>
//-----------------------------//
#include <cmath>
//-----------------------------//
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
//-----------------------------//
#include "AkulaPackage/SerialConnector.h"
//-----------------------------//
constexpr size_t PacketSize = 22;
//-----------------------------//
class AkulaMainNode : public rclcpp::Node {
public:
    AkulaMainNode() : Node("AkulaMainNode") {
        mPubIMU = create_publisher <sensor_msgs::msg::Imu>("mcu_imu", 1000);
        mPubTemp = create_publisher <sensor_msgs::msg::Temperature>("mcu_imu_temp", 1000);
        mPubCameraTS = create_publisher <sensor_msgs::msg::TimeReference>("mcu_cameras_ts", 1000);

        mConnector = new SerialConnector("/dev/ttyACM2", B115200, 1000, PacketSize);
    }
    ~AkulaMainNode() {
        if (mConnector) {
            delete(mConnector);
            mConnector = nullptr;
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
    SerialConnector*    mConnector              = nullptr;

    uint8_t             mBuffer[PacketSize];
    int16_t             mTempVal;

    uint32_t            mSeconds;
    uint32_t            mTimeTIM5;
    double              mAccel[3];
    double              mTemp;
    double              mGyro[3];

    //----------//

    std::shared_ptr <rclcpp::Publisher <sensor_msgs::msg::Imu>>             mPubIMU;
    std::shared_ptr <rclcpp::Publisher <sensor_msgs::msg::Temperature>>     mPubTemp;
    std::shared_ptr <rclcpp::Publisher <sensor_msgs::msg::TimeReference>>   mPubCameraTS;

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
