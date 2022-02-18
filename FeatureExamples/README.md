# FeatureExamples

This directory contains some examples, reduced to several files, of features used in the main project for better understanding of internal components:
1. **Encoders** include pin setup and a way to read values without involving callback function
2. **Motors** include pin setup for PWM usage and example of motor control
3. **SpiImu** includes pin setup and data reading example via SPI protocol for MPU9250 (https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf)
4. **SerialConnector** literally includes a part of the project which is responsible for communication between the main server and the stm32 board
5. **JavaClient** is also a part of the main project and based on the same approach, in terms of server-client communication, as the main server
