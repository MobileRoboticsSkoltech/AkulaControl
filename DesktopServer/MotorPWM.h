//
// Created by devilox on 6/28/21.
//
//-----------------------------//
#ifndef DESKTOPSERVER_MOTORPWM_H
#define DESKTOPSERVER_MOTORPWM_H
//-----------------------------//
#include <cstdint>
#include <cmath>
#include <iostream>
//-----------------------------//
/**
 * @description
 * This class implements everything related to motors and provides data suitable for transferring to MCU
 */
class MotorPWM {
public:
    MotorPWM(uint32_t tMaxPWM, float tMaxCoordRadius);

    [[nodiscard]] std::pair <int32_t, int32_t> getMotorsPWM(float tPosX, float tPosY) const;
private:
    uint32_t mMaxPWM;
    float mMaxCoordRadius;
};
//-----------------------------//
#endif
