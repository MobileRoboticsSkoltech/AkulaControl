//
// Created by devilox on 6/28/21.
//
//-----------------------------//
#include "MotorPWM.h"
//-----------------------------//
/**
 * @param tMaxPWM Maximum PWM value set on MCU
 * @param tMaxCoordRadius Maximum joystick radius
 */
MotorPWM::MotorPWM(uint32_t tMaxPWM, float tMaxCoordRadius) {
    mMaxPWM             = tMaxPWM;
    mMaxCoordRadius     = tMaxCoordRadius;
}
//-----------------------------//
/**
 * @description
 * This function converts coordinates for a joystick to PWM in  the rhombus-shaped area for skid-steering
 * @param tPosX Joystick X coordinate
 * @param tPosY Joystick Y coordinate
 * @return Returns pair of PWM values for left (first) and right (second) motors
 */
std::pair <int32_t, int32_t> MotorPWM::getMotorsPWM(float tPosX, float tPosY) const {
    if (tPosX == 0 && tPosY == 0) {
        return {0, 0};
    }

    float Multiplier;
    float AreaRadius;

    float NewX;
    float NewY;

    if (tPosX >= 0 && tPosY >= 0) {
        Multiplier = float(mMaxPWM) * 2 / (2 * tPosY / tPosX + 1);
        AreaRadius = std::sqrt(std::pow(Multiplier, 2.0f) + std::pow(-0.5f * Multiplier + float(mMaxPWM), 2.0f));
    } else if (tPosX < 0 && tPosY >= 0) {
        Multiplier = float(mMaxPWM) * 2 / (2 * tPosY / tPosX - 1);
        AreaRadius = std::sqrt(std::pow(Multiplier, 2.0f) + std::pow(0.5f * Multiplier + float(mMaxPWM), 2.0f));
    } else if (tPosX < 0 && tPosY < 0) {
        Multiplier = -float(mMaxPWM) * 2 / (2 * tPosY / tPosX + 1);
        AreaRadius = std::sqrt(std::pow(Multiplier, 2.0f) + std::pow(-0.5f * Multiplier - float(mMaxPWM), 2.0f));
    } else {
        Multiplier = -float(mMaxPWM) * 2 / (2 * tPosY / tPosX - 1);
        AreaRadius = std::sqrt(std::pow(Multiplier, 2.0f) + std::pow(0.5f * Multiplier - float(mMaxPWM), 2.0f));
    }

    NewX = tPosX / mMaxCoordRadius * AreaRadius;
    NewY = tPosY / mMaxCoordRadius * AreaRadius;

    std::pair <int32_t, int32_t> PWM {
            NewY + NewX / 2,
            NewY - NewX / 2
    };

    return PWM;
}