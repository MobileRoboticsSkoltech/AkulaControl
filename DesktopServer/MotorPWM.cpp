//
// Created by devilox on 6/28/21.
//
//-----------------------------//
#include "MotorPWM.h"
//-----------------------------//
MotorPWM::MotorPWM(uint32_t tMaxPWM, float tMaxCoordRadius) {
    mMaxPWM             = tMaxPWM;
    mMaxCoordRadius     = tMaxCoordRadius;
}
//-----------------------------//
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
        AreaRadius = std::sqrt(std::pow(Multiplier, 2.0) + std::pow(-0.5 * Multiplier + mMaxPWM, 2.0));
    } else if (tPosX < 0 && tPosY >= 0) {
        Multiplier = float(mMaxPWM) * 2 / (2 * tPosY / tPosX - 1);
        AreaRadius = std::sqrt(std::pow(Multiplier, 2.0) + std::pow(0.5 * Multiplier + mMaxPWM, 2.0));
    } else if (tPosX < 0 && tPosY < 0) {
        Multiplier = -float(mMaxPWM) * 2 / (2 * tPosY / tPosX + 1);
        AreaRadius = std::sqrt(std::pow(Multiplier, 2.0) + std::pow(-0.5 * Multiplier - mMaxPWM, 2.0));
    } else {
        Multiplier = -float(mMaxPWM) * 2 / (2 * tPosY / tPosX - 1);
        AreaRadius = std::sqrt(std::pow(Multiplier, 2.0) + std::pow(0.5 * Multiplier - mMaxPWM, 2.0));
    }

    NewX = tPosX / mMaxCoordRadius * AreaRadius;
    NewY = tPosY / mMaxCoordRadius * AreaRadius;

    std::pair <int32_t, int32_t> PWM {
            NewY + NewX / 2,
            NewY - NewX / 2
    };

    return PWM;
}