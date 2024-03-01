//
// Created by Harshavardhan Karnati on 01/02/2024.
//
#include <iostream>
#include <memory>

#include "../include/parametricAnalysis.h"

std::shared_ptr<parametricAnalysis_h::parametric> parametric_analysis (bool read_from_jsons) {
    constexpr float MTOW = 3000.0f;
    constexpr float altitudeCruise = 7620.0f;
    constexpr float aspectRatio = 10.0f;
    constexpr float oswaldRatio = 0.80f;
    constexpr float velocityCruise = 100.0f;
    constexpr float velocityClimb = velocityCruise;
    constexpr float climbRate = 0.06f;
    constexpr float velocityFirstClimb = 20.0f;
    constexpr float climbRateFirst = 3.6666666f;
    constexpr float velocitySecondClimb = 1.3 * 40.0f;
    constexpr float climbRateSecond = 0.125f;
    constexpr float altitudeClimbFirst = 16.75f;
    constexpr float altitudeClimbSecond = 45.75f;
    constexpr float altitudeClimb = 1530.5f;
    constexpr float CdoTarget = 0.01f;

    auto parametric_object = std::make_shared<parametricAnalysis_h::parametric>(
                                                MTOW,
                                                altitudeCruise,
                                                aspectRatio,
                                                oswaldRatio,
                                                velocityCruise,
                                                velocityClimb,
                                                climbRate,
                                                velocityFirstClimb,
                                                climbRateFirst,
                                                velocitySecondClimb,
                                                climbRateSecond,
                                                altitudeClimbFirst,
                                                altitudeClimbSecond,
                                                altitudeClimb,
                                                CdoTarget,
                                                read_from_jsons);
    parametric_object->getParametrics();
    return parametric_object;
}