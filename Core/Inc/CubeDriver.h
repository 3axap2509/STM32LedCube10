//
// Created by TheHAR on 25.09.2025.
//

#ifndef LED_CUBE_C6T6_CUBEDRIVER_H
#define LED_CUBE_C6T6_CUBEDRIVER_H
typedef enum
{
    ResetAll = 11,
    ResetLayer = 12,
    ResetLayers = 13,

    SetDelay = 21,

    DrawLine = 31,
    DrawDot = 32,
    DrawSquare = 33,
    DrawCircle = 34,

    AddLayer = 41,
}DriverCommands;

#endif //LED_CUBE_C6T6_CUBEDRIVER_HS