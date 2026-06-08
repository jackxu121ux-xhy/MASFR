#include "main.h"
#include "delay.h"
#include "SMS_STS.h"

void robotOneStepForward(int8_t firstSide, float division);
void robotOneStepLeft(float division);
void robotOneStepRight(float division);
void robotOneStepBackward(int8_t firstSide, float division);
void robotContinuousForward(int8_t firstSide, float division, uint8_t* stopFlag);