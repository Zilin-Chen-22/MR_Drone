// This file is used for connect input and origina beta-flight codes

#pragma once
#include "common/axis.h"
#include "fc/rc_controls.h"

#include "posture_controller.h"

// Number of propeller
#define NUM_PROP 4

// ESC switching parameters
#define ESC_MAX 2000
#define ESC_MIN 1000

#define DISTURB_PARAMETER 0.1  // Disturb rate
#define MAX_RPM 500

void Init(void);
void escToRpm(void);
bool pidSimulation(float* gyroInput, uint32_t gyro_len, float* rcCommand, uint32_t rcCommand_len, uint32_t currentTime);
bool getRpmCommand(float *rpm_ptr, uint32_t rpm_len);
bool update_motor_data(bool activate);
bool postureController(float *postureInput, float* gyroInput, uint32_t gyro_len, float* inputCommand, uint32_t inputCommand_len, uint32_t currentTime);