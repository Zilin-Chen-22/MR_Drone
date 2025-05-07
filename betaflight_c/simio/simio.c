#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <time.h>


#include "config/config.h"

#include "fc/rc.h"

#include "flight/mixer_init.h"
#include "flight/pid.h"
#include "flight/pid_init.h"

#include "sensors/gyro.h"

#include "simio.h"

#include "common/maths.h"

// static float inputRate[XYZ_AXIS_COUNT];
// static float inputThruttle;


static pidProfile_t pidProfile;
static float rpm[4];

pos_pidAxisData_t pos_pidData[XYZ_AXIS_COUNT];
pos_pidf_t pos_pid[XYZ_AXIS_COUNT];

// static float motor_upwards[19][2] = {  
//     // Experiment data for ESC to RPM while ESC going up
//    {1000,0},
//    {1105.26315789474,3309},
//    {1157.89473684211,5373.5},
//    {1210.52631578947,7319},
//    {1263.15789473684,9210},
//    {1315.78947368421,10982.5},
//    {1368.42105263158,12664},
//    {1421.05263157895,14145.5},
//    {1473.68421052632,15508.5},
//    {1526.31578947368,16917.5},
//    {1578.94736842105,18260.5},
//    {1631.57894736842,19521},
//    {1684.21052631579,20686},
//    {1736.84210526316,21810},
//    {1789.47368421053,22825.5},
//    {1842.10526315789,23799.5},
//    {1894.73684210526,24679.5},
//    {1947.36842105263,25593.5},
//    {2000,25800.5}
// };

static float motor_upwards[21][2] = {  
    // Experiment data for ESC to RPM while ESC going up
    {1000, 0.000000},
    {1050, 0.161250},
    {1100, 2.580000},
    {1150, 13.061250},
    {1200, 41.280000},
    {1250, 100.781250},
    {1300, 208.980000},
    {1350, 387.161250},
    {1400, 660.480000},
    {1450, 1057.961250},
    {1500, 1612.500000},
    {1550, 2360.861250},
    {1600, 3343.680000},
    {1650, 4605.461250},
    {1700, 6194.580000},
    {1750, 8163.281250},
    {1800, 10567.680000},
    {1850, 13467.761250},
    {1900, 16927.380000},
    {1950, 21014.261250},
    {2000, 25800.000000}
};

// Init all the needed variables, including mixers and pid
void Init(void) {
    gyro.targetLooptime = 2500;     // PID loop time in us
    pidRuntime.pidStabilisationEnabled = true;
    mixerRuntime.motorCount = NUM_PROP;    // Number of motors
    mixerRuntime.motorOutputHigh = ESC_MAX;    // Max output
    mixerRuntime.motorOutputLow = ESC_MIN;     // Min output
    resetPidProfile(&pidProfile);            // Set PID default settings
    currentPidProfile = &pidProfile;         
    mixerInit(MIXER_QUADX);     // QUADX is the Drone style
    pidInit(&pidProfile);        
    mixerInitProfile();
    // Initial pos pid data
    for (uint8_t axis = ROLL; axis <= YAW; axis++) {
        pos_pid[axis].P = POS_PID_KP;
        pos_pid[axis].I = POS_PID_KI;
        pos_pid[axis].D = POS_PID_KD;
        pos_pidData[axis].P = 0;
        pos_pidData[axis].I = 0;
        pos_pidData[axis].D = 0;
        pos_pidData[axis].previous_error = 0;
        pos_pidData[axis].Sum = 0;
    }


    if (update_motor_data(true) == true) {
        printf("Successfully updated Motor data!\n");
    }
    else {
        printf("Failed to update Motor Data!\n");
    }
    srand(time(NULL));
    // update_motor_data(true);
}

// Change the ESC switching frequency into propeller RPM by linear interpolation
// The linear interpolation data is from physical experiment
void escToRpm(void) {
    int disturb_amplitude = DISTURB_PARAMETER * MAX_RPM;
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < (int)(sizeof(motor_upwards)/sizeof(motor_upwards[0]) - 1); j++){
            if(motor[i] >= motor_upwards[j][0] && motor[i] <= motor_upwards[j+1][0]){
                rpm[i] = (motor_upwards[j+1][1] * (motor[i] - motor_upwards[j][0]) + motor_upwards[j][1] * (motor_upwards[j+1][0] - motor[i]))/(motor_upwards[j+1][0]-motor_upwards[j][0]);
                // Introduce noise to RPM
                rpm[i] += ((rand()) % (disturb_amplitude * 2)- disturb_amplitude);
                if (rpm[i] > motor_upwards[(int)(sizeof(motor_upwards)/sizeof(motor_upwards[0]) - 1)][1]) {
                    rpm[i] = motor_upwards[(int)(sizeof(motor_upwards)/sizeof(motor_upwards[0]) - 1)][1];
                }
                break;
            }
        }
    }
}

// Main PID simulation part, input gyro data and angle command, output propeller RPM
// Angle command contains angular velocity in rad/s, and a throttle in N
bool pidSimulation(float* gyroInput, uint32_t gyro_len, float* inputCommand, uint32_t inputCommand_len, uint32_t currentTime) {
    if (inputCommand_len > NUM_PROP || gyro_len > XYZ_AXIS_COUNT) {
        return false;
    }
    for (int axis = ROLL; axis <= YAW; axis++) {
        setSetpointRate(inputCommand[axis] * 180 / M_PIf, axis);
        rcCommand[axis] = constrainf(500 * inputCommand[axis], 0, 500);
    }
    
    for (uint32_t axis = 0; axis < gyro_len; axis++) {
        gyro.gyroADCf[axis] = gyroInput[axis] * 180 / M_PIf;
    }
    rcCommand[THROTTLE] = inputCommand[THROTTLE] * 1000 / 100 + 1000;
    pidController(&pidProfile, currentTime);
    mixTable(currentTime);
    escToRpm();
    return true;
}

bool postureController(float *postureInput, float* gyroInput, uint32_t gyro_len, float* inputCommand, uint32_t inputCommand_len, uint32_t currentTime) {
    if (inputCommand_len > 4|| gyro_len > XYZ_AXIS_COUNT) {
        return false;
    }
    for (uint8_t axis = ROLL; axis <= YAW; axis++) {
        float error = inputCommand[axis] - postureInput[axis];
        pos_pidData[axis].P = pos_pid[axis].P * error;

        float ItermChange = error * pos_pid[axis].I;
        pos_pidData[axis].I = constrainf(pos_pidData[axis].I + ItermChange, -POS_PID_MAX_KI, POS_PID_MAX_KI);
        // pos_pidData[axis].I = error + pos_pid[axis].I;

        pos_pidData[axis].D = (error - pos_pidData[axis].previous_error) * pos_pid[axis].D / pidRuntime.pidFrequency ;

        pos_pidData[axis].Sum = constrainf(pos_pidData[axis].P + pos_pidData[axis].I + pos_pidData[axis].D, -ANGULAR_VELOCITY_MAX, ANGULAR_VELOCITY_MAX);

        pos_pidData[axis].previous_error = error;
    }

    float angular_velocity[XYZ_AXIS_COUNT + 1] = {0.0, 0.0, 0.0, 0.0};
    for (uint8_t axis = ROLL; axis <= YAW; axis++) {
        angular_velocity[axis] = pos_pidData[axis].Sum;
    }
    angular_velocity[2] = -angular_velocity[2];
    angular_velocity[3] = inputCommand[3];
    pidSimulation(gyroInput, gyro_len, angular_velocity, inputCommand_len, currentTime);
    return true;
}

// RPM read interface
bool getRpmCommand(float *rpm_ptr, uint32_t rpm_len) {
    if (rpm_len < NUM_PROP) {
        return false;
    }
    for (uint32_t i = 0; i < NUM_PROP; i++) {
        rpm_ptr[i] = rpm[i];
    }
    return true;
}

// update motor data
bool update_motor_data(bool activate){
    if (activate == false) {
        return true;
    }

    FILE *fp = fopen("../data.txt", "r");
    
    if (fp == NULL) {
        return false;
    }

    // char * lines_temp = NULL;
    char lines_temp[50];
    for (int i = 0; i < 20; i++) {
        fgets(lines_temp, 50, fp);
    }

    for (int i = 0; i < 21; i++) {
        fscanf(fp, "%f%f", &motor_upwards[i][0], &motor_upwards[i][1]);
    }
   
    fclose(fp);

    return true;
}