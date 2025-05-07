
#ifdef __cplusplus
extern "C" {
#endif

    #include <stdio.h>
    #include <stdint.h>

    #include "simio/simio.h"
    // #include "flight/mixer_init.h"
     
#ifdef __cplusplus
}
#endif

int main() {

    Init();
    
    uint32_t gyro_len = 3;
    uint32_t rcCommand_len = 4;
    uint32_t currentTime = 0;
    float gyro[3] = {1.2, 0, 0};
    float rcCommand[4] = {1.3, 0, 0, 8};

    pidSimulation(gyro, gyro_len, rcCommand, rcCommand_len, currentTime);
    
    float rpmCommand[NUM_PROP];
    getRpmCommand(rpmCommand, NUM_PROP);
    for (int i = 0; i < 4; i++) {
        printf("%f\t", rpmCommand[i]);
    }
    printf("\n");

    return 0;
}