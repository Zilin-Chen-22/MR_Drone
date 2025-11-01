HOW TO USE THE FUNCTION:

    Initial before start:
        use "Init()" without any input

    Get the result of the controller:
        use "scheduler(gyroInput, rcCommand)"
        
        function input:
            gyroInput: float gyroInput[3], no range limit
                unit: rad/s
                gyroInput[0] for Rolling angular velocity
                gyroInput[1] for Pitch angular velocity
                gyroInput[2] for Yaw angular velocity
            rcCommand: float rcCommand[4].
                unit: rad/s for first three
                rcCommand[0] for Rolling angular velocity set point
                rcCommand[1] for Pitch angular velocity set point
                rcCommand[2] for Yaw angular velocity set point
                rcCommand[3] for basic throttle set point, motor thrust [Unit: N(per motor)], 0 ~ 9
        
        function output:
            a pointer pointing to a 1*4 array: int rpm[4].
                unit: rpm
                0 to 3 represents the motor 1 to 4
                all outputs >= 0
        
        use "getRpmCommand(float *ptr, uint32_t len)"
            pointer pointing to the output

    PID Tunning:
        See figure "PIDTunning_1.png"
        In setupPID:
            pid_raw[3][5] = {
                ROLL{Proportional, Integral, Derivative, Feedforward, Dmax},
                PITCH{Proportional, Integral, Derivative, Feedforward, Dmax},
                YAW{Proportional, Integral, Derivative, Feedforward, Dmax}
            }


WHAT IS THE FUNCTION FOR:

    The function is used for simulating the pid controller for betaflight.
    The core code is copy-pasted from open source from betaflight.
    It needs an input of the set point of angular velocity and the basic throttle, and the current angular velocity measured by the imu-section.
    With these input, it gives out a signal for the motor, and automatically changes into the rpm of the fans according to the experiment data.