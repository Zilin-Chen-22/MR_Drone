#include <iostream>
#include <algorithm>
#include <array>
#include <cmath>
#include "Model.h"
// #include "quadrotor_model.h"
#include "RigidBody.h"
// #include "quad_pos_controller.h"
#ifdef __cplusplus
extern "C" {
#endif

    #include "simio/simio.h"

#ifdef __cplusplus
}
#endif

// #define ORIGINAL_DRONE

#include "io_proc.h"
// get the net torque on the drone

const uint32_t numOfProp = 4u;
const uint32_t dimOfTorque = 3u;

#ifdef ORIGINAL_DRONE
const double quad_mass = 0.837;

// static thrust curve parameter
const double kt = 1.4444916466735111e-08;
const double kq = 0.014048128160288385;

// quadrotor arm distance
const double lf = 179.376 / 1000.0;  // m
const double lr = 139.476 / 1000.0;  // m
#else
const double quad_mass = 0.0239695;
const double kt = 0.78264e-4;
const double kq = 0.0107;
const double lf = 0.0961665;  // m
const double lr = 0.0961665;  // m
#endif

using QuadArm = std::array<Eigen::Vector3d, numOfProp>;
using TorqueDir = std::array<Eigen::Vector3d, numOfProp>;
struct PropLiftTorque {
    double lift{0.0};
    double torque{0.0};
};

// get the direction of the torque
TorqueDir GetTorqueDir(const QuadArm &arm)
{
    /*
    td = list()
    for x in arm:
        td.append(np.cross(x, np.array([0, 0, 1])))
    */
    Eigen::Vector3d zAxis;
    zAxis.setZero();
    zAxis(2) = 1.0;
    TorqueDir res;
    for (uint16_t i = 0; i < numOfProp; i++) {
        res[i] = arm[i].cross(zAxis);
    }
    return res;
}

// get static thrust and torque of a propeller
PropLiftTorque GetPropStaticLiftTorque(double rpm)
{
    PropLiftTorque res;
    res.lift = kt * rpm * rpm;
    res.torque = kq * res.lift;
    return res;
}

void GetNetTorque(const TorqueDir &torqueDir,                // torque direction
                  const std::array<double, numOfProp> &spd,  // spin direction
                  const std::array<double, numOfProp> &rpm,  // rpm
                  Eigen::Ref<Vector3d> &torque)              // net torque
{
    // from python code
    // get_net_lift_torque(t, q, td, spd):
    //  f = 0.0
    // tau = 0.0
    // for i, x in enumerate(t):
    //     f += x
    //     tau += x * td[i] + q[i] * spd[i]
    // return f, tau
    // reset torque to zero
    torque.setZero();
    Eigen::Vector3d zAxis;  // to be optimized later
    zAxis.setZero();
    zAxis(2) = 1.0;
    for (uint16_t i = 0; i < numOfProp; i++) {
        // get lift and torque
        auto liftTorque = GetPropStaticLiftTorque(rpm[i]);
        torque += liftTorque.lift * torqueDir[i] + liftTorque.torque * zAxis * spd[i];
    }
}

void GetNetLift(const TorqueDir &torqueDir,                // torque direction
    const std::array<double, numOfProp> &spd,  // spin direction
    const std::array<double, numOfProp> &rpm,  // rpm
    double &lift)              // net torque
{
// from python code
// get_net_lift_torque(t, q, td, spd):
//  f = 0.0
// tau = 0.0
// for i, x in enumerate(t):
//     f += x
//     tau += x * td[i] + q[i] * spd[i]
// return f, tau
// reset torque to zero
lift = 0.0;
// Eigen::Vector3d zAxis;  // to be optimized later
// zAxis.setZero();
// zAxis(2) = 1.0;
for (uint16_t i = 0; i < numOfProp; i++) {
// get lift and torque
auto liftTorque = GetPropStaticLiftTorque(rpm[i]);
lift += liftTorque.lift;
}
}

int main(void)
{
    // initialize the arm position
    QuadArm arm;

    arm[0] << -lr / 2.0, -lf / 2.0, 0.0;
    arm[1] << lr / 2.0, -lf / 2.0, 0.0;
    arm[2] << -lr / 2.0, lf / 2.0, 0.0;
    arm[3] << lr / 2.0, lf / 2.0, 0.0;

    const std::array<double, numOfProp> spd{-1.0, 1.0, 1.0, -1.0};
    std::array<double, numOfProp> rpm{10000, 14000, 9000, 8000};
    auto torqueDir = GetTorqueDir(arm);

    // make some tests
    Eigen::Vector3d torque;
    Eigen::Ref<Vector3d> torqueRef(torque);
    GetNetTorque(torqueDir, spd, rpm, torqueRef);
    // std::cout << torqueRef << '\n';

    double lift = 0.0;

    rpm[0] = 12000;
    rpm[1] = 8000;
    rpm[2] = 11000;
    rpm[3] = 9000;
    GetNetTorque(torqueDir, spd, rpm, torqueRef);
    // std::cout << torqueRef << '\n';

    rpm[0] = 9000;
    rpm[1] = 11000;
    rpm[2] = 7000;
    rpm[3] = 15000;
    GetNetTorque(torqueDir, spd, rpm, torqueRef);
    // std::cout << torqueRef << '\n';

    // control update time step
    const double controlUpdateRate = 400;                    // Hz
    const double controlTimeStep = 1.0 / controlUpdateRate;  // unit s
    // simulation time step
    const uint32_t simToConTRatio = 20.0;  // ratio between sim and control update
    const double simTimeStep = controlTimeStep / simToConTRatio;
    // define solver
    Mdss::SolverConfig config1;
    config1.eposilon = 0.00001;
    config1.adaptive_step = false;
    config1.frame_step = simTimeStep;
    config1.mim_step = simTimeStep / 10.0;
    config1.start_time = 0.0;
    config1.solver_type = RungeKuttaFamily::DORMANDPRINCE;
    config1.loggingconfig.filename = "pid_test";
    config1.loggingconfig.uselogging = false;
    config1.loglevel = Mdss::LOGLEVEL_ERROR;
    Mdss::Model model(config1);

    // define rigid-body block
    dynamics::RigidBodyParameter rigid_1;
    dynamics::RigidBodyCondition rigid_1_IC;

    #ifdef ORIGINAL_DRONE
    rigid_1.J << 2.27e-3, 8.05e-5, -4.36e-4, 8.05e-5, 3.56e-3, -5.38e-4, -4.36e-4, -5.38e-4, 3.732e-4;
    #else
    rigid_1.J << 720538.6335e-9, 11143.5293e-9, 10880.7758e-9, 11143.5293e-9, 721181.4959e-9, 249488.9285e-9, 10880.7758e-9, 249488.9285e-9, 435438.3069e-9;
    #endif

    rigid_1.m = quad_mass;

    rigid_1_IC.Euler.setZero();
    rigid_1_IC.Omega_BI.setZero();
    rigid_1_IC.V_I.setZero();
    rigid_1_IC.X_I.setZero();

    auto rigid_body_1 = model.AddSubsystem(dynamics::CreateRigidBodyBlock(rigid_1, rigid_1_IC));

    bool flag = model.Compile();

    const double simulationTime = 10.0;
    const uint32_t N_steps = static_cast<uint32_t>(ceil(simulationTime / simTimeStep));
    std::cout << "simulation step: " << N_steps << '\n';

    if (!flag) {
        // if unsuccessful, run updates
        std::cout << "pre process failed! check subsystem connections \n";
        return 1;
    }

    // setting up betafly controller.
    Init();
    const uint32_t gyro_len = 3;
    const uint32_t rcCommand_len = 4;
    const uint32_t rpmCommandLen = numOfProp;
    VectorXd extern_input;
    model.ReshapeExternalInputVector(extern_input);
    // initialize gyro input
    float gyro[3] = {0.0, 0.0, 0.0};
    float rpmCom[numOfProp] = {0.0, 0.0, 0.0};
    // initailize rcCommand input
    float rcCommand[numOfProp] = {0.0, 0.0, 0.0, 0.0};
    double p = 0.0;
    double q = 0.0;
    double r = 0.0;
    rcCommand[THROTTLE] = 100;  // From 1 - 100 percent

    float Vz = 0.0;
    float Az = 0.0;

    // drone posture
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    float posture[3] = {0.0, 0.0, 0.0};

    // initialize the recorder
    FileIO::WriteBinary<double, 1024> dataLogger;
    dataLogger.Open("betaflight_test.dat", std::ios::trunc);
    // write the number of time steps:
    dataLogger.WriteToBuffer(static_cast<double>(N_steps));
    // write the number of entries
    // gyro, rpm command, torque, rcCommand
    dataLogger.WriteToBuffer(static_cast<double>(1 + 3 + numOfProp + 3 + 3 + 3 + 2));

    int frequency = 5;

    for (uint32_t i = 0; i < N_steps; i++) {
        // record sim time
        dataLogger.WriteToBuffer(model.Run_GetSystemTime());
        // set rc command
        
        rcCommand[Z] = 0.5 * sinf(2 * M_PIf * frequency * i / N_steps);
        rcCommand[X] = 0.0;
        // rcCommand[X] = 0.2 * cosf(2 * M_PIf * frequency * i / N_steps);
        // rcCommand[Z] = 0.2 * sinf(2 * M_PIf * frequency * (i + N_steps / 4) / N_steps);
        // rcCommand[X] = (int(i / 1600) + 1) * 0.1;
        rcCommand[Y] = 0;
        // rcCommand[Z] = 0;

        // get output feedback
        p = model.GetSubsystemOutput(rigid_body_1, dynamics::RigidbodyOutput::RIGIDBODY_OUTPUT_omega_BIx);
        q = model.GetSubsystemOutput(rigid_body_1, dynamics::RigidbodyOutput::RIGIDBODY_OUTPUT_omega_BIy);
        r = model.GetSubsystemOutput(rigid_body_1, dynamics::RigidbodyOutput::RIGIDBODY_OUTPUT_omega_BIz);

        // todo: Get z Velocity, and test if can stay stable RIGIDBODY_OUTPUT_VIz
        // z_velocity = model.GetSubsystemOutput(rigid_body_1, dynamics::RigidbodyOutput::RIGIDBODY_OUTPUT_omega_BIz);
        Az = float(model.GetSubsystemOutput(rigid_body_1, dynamics::RigidbodyOutput::RIGIDBODY_OUTPUT_VBz));
        Vz = float(model.GetSubsystemOutput(rigid_body_1, dynamics::RigidbodyOutput::RIGIDBODY_OUTPUT_XIx));
        
        
        gyro[0] = p;
        gyro[1] = q;
        gyro[2] = -r;  // ?
                      // display gyro

        // calculate drone posture
        roll = roll + p * simTimeStep;
        pitch = pitch + q * simTimeStep;
        yaw = yaw + r * simTimeStep;
        posture[0] = float(roll);
        posture[1] = float(pitch);
        posture[2] = float(yaw);
        // make sure angle range, between -pi to pi
        for (uint32_t i = 0; i < 3; i++) {
            while (true) {
                if (posture[i] < -M_PIf) {
                    posture[i] += (M_PIf * 2);
                }
                else if (posture[i] > M_PIf) {
                    posture[i] -= (M_PIf * 2);
                }
                else {
                    break;
                }
            }
        }

        // std::cout << "gyro: \n";
        for (uint32_t i = 0; i < 3; i++) {
            // std::cout << gyro[i] << ", ";
            dataLogger.WriteToBuffer(gyro[i]);
        }
        if (i % simToConTRatio == 0) {
            // update controller accroding to sim/control ratio
            postureController(posture, gyro, gyro_len, rcCommand, rcCommand_len, uint32_t(i / simTimeStep * 1000000));
            // pidSimulation(gyro, gyro_len, rcCommand, rcCommand_len, uint32_t(i / simTimeStep * 1000000));
            // get rpm command
            getRpmCommand(rpmCom, numOfProp);
        }

        // show the rpm command
        for (uint32_t i = 0; i < numOfProp; i++) {
            dataLogger.WriteToBuffer(rpmCom[i]);
        }
        // calculate total thrust and torque
        std::copy(rpmCom, rpmCom + numOfProp, rpm.begin());
        GetNetTorque(torqueDir, spd, rpm, torqueRef);
        GetNetLift(torqueDir, spd, rpm, lift);
        if ( i == N_steps - 1 ) {
            std::cout << torqueRef << std::endl;
        }
        for (uint32_t i = 0; i < 3; i++) {
            dataLogger.WriteToBuffer(torqueRef(i));
        }
        extern_input(dynamics::RigidbodyInput::RIGIDBODY_INPUT_TBx) = torqueRef(0);
        extern_input(dynamics::RigidbodyInput::RIGIDBODY_INPUT_TBy) = torqueRef(1);
        extern_input(dynamics::RigidbodyInput::RIGIDBODY_INPUT_TBz) = torqueRef(2);
        // extern_input(dynamics::RigidbodyInput::RIGIDBODY_INPUT_FBz) = lift;
        model.Run_Update(extern_input);

        // display rcCommand
        for (uint32_t i = 0; i < 3; i++) {
            // std::cout << gyro[i] << ", ";
            dataLogger.WriteToBuffer(rcCommand[i]);
        }

        for (uint32_t i = 0; i < 3; i++) {
            dataLogger.WriteToBuffer(posture[i]);
        }

        dataLogger.WriteToBuffer(Vz);
        dataLogger.WriteToBuffer(Az);
    }

    // add a rigid body
    model.PostRunProcess();
    dataLogger.FlushBuffer();
    dataLogger.Close();
    return 0;
}