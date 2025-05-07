#include <iostream>
#include <algorithm>
#include <array>
#include "Model.h"
// #include "quadrotor_model.h"
#include "RigidBody.h"
// #include "quad_pos_controller.h"
#include "basic_pid.h"
#include "io_proc.h"
// get the net torque on the drone

const uint32_t numOfProp = 4u;
const uint32_t dimOfTorque = 3u;
const double quad_mass = 0.837;
// static thrust curve parameter
const double kt = 1.4444916466735111e-08;
const double kq = 0.014048128160288385;
// quadrotor arm distance
const double lf = 179.376 / 1000.0;  // m
const double lr = 139.476 / 1000.0;  // m

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
    std::cout << torqueRef << '\n';

    rpm[0] = 12000;
    rpm[1] = 8000;
    rpm[2] = 11000;
    rpm[3] = 9000;
    GetNetTorque(torqueDir, spd, rpm, torqueRef);
    std::cout << torqueRef << '\n';

    rpm[0] = 9000;
    rpm[1] = 11000;
    rpm[2] = 7000;
    rpm[3] = 15000;
    GetNetTorque(torqueDir, spd, rpm, torqueRef);
    std::cout << torqueRef << '\n';

    // the time step is 500us
    double timeStep = 500e-6;  // uint: s
    // define solver
    Mdss::SolverConfig config1;
    config1.eposilon = 0.00001;
    config1.adaptive_step = false;
    config1.frame_step = timeStep;
    config1.mim_step = timeStep / 10.0;
    config1.start_time = 0.0;
    config1.solver_type = RungeKuttaFamily::DORMANDPRINCE;
    config1.loggingconfig.filename = "pid_test";
    config1.loggingconfig.uselogging = false;
    config1.loglevel = Mdss::LOGLEVEL_ERROR;
    Mdss::Model model(config1);

    // define rigid-body block
    dynamics::RigidBodyParameter rigid_1;
    dynamics::RigidBodyCondition rigid_1_IC;

    rigid_1.J << 2.27e-3, 8.05e-5, -4.36e-4, 8.05e-5, 3.56e-3, -5.38e-4, -4.36e-4, -5.38e-4, 3.732e-4;

    rigid_1.m = 1;

    rigid_1_IC.Euler.setZero();
    rigid_1_IC.Omega_BI.setZero();
    rigid_1_IC.V_I.setZero();
    rigid_1_IC.X_I.setZero();

    auto rigid_body_1 = model.AddSubsystem(dynamics::CreateRigidBodyBlock(rigid_1, rigid_1_IC));

    bool flag = model.Compile();

    const uint32_t N_steps = 500;

    if (!flag) {
        // if unsuccessful, run updates
        std::cout << "pre process failed! check subsystem connections \n";
        return 1;
    }

    BasicPIDParam param;
    param.param[ATTL_X].Kd = 0.5;
    param.param[ATTL_Y].Kd = 0.5;
    param.param[ATTL_Z].Kd = 0.5;

    // setting up betafly controller.
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
    rcCommand[THROTTLE] = 6.0;

    // initialize the recorder
    FileIO::WriteBinary<double, 1024> dataLogger;
    dataLogger.Open("adv_pid_test.dat", std::ios::trunc);
    // write the number of time steps:
    dataLogger.WriteToBuffer(static_cast<double>(N_steps));
    // write the number of entries
    // gyro, rpm command, torque
    dataLogger.WriteToBuffer(static_cast<double>(1 + 3 + 3 + 3));

    BasicPIDInit(&param);
    BasicPIDInput pidInput;
    BasicPIDCom pidCom;
    BasicPIDOutput output;
    BasicPIDStates states;
    pidCom.augularCom[ATTL_X] = 0.3;
    pidCom.augularCom[ATTL_Y] = 0.0;
    pidCom.augularCom[ATTL_Z] = 0.0;
    for (uint32_t i = 0; i < N_steps; i++) {
        // record sim time
        dataLogger.WriteToBuffer(model.Run_GetSystemTime());
        // set rc command
        // rcCommand[X] = 0.3;
        // get output feedback
        for (uint16_t i = 0; i < NUM_OF_CHANNEL; i++) {
            pidInput.augularVel[i] = model.GetSubsystemOutput(
                rigid_body_1, dynamics::RigidbodyOutput::RIGIDBODY_OUTPUT_omega_BIx + i);
        }

        BasicPIDUpdate(&pidInput, &pidCom);

        for (uint32_t i = 0; i < 3; i++) {
            // std::cout << gyro[i] << ", ";
            dataLogger.WriteToBuffer(pidInput.augularVel[i]);
        }

        BasicPIDGetOutput(&output);
        for (uint32_t i = 0; i < 3; i++) {
            torqueRef(i) = output.tau[i];
        }

        for (uint32_t i = 0; i < 3; i++) {
            dataLogger.WriteToBuffer(torqueRef(i));
        }
        BasicPIDGetDebugInfo(&states);
        // the debug
        for (uint32_t i = 0; i < 3; i++) {
            dataLogger.WriteToBuffer(states.err[i]);
        }

        // modify the external
        // update system
        extern_input(dynamics::RigidbodyInput::RIGIDBODY_INPUT_TBx) = torqueRef(0);
        extern_input(dynamics::RigidbodyInput::RIGIDBODY_INPUT_TBy) = torqueRef(1);
        extern_input(dynamics::RigidbodyInput::RIGIDBODY_INPUT_TBz) = torqueRef(2);
        model.Run_Update(extern_input);
        // std::cout << "-----\n";
    }
    // add a rigid body
    model.PostRunProcess();
    dataLogger.FlushBuffer();
    dataLogger.Close();
    return 0;
}