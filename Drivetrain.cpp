

#include "./subsystems/Drivetrain.hpp"

namespace robot {

DriveTrain::DriveTrain(tap::motor::DjiMotor& fl,
                       tap::motor::DjiMotor& fr,
                       tap::motor::DjiMotor& rr,
                       tap::motor::DjiMotor& rl,
                       const tap::algorithms::SmoothPidConfig& pidCfg)
    : mFL_(fl), mFR_(fr), mRR_(rr), mRL_(rl),
      pFL_(pidCfg), pFR_(pidCfg), pRR_(pidCfg), pRL_(pidCfg)
{}

void DriveTrain::command(float vx, float vy, float wz)
{
    vx = db(vx, deadband_);
    vy = db(vy, deadband_);
    wz = db(wz, deadband_);

    float FL =  (vx + vy + wz);
    float FR =  (vx - vy - wz);
    float RR =  (-vx - vy + wz);
    float RL =  (-vx + vy - wz);

    float maxmag = std::max({std::fabs(FL), std::fabs(FR), std::fabs(RR), std::fabs(RL), 1.0f});
    FL /= maxmag; FR /= maxmag; RR /= maxmag; RL /= maxmag;

    const float dFL = FL * rpm_scale_;
    const float dFR = FR * rpm_scale_;
    const float dRR = RR * rpm_scale_;
    const float dRL = RL * rpm_scale_;

    pFL_.runControllerDerivateError(dFL - mFL_.getShaftRPM(), 1);
    pFR_.runControllerDerivateError(dFR - mFR_.getShaftRPM(), 1);
    pRR_.runControllerDerivateError(dRR - mRR_.getShaftRPM(), 1);
    pRL_.runControllerDerivateError(dRL - mRL_.getShaftRPM(), 1);

    mFL_.setDesiredOutput(static_cast<int32_t>(pFL_.getOutput()));
    mFR_.setDesiredOutput(static_cast<int32_t>(pFR_.getOutput()));
    mRR_.setDesiredOutput(static_cast<int32_t>(pRR_.getOutput()));
    mRL_.setDesiredOutput(static_cast<int32_t>(pRL_.getOutput()));
}

} // namespace robot
