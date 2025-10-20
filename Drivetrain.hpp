#ifndef ROBOT_DRIVETRAIN_HPP
#define ROBOT_DRIVETRAIN_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "tap/motor/dji_motor.hpp"
#include "./tap/algorithms/smooth_pid.hpp"

namespace robot {

class DriveTrain {
public:
    DriveTrain(tap::motor::DjiMotor& fl,
               tap::motor::DjiMotor& fr,
               tap::motor::DjiMotor& rr,
               tap::motor::DjiMotor& rl,
               const tap::algorithms::SmoothPidConfig& pidCfg);

    void command(float vx, float vy, float wz);

    void setRpmScale(float s)  { rpm_scale_ = s; }
    void setDeadband(float d)  { deadband_  = d; }

    float rpmScale() const { return rpm_scale_; }
    float deadband() const { return deadband_; }

private:
    static float db(float v, float d) { return (std::fabs(v) < d) ? 0.0f : v; }

    tap::motor::DjiMotor &mFL_, &mFR_, &mRR_, &mRL_;
    tap::algorithms::SmoothPid pFL_, pFR_, pRR_, pRL_;

    float rpm_scale_ = 4000.0f;
    float deadband_  = 0.03f;
};

} // namespace robot

#endif // ROBOT_DRIVETRAIN_HPP
