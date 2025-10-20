#include <cmath>
#include <cstdint>
#include <iostream>
#include <algorithm>

// =============== TAPROOT HEADERS ==================
#include "referenceHead.hpp"
#include "modm/architecture/interface/clock.hpp"
#include "drivers.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/algorithms/smooth_pid.hpp"

// =================== CONSTANTS =====================
static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;   // Hz
static constexpr float PWM_FREQUENCY       = 500.0f;   // Hz

static constexpr tap::can::CanBus CAN_BUS2 = tap::can::CanBus::CAN_BUS2; // Wheels on CAN2

// Motor IDs (change if your wiring is different)
static constexpr tap::motor::MotorId MOTOR_FL = tap::motor::MOTOR2; // Front Left
static constexpr tap::motor::MotorId MOTOR_FR = tap::motor::MOTOR3; // Front Right
static constexpr tap::motor::MotorId MOTOR_RR = tap::motor::MOTOR4; // Rear Right
static constexpr tap::motor::MotorId MOTOR_RL = tap::motor::MOTOR1; // Rear Left

// ================= DRIVE TRAIN CLASS ================
struct DriveTrain {
    tap::motor::DjiMotor &mFL, &mFR, &mRR, &mRL;
    tap::algorithms::SmoothPid pFL, pFR, pRR, pRL;

    float rpm_scale = 4000.f;  // wheel speed when joystick = 1
    float deadband  = 0.05f;   // ignore small stick noise

    DriveTrain(tap::motor::DjiMotor& fl,
               tap::motor::DjiMotor& fr,
               tap::motor::DjiMotor& rr,
               tap::motor::DjiMotor& rl,
               const tap::algorithms::SmoothPidConfig &cfg)
        : mFL(fl), mFR(fr), mRR(rr), mRL(rl),
          pFL(cfg), pFR(cfg), pRR(cfg), pRL(cfg) {}

    static float db(float v, float d) { return (std::fabs(v) < d) ? 0.f : v; }

    void command(float vx, float vy, float wz, float dt)
    {
        // 1. Apply deadband
        vx = db(vx, deadband);
        vy = db(vy, deadband);
        wz = db(wz, deadband);

        // 2. Mecanum wheel mixing (robot-oriented)
        float FL =  (vx + vy + wz);
        float FR =  (vx - vy - wz);
        float RR =  (-vx - vy + wz);
        float RL =  (-vx + vy - wz);

        // 3. Normalize so max magnitude = 1
        float maxmag = std::max({std::fabs(FL), std::fabs(FR), std::fabs(RR), std::fabs(RL), 1.f});
        FL /= maxmag; FR /= maxmag; RR /= maxmag; RL /= maxmag;

        // 4. Target RPMs
        float dFL = FL * rpm_scale;
        float dFR = FR * rpm_scale;
        float dRR = RR * rpm_scale;
        float dRL = RL * rpm_scale;

        // 5. PID speed control
        pFL.runControllerDerivateError(dFL - mFL.getShaftRPM(), dt);
        pFR.runControllerDerivateError(dFR - mFR.getShaftRPM(), dt);
        pRR.runControllerDerivateError(dRR - mRR.getShaftRPM(), dt);
        pRL.runControllerDerivateError(dRL - mRL.getShaftRPM(), dt);

        // 6. Send output to motors
        mFL.setDesiredOutput(static_cast<int32_t>(pFL.getOutput()));
        mFR.setDesiredOutput(static_cast<int32_t>(pFR.getOutput()));
        mRR.setDesiredOutput(static_cast<int32_t>(pRR.getOutput()));
        mRL.setDesiredOutput(static_cast<int32_t>(pRL.getOutput()));
    }
};

// ================= IO INITIALIZATION =================
static void initializeIo(src::Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
    drivers->remote.initialize();

    // IMU not used here, but still initialized
    drivers->mpu6500.init(MAIN_LOOP_FREQUENCY, 0.1f, 0.f);

    drivers->refSerial.initialize();
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER8, PWM_FREQUENCY);
}

// ======================= MAIN =======================
int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Starting drivetrain test..." << std::endl;
#endif

    src::Drivers *drivers = src::DoNotUse_getDrivers();
    Board::initialize();
    initializeIo(drivers);

    // Create wheel motors (check 'reversed' bools per wheel)
    tap::motor::DjiMotor mFL(drivers, MOTOR_FL, CAN_BUS2, false, "FL");
    tap::motor::DjiMotor mFR(drivers, MOTOR_FR, CAN_BUS2, true , "FR");
    tap::motor::DjiMotor mRR(drivers, MOTOR_RR, CAN_BUS2, false, "RR");
    tap::motor::DjiMotor mRL(drivers, MOTOR_RL, CAN_BUS2, true , "RL");

    mFL.initialize(); mFR.initialize(); mRR.initialize(); mRL.initialize();

    tap::communication::serial::Remote remote(drivers);
    remote.initialize();

    // PID tuning for smooth wheel RPM control
    tap::algorithms::SmoothPidConfig wheelCfg{
        .kp = 10.0f,
        .ki = 0.8f,
        .kd = 0.5f,
        .maxICumulative = 0.0f,
        .maxOutput = 8000.0f,
        .tQDerivativeKalman = 1.0f,
        .tRDerivativeKalman = 0.0f,
        .tQProportionalKalman = 1.0f,
        .tRProportionalKalman = 0.0f,
        .errDeadzone = 1.0f,
        .errorDerivativeFloor = 0.0f
    };

    DriveTrain drivetrain(mFL, mFR, mRR, mRL, wheelCfg);
    drivetrain.rpm_scale = 4000.f;
    drivetrain.deadband  = 0.05f;

    tap::arch::PeriodicMilliTimer loopTimer(1000.0f / MAIN_LOOP_FREQUENCY);

    while (true)
    {
        drivers->canRxHandler.pollCanData();
        remote.read();

        if (loopTimer.execute())
        {
            float vx = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);
            float vy = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
            float wz = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);

            const float dt = 1.0f / MAIN_LOOP_FREQUENCY;
            const float scale = 0.6f; // limit speed for testing

            drivetrain.command(vx * scale, vy * scale, wz * scale, dt);
            drivers->djiMotorTxHandler.encodeAndSendCanData();
        }

        modm::delay_us(100);
    }

    return 0;
}
