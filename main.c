
#include <iostream>
#include "referenceHead.hpp"
#include "modm/architecture/interface/clock.hpp"
#include "drivers.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "./subsystems/Drivetrain.hpp"   // <-- include your new drivetrain class


static constexpr float MAIN_LOOP_FREQUENCY = 500.0f; // Hz
static constexpr float PWM_FREQUENCY       = 500.0f; // Hz

// CAN bus setup
static constexpr tap::can::CanBus CAN_BUS2 = tap::can::CanBus::CAN_BUS2; // Drivetrain on CAN2

// Motor IDs (adjust if your wiring differs)
static constexpr tap::motor::MotorId MOTOR_FL = tap::motor::MOTOR2; // Front Left
static constexpr tap::motor::MotorId MOTOR_FR = tap::motor::MOTOR3; // Front Right
static constexpr tap::motor::MotorId MOTOR_RR = tap::motor::MOTOR4; // Rear  Right
static constexpr tap::motor::MotorId MOTOR_RL = tap::motor::MOTOR1; // Rear  Left

static void initializeIo(src::Drivers *drivers);

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

    // Optional IMU (not used in drivetrain test)
    drivers->mpu6500.init(MAIN_LOOP_FREQUENCY, 0.1f, 0.f);

    drivers->refSerial.initialize();
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER8, PWM_FREQUENCY);
}

// ========================
// Main Function
// ========================
int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting (drivetrain-only)..." << std::endl;
#endif

    src::Drivers *drivers = src::DoNotUse_getDrivers();
    Board::initialize();
    initializeIo(drivers);

    tap::motor::DjiMotor mFL(drivers, MOTOR_FL, CAN_BUS2, false, "FL");
    tap::motor::DjiMotor mFR(drivers, MOTOR_FR, CAN_BUS2, true , "FR");
    tap::motor::DjiMotor mRR(drivers, MOTOR_RR, CAN_BUS2, false, "RR");
    tap::motor::DjiMotor mRL(drivers, MOTOR_RL, CAN_BUS2, true , "RL");

    mFL.initialize();
    mFR.initialize();
    mRR.initialize();
    mRL.initialize();

    tap::communication::serial::Remote remote(drivers);
    remote.initialize();


    tap::algorithms::SmoothPidConfig wheelCfg(
        10, 0.8, 0.5, 0, 8000, 1, 0, 1, 0
    );

    robot::DriveTrain dt(mFL, mFR, mRR, mRL, wheelCfg);
    dt.setRpmScale(4000.f);  // typical for 3508/C620
    dt.setDeadband(0.05f);   // joystick noise filtering

    tap::arch::PeriodicMilliTimer loopTimer(1000.0f / MAIN_LOOP_FREQUENCY);

    // ========================
    // Control Loop
    // ========================
    while (true)
    {
        drivers->canRxHandler.pollCanData();
        remote.read();

        if (loopTimer.execute())
        {
            // Read controller channels
            float vx = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_VERTICAL);   // forward/back
            float vy = remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL); // strafe
            float wz = remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);// yaw

            // Scale for safe testing
            const float scale = 0.6f;  // 60% output for first run
            dt.command(vx * scale, vy * scale, wz * scale);

            // Send motor commands
            drivers->djiMotorTxHandler.encodeAndSendCanData();
        }

        // Avoid 100% CPU load
        modm::delay_us(100);
    }

    return 0;
}

