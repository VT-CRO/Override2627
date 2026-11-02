#include "subsystems/drivetrain.h"
#include "config/auton.h"
#include "config/drivetrain.h"
#include "config/elevator.h"
#include "config/ports.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace cfg = config::drivetrain;

namespace {
//move current toward target, but only by rate * dt. rise rate when the voltage is growing or flipping
//direction, fall rate when it is shrinking toward 0 (same logic as the elevator's applyVoltage)
double slewVoltage(double target, double current, double riseRate, double fallRate, double dtSec){
    bool flipping {(target > 0 && current < 0) || (target < 0 && current > 0)};
    bool growing {std::abs(target) > std::abs(current)};
    double maxStep {((growing || flipping) ? riseRate : fallRate) * dtSec};
    return current + std::clamp(target - current, -maxStep, maxStep);
}

//TODO set wheel size and offsets (inches) by spinning the bot in place: offset = wheel distance / total radians.
//vertical: + is right of the tracking center. horizontal: + is in front of it
//AMT102-V pods read far more than the 360 ticks/rev LemLib assumes, so the ratio is ticks per rev / 360.
//measured: the better 300 cm roll read 157775 ticks = 8015 per rev on a 1.91 in wheel (8192 for the default 2048 PPR)
constexpr float podDiameter {1.91};
constexpr float vertTickRatio {22.27};
//horiz measured separately: the better 300 cm roll read 141265 ticks (about 10% under the vert pod's rate, unexplained)
constexpr float horizTickRatio {19.94};
//offsets from true 3600 deg spins both ways (the IMU read 3635), half the clockwise minus counterclockwise reading to cancel
//the robot walking while it spins: vert is on the centerline (under 0.3 in), horiz is 1.84 in in front
//sign flipped from the measured +1.84: a 90 deg turn in place moved the pose 5.4 in sideways (about 2 x 1.84 x the turn in radians)
constexpr float horizOffsetIn {-1.84};
}

Drivetrain::Drivetrain(pros::v5::Controller& master, const Elevator& elevator)
    : master(master),
      elevator(elevator),
      leftMotors({config::ports::leftMotorOne, config::ports::leftMotorTwo, config::ports::leftMotorThree},
                 config::ports::drivetrainGearRatio),
      rightMotors({config::ports::rightMotorOne, config::ports::rightMotorTwo, config::ports::rightMotorThree},
                  config::ports::drivetrainGearRatio),
      imu(config::ports::imu),
      //TODO set reversed so pushing the robot forward / right counts up
      vertEncoder(config::ports::vertPodTop, config::ports::vertPodBottom, false),
      horizEncoder(config::ports::horizPodTop, config::ports::horizPodBottom, false),
      vertPod(&vertEncoder, podDiameter, 0, vertTickRatio),
      horizPod(&horizEncoder, podDiameter, horizOffsetIn, horizTickRatio),
      //TODO tune
      throttleCurve(3, cfg::minOutput, 1.019),
      steerCurve(3, cfg::minOutput, 1.019),
      chassis(
          //read documentation to understand each param. TODO tune to real bot
          lemlib::Drivetrain(&leftMotors, &rightMotors, 10, lemlib::Omniwheel::NEW_325, 200, 2),
          //lateral. TODO tune
          lemlib::ControllerSettings(
              10,  //P
              0,  //I
              3,  //D
              3,
              .5,  //exits once within .5 in of the target for 100 ms
              100,
              1,   //or within 1 in for 500 ms
              500,
              20  //slew rate, max output change per 10 ms
          ),
          //angular. from the rotation sysID (hold RIGHT alone): kS 1529 mV, kV 15.23 mV per deg/s, kA 2.742 mV per deg/s^2.
          //in LemLib units (127 = 12 V, D per 10 ms loop) that is kS 16.2, kV 0.161 per deg/s, kA 0.029 per deg/s^2.
          //critically damped at natural frequency wn: P = kA wn^2, D = 100 (2 kA wn - kV). wn 20 rad/s here (was 15: P 6.53,
          //D 71; before that 9.7: P 2.74, D 40.27). kS / P (the deadband the
          //integral has to creep through) is 1.4 deg. if turnBy shows overshoot, raise D a little
          lemlib::ControllerSettings(
              11.6,  //P
              0.068,  //I, only within the windup range below, to push through friction
              115,  //D, 15% over the critically damped 100: with 100 the 90 deg test still overshot 0.5-5 deg (sensor lag)
              6,  //integral only counts within this many degrees of the target
              1,  //exits once within 1 deg for 100 ms
              100,
              3,  //or within 3 deg for 500 ms
              500,
              0  //slew off: LemLib's slew also limits braking, and at 15 the 90 deg test overshot 15-21 deg
          ),
          lemlib::OdomSensors(&vertPod, nullptr, &horizPod, nullptr, &imu),
          &throttleCurve,
          &steerCurve) {}

void Drivetrain::init(){
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.calibrate();
}

void Drivetrain::drive(){
    double leftY {master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)};
    double rightX {master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)};

    if(chassis.isInMotion()){
        bool stickMoved {std::abs(leftY) >= 10 || std::abs(rightX) >= 10};
        if(!stickMoved){
            //the auto drive owns the motors: restart the slew from 0 so the driver doesn't get a jump when it ends
            throttleVoltage = 0;
            turnVoltage = 0;
            lastDriveTime = 0;
            return;
        }
        chassis.cancelMotion();
    }

    //anti tip: the higher the elevator, the gentler the limits (0 = down, 1 = top)
    double height {std::clamp(elevator.rotations() / config::elevator::maxMotorRotations, 0.0, 1.0)};

    //top speed fades linearly from full (down) to elevatorTopSpeedScale (top), applied after the curve so it is exact
    double speedScale {1.0 - (1.0 - cfg::elevatorTopSpeedScale) * height};
    //only the part above the friction minimum is scaled, so the minimum voltage jump survives
    auto scaleAboveMin {[](double v, double scale){
        return v == 0 ? 0.0 : std::copysign(cfg::minOutput + (std::abs(v) - cfg::minOutput) * scale, v);
    }};
    double throttle {scaleAboveMin(throttleCurve.curve(leftY), speedScale)};
    double turn {scaleAboveMin(steerCurve.curve(rightX), speedScale * cfg::turnScale)};

    //time since the last loop. first call or a long gap: assume one 20 ms loop
    std::uint32_t now {pros::millis()};
    double dtSec {(lastDriveTime == 0 || now - lastDriveTime > 100) ? 0.02 : (now - lastDriveTime) / 1000.0};
    lastDriveTime = now;

    auto blend {[height](double low, double high){ return low + (high - low) * height; }};

    //stick units (-127 to 127) to millivolts (-12000 to 12000). both translation and turning are slewed
    throttleVoltage = slewVoltage(throttle * 12000.0 / 127.0, throttleVoltage,
                                  blend(cfg::voltageRiseLowMvPerSec, cfg::voltageRiseHighMvPerSec),
                                  blend(cfg::voltageFallLowMvPerSec, cfg::voltageFallHighMvPerSec), dtSec);
    turnVoltage = slewVoltage(turn * 12000.0 / 127.0, turnVoltage, cfg::turnRiseMvPerSec, cfg::turnFallMvPerSec, dtSec);

    //arcade mix into left/right, scaling both down together if one side goes past 12 V (keeps the turn ratio)
    double leftVoltage {throttleVoltage + turnVoltage};
    double rightVoltage {throttleVoltage - turnVoltage};
    double biggest {std::max(std::abs(leftVoltage), std::abs(rightVoltage))};
    if(biggest > 12000){
        leftVoltage *= 12000 / biggest;
        rightVoltage *= 12000 / biggest;
    }

    //straight to the motors (LemLib odometry still tracks the robot, it reads the encoders on its own)
    //move_voltage ignores the brake mode, so brake() only once the slew has fully ramped down to 0
    if(throttleVoltage == 0 && turnVoltage == 0){
        leftMotors.brake();
        rightMotors.brake();
    }else{
        leftMotors.move_voltage(static_cast<int>(leftVoltage));
        rightMotors.move_voltage(static_cast<int>(rightVoltage));
    }
}

void Drivetrain::turnBy(double degrees){
    float targetTheta {static_cast<float>(chassis.getPose().theta + degrees)};
    //runs in its own task, watching the turn to report overshoot for checking angularController
    pros::Task([this, targetTheta, degrees]{
        //an in-place turn shouldn't move the robot, so any change in x / y is odometry error. over four taps (360 deg)
        //the tracking center physically ends where it started, so compare the pose after the fourth to before the first
        lemlib::Pose startPose {chassis.getPose()};
        std::uint32_t start {pros::millis()};
        chassis.turnToHeading(targetTheta, 3000, {}, true);
        //first time within 3 and 1 deg of the target (how fast it gets there), 0 = never
        std::uint32_t within3Ms {0};
        std::uint32_t within1Ms {0};
        double overshoot {0};
        double peakSpeed {0};
        double last {chassis.getPose().theta};
        pros::delay(10); //let the async motion start before watching isInMotion
        while(chassis.isInMotion()){
            double theta {chassis.getPose().theta};
            double error {std::abs(targetTheta - theta)};
            if(within3Ms == 0 && error <= 3){
                within3Ms = pros::millis() - start;
            }
            if(within1Ms == 0 && error <= 1){
                within1Ms = pros::millis() - start;
            }
            //positive once it has gone past the target in the turn direction
            overshoot = std::max(overshoot, (theta - targetTheta) * (degrees > 0 ? 1.0 : -1.0));
            peakSpeed = std::max(peakSpeed, std::abs(theta - last) * 100); //10 ms loop, so deg/s
            last = theta;
            pros::delay(10);
        }
        //the exit needs it within 1 deg for 100 ms (or 3 deg for 500 ms), so done is that much after it got there
        std::uint32_t doneMs {pros::millis() - start};
        double end {chassis.getPose().theta};
        std::printf("turned to %.2f deg, target %.2f deg, error %.2f deg, overshoot %.2f deg, peak %.0f deg/s\n",
                    end, targetTheta, targetTheta - end, overshoot, peakSpeed);
        std::printf("  timing: within 3 deg at %lu ms, within 1 deg at %lu ms, done at %lu ms\n",
                    static_cast<unsigned long>(within3Ms), static_cast<unsigned long>(within1Ms),
                    static_cast<unsigned long>(doneMs));
        std::printf("  pose: (%.2f, %.2f) -> (%.2f, %.2f), moved %.2f in during the turn\n", startPose.x, startPose.y,
                    chassis.getPose().x, chassis.getPose().y, std::hypot(chassis.getPose().x - startPose.x, chassis.getPose().y - startPose.y));
    });
}

void Drivetrain::pushUntilStall(int power, std::uint32_t timeoutMs, double heading){
    chassis.cancelAllMotions();
    //hold heading with its own gentle PD: LemLib's moveToPoint steered with the turn gains, which at this low power
    //swung the robot side to side. the correction is capped at `power` so neither side ever drives backward
    lemlib::PID headingPid(config::auton::pushHeadingKp, 0, config::auton::pushHeadingKd);
    std::uint32_t start {pros::millis()};
    std::uint32_t movingAt {start};
    while(pros::millis() - start < timeoutMs){
        //a raised elevator sways more, so the hold weakens linearly from full (down) to pushHeadingTopScale (top)
        double height {std::clamp(elevator.rotations() / config::elevator::maxMotorRotations, 0.0, 1.0)};
        double strength {1.0 - (1.0 - config::auton::pushHeadingTopScale) * height};
        double turn {std::clamp(strength * headingPid.update(std::remainder(heading - chassis.getPose().theta, 360.0)),
                                -static_cast<double>(power), static_cast<double>(power))};
        leftMotors.move(static_cast<int>(power + turn));
        rightMotors.move(static_cast<int>(power - turn));
        pros::delay(10);
        std::uint32_t now {pros::millis()};
        double speed {std::max(std::abs(leftMotors.get_actual_velocity()), std::abs(rightMotors.get_actual_velocity()))};
        if(now - start < config::auton::pushGraceMs || speed >= config::auton::pushStallRpm){
            movingAt = now;
        }
        else if(now - movingAt >= config::auton::pushStallConfirmMs){
            break;
        }
    }
    leftMotors.brake();
    rightMotors.brake();
    std::printf("push: stopped after %lu ms, heading %.1f deg (held %.1f)\n",
                static_cast<unsigned long>(pros::millis() - start), chassis.getPose().theta, heading);
}
