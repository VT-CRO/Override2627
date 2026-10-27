#include "drivetrain.h"
#include "constants.h"
#include "api.h"
#include "elevator.h"
#include "lemlib/api.hpp"
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <functional>

namespace drivetrain{

    pros::MotorGroup leftMotorGroup(
        {constants::leftMotorOne,
        constants::leftMotorTwo,
        constants::leftMotorThree},
        constants::drivetrainGearRatio);

    pros::MotorGroup rightMotorGroup(
        {constants::rightMotorOne, 
        constants::rightMotorTwo, 
        constants::rightMotorThree}, 
        constants::drivetrainGearRatio);

    //the IMU read 3635.2 deg over a true 3600 deg spin, so scale its angles. LemLib reads its heading through these
    class ScaledImu : public pros::Imu {
        static constexpr double scale {3600.0 / 3635.2};
    public:
        using pros::Imu::Imu;

        double get_rotation() const override {
            return pros::Imu::get_rotation() * scale;
        }

        double get_heading() const override {
            double heading {std::fmod(get_rotation(), 360.0)};
            return heading < 0 ? heading + 360.0 : heading;
        }
    };

    ScaledImu imu(constants::imu);

    void printHeading(){
        static uint32_t lastPrint {0};
        if(pros::millis() - lastPrint >= 500){
            lastPrint = pros::millis();
            lemlib::Pose pose {chassis.getPose()};
            std::printf("x %.2f y %.2f in, heading %.2f deg (raw IMU %.2f deg)\n", pose.x, pose.y, pose.theta,
                        imu.pros::Imu::get_rotation());
        }
    }

    //read documentation to understand each param
    //TODO tune to real bot 
    static lemlib::Drivetrain drivetrain(
        &leftMotorGroup, 
        &rightMotorGroup, 
        10, 
        lemlib::Omniwheel::NEW_325,
        200, 
        2
    );

    //TODO tune
    static lemlib::ControllerSettings lateralController(
        10,  //P
        0,  //I
        3,  //D
        3,
        .5,  //exits once within .5 in of the target for 100 ms
        100,
        1,   //or within 1 in for 500 ms
        500,
        20  //slew rate, max output change per 10 ms
    );

    //from the rotation sysID (hold RIGHT alone): kS 1529 mV, kV 15.23 mV per deg/s, kA 2.742 mV per deg/s^2.
    //in LemLib units (127 = 12 V, D per 10 ms loop) that is kS 16.2, kV 0.161 per deg/s, kA 0.029 per deg/s^2.
    //critically damped at natural frequency wn: P = kA wn^2, D = 100 (2 kA wn - kV). wn 20 rad/s here (was 15: P 6.53,
    //D 71; before that 9.7: P 2.74, D 40.27). kS / P (the deadband the
    //integral has to creep through) is 1.4 deg. if turnBy shows overshoot, raise D a little
    static lemlib::ControllerSettings angularController(
        11.6,  //P
        0.068,  //I, only within the windup range below, to push through friction
        115,  //D, 15% over the critically damped 100: with 100 the 90 deg test still overshot 0.5-5 deg (sensor lag)
        6,  //integral only counts within this many degrees of the target
        1,  //exits once within 1 deg for 100 ms
        100,
        3,  //or within 3 deg for 500 ms
        500,
        0  //slew off: LemLib's slew also limits braking, and at 15 the 90 deg test overshot 15-21 deg
    );

    //TODO set reversed so pushing the robot forward / right counts up
    static pros::adi::Encoder vertEncoder(constants::vertPodTop, constants::vertPodBottom, false);
    static pros::adi::Encoder horizEncoder(constants::horizPodTop, constants::horizPodBottom, false);

    //TODO set wheel size and offsets (inches) by spinning the bot in place: offset = wheel distance / total radians.
    //vertical: + is right of the tracking center. horizontal: + is in front of it
    //AMT102-V pods read far more than the 360 ticks/rev LemLib assumes, so the ratio is ticks per rev / 360.
    //measured: the better 300 cm roll read 157775 ticks = 8015 per rev on a 1.91 in wheel (8192 for the default 2048 PPR)
    constexpr float podDiameter {1.91};
    constexpr float vertTickRatio {22.27};
    //horiz measured separately: the better 300 cm roll read 141265 ticks (about 10% under the vert pod's rate, unexplained)
    constexpr float horizTickRatio {19.94};
    static lemlib::TrackingWheel vertPod(&vertEncoder, podDiameter, 0, vertTickRatio);
    //offsets from true 3600 deg spins both ways (the IMU read 3635), half the clockwise minus counterclockwise reading to cancel
    //the robot walking while it spins: vert is on the centerline (under 0.3 in), horiz is 1.84 in in front
    //sign flipped from the measured +1.84: a 90 deg turn in place moved the pose 5.4 in sideways (about 2 x 1.84 x the turn in radians)
    static lemlib::TrackingWheel horizPod(&horizEncoder, podDiameter, -1.84, horizTickRatio);

    static lemlib::OdomSensors sensors(
        &vertPod,
        nullptr,
        &horizPod,
        nullptr,
        &imu
    );

    //TODO tune
    lemlib::ExpoDriveCurve throttleCurve(3, constants::driveMinOutput, 1.019);
    lemlib::ExpoDriveCurve steerCurve(3, constants::driveMinOutput, 1.019);

    lemlib::Chassis chassis(
        drivetrain,
        lateralController,
        angularController,
        sensors,
        &throttleCurve,
        &steerCurve
    );

    //slew state for lemlibArcade: the forward/back voltage (mV) after slewing, and when we last ran
    static double throttleVoltage {0};
    static double turnVoltage {0};
    static std::uint32_t lastDriveTime {0};

    //move current toward target, but only by rate * dt. rise rate when the voltage is growing or flipping
    //direction, fall rate when it is shrinking toward 0 (same logic as the elevator's applyVoltage)
    static double slewVoltage(double target, double current, double riseRate, double fallRate, double dtSec){
        bool flipping {(target > 0 && current < 0) || (target < 0 && current > 0)};
        bool growing {std::abs(target) > std::abs(current)};
        double maxStep {((growing || flipping) ? riseRate : fallRate) * dtSec};
        return current + std::clamp(target - current, -maxStep, maxStep);
    }

    void lemlibArcade(){
        double maxElevatorRotations {constants::elevatorMaxMotorRotations};
        double currentElevatorRotations {elevator::getElevatorRotations()};

        double leftY {constants::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)};
        double rightX {constants::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)};

        //anti tip: the higher the elevator, the gentler the limits (0 = down, 1 = top)
        double height {std::clamp(currentElevatorRotations / maxElevatorRotations, 0.0, 1.0)};

        //top speed fades linearly from full (down) to elevatorTopSpeedScale (top), applied after the curve so it is exact
        double speedScale {1.0 - (1.0 - constants::elevatorTopSpeedScale) * height};
        //only the part above the friction minimum is scaled, so the minimum voltage jump survives
        auto scaleAboveMin {[](double v, double scale){
            return v == 0 ? 0.0 : std::copysign(constants::driveMinOutput + (std::abs(v) - constants::driveMinOutput) * scale, v);
        }};
        double throttle {scaleAboveMin(throttleCurve.curve(leftY), speedScale)};
        double turn {scaleAboveMin(steerCurve.curve(rightX), speedScale * constants::driveTurnScale)};

        //time since the last loop. first call or a long gap: assume one 20 ms loop
        std::uint32_t now {pros::millis()};
        double dtSec {(lastDriveTime == 0 || now - lastDriveTime > 100) ? 0.02 : (now - lastDriveTime) / 1000.0};
        lastDriveTime = now;

        auto blend {[height](double low, double high){ return low + (high - low) * height; }};

        //stick units (-127 to 127) to millivolts (-12000 to 12000). both translation and turning are slewed
        throttleVoltage = slewVoltage(throttle * 12000.0 / 127.0, throttleVoltage,
                                      blend(constants::driveVoltageRiseLowMvPerSec, constants::driveVoltageRiseHighMvPerSec),
                                      blend(constants::driveVoltageFallLowMvPerSec, constants::driveVoltageFallHighMvPerSec), dtSec);
        turnVoltage = slewVoltage(turn * 12000.0 / 127.0, turnVoltage,
                                  constants::driveTurnRiseMvPerSec, constants::driveTurnFallMvPerSec, dtSec);

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
            leftMotorGroup.brake();
            rightMotorGroup.brake();
        }else{
            leftMotorGroup.move_voltage(static_cast<int>(leftVoltage));
            rightMotorGroup.move_voltage(static_cast<int>(rightVoltage));
        }
    }

    void driveForward(double inches){
        lemlib::Pose pose {chassis.getPose()};
        //LemLib headings are degrees with 0 along +y, clockwise positive
        double heading {pose.theta * M_PI / 180.0};
        float targetX {static_cast<float>(pose.x + inches * std::sin(heading))};
        float targetY {static_cast<float>(pose.y + inches * std::cos(heading))};
        float targetTheta {static_cast<float>(pose.theta)};
        //blocks until the motion ends, so run it in its own task and print how close it got.
        //moveToPose with the starting heading as the target heading holds that IMU heading the whole way, so drift gets corrected
        pros::Task([targetX, targetY, targetTheta]{
            chassis.moveToPose(targetX, targetY, targetTheta, 5000, {}, false);
            lemlib::Pose end {chassis.getPose()};
            std::printf("drove to x %.2f y %.2f heading %.1f deg, target x %.2f y %.2f, error %.2f in\n",
                        end.x, end.y, end.theta, targetX, targetY, std::hypot(targetX - end.x, targetY - end.y));
        });
    }

    void turnBy(double degrees){
        float targetTheta {static_cast<float>(chassis.getPose().theta + degrees)};
        //runs in its own task like driveForward, watching the turn to report overshoot for checking angularController
        pros::Task([targetTheta, degrees]{
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

    void pushUntilStall(int power, std::uint32_t timeoutMs, double heading){
        chassis.cancelAllMotions();
        //hold heading with its own gentle PD: LemLib's moveToPoint steered with the turn gains, which at this low power
        //swung the robot side to side. the correction is capped at `power` so neither side ever drives backward
        lemlib::PID headingPid(constants::autonPushHeadingKp, 0, constants::autonPushHeadingKd);
        std::uint32_t start {pros::millis()};
        std::uint32_t movingAt {start};
        while(pros::millis() - start < timeoutMs){
            //a raised elevator sways more, so the hold weakens linearly from full (down) to autonPushHeadingTopScale (top)
            double height {std::clamp(elevator::getElevatorRotations() / constants::elevatorMaxMotorRotations, 0.0, 1.0)};
            double strength {1.0 - (1.0 - constants::autonPushHeadingTopScale) * height};
            double turn {std::clamp(strength * headingPid.update(std::remainder(heading - chassis.getPose().theta, 360.0)),
                                    -static_cast<double>(power), static_cast<double>(power))};
            leftMotorGroup.move(static_cast<int>(power + turn));
            rightMotorGroup.move(static_cast<int>(power - turn));
            pros::delay(10);
            std::uint32_t now {pros::millis()};
            double speed {std::max(std::abs(leftMotorGroup.get_actual_velocity()), std::abs(rightMotorGroup.get_actual_velocity()))};
            if(now - start < constants::autonPushGraceMs || speed >= constants::autonPushStallRpm){
                movingAt = now;
            }
            else if(now - movingAt >= constants::autonPushStallConfirmMs){
                break;
            }
        }
        leftMotorGroup.brake();
        rightMotorGroup.brake();
        std::printf("push: stopped after %lu ms, heading %.1f deg (held %.1f)\n",
                    static_cast<unsigned long>(pros::millis() - start), chassis.getPose().theta, heading);
    }

    void drive(){
        if(chassis.isInMotion()){
            bool stickMoved {std::abs(constants::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) >= 10
                          || std::abs(constants::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) >= 10};
            if(!stickMoved){
                //the auto drive owns the motors: restart the slew from 0 so the driver doesn't get a jump when it ends
                throttleVoltage = 0;
                turnVoltage = 0;
                lastDriveTime = 0;
                return;
            }
            chassis.cancelMotion();
        }
        lemlibArcade();
    }

    void init(){
        leftMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        rightMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        chassis.calibrate();
    }
}