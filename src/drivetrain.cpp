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
        5,  //P
        .3,  //I, builds up to push through friction when it stalls just short. only counts within 3 in of the target
        8,  //D
        3,
        .5,  //exits once within .5 in of the target for 100 ms
        100,
        1,   //or within 1 in for 500 ms
        500,
        .1  //slew rate
    );

    //from the rotation sysID (hold RIGHT alone): kS 1529 mV, kV 15.23 mV per deg/s, kA 2.742 mV per deg/s^2
    static lemlib::ControllerSettings angularController(
        2.74,  //P
        0.068,  //I, only within the windup range below, to push through friction
        40.27,  //D
        6,  //integral only counts within this many degrees of the target
        1,  //exits once within 1 deg for 100 ms
        100,
        3,  //or within 3 deg for 500 ms
        500,
        0  //slew off, the gains already assume the motors' own response
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
            chassis.turnToHeading(targetTheta, 3000, {}, true);
            pros::delay(20); //let the async motion start before watching isInMotion
            double overshoot {0};
            while(chassis.isInMotion()){
                //positive once it has gone past the target in the turn direction
                overshoot = std::max(overshoot, (chassis.getPose().theta - targetTheta) * (degrees > 0 ? 1.0 : -1.0));
                pros::delay(10);
            }
            double end {chassis.getPose().theta};
            std::printf("turned to %.2f deg, target %.2f deg, error %.2f deg, overshoot %.2f deg\n",
                        end, targetTheta, targetTheta - end, overshoot);
        });
    }

    //where a trapezoidal profile over `distance` is t seconds in: speeds up at accel, cruises at maxSpeed, slows down at
    //accel. moves too short to reach maxSpeed become a triangle. position and speed carry the sign of distance
    struct ProfilePoint{
        double position;
        double speed;
        bool done;
    };
    static ProfilePoint trapezoid(double distance, double maxSpeed, double accel, double t){
        double d {std::abs(distance)};
        double peak {std::min(maxSpeed, std::sqrt(d * accel))};
        if(peak <= 0){
            return {0, 0, true};
        }
        double accelTime {peak / accel};
        double accelDist {0.5 * peak * accelTime};
        double cruiseTime {(d - 2 * accelDist) / peak};
        double position {d};
        double speed {0};
        if(t < accelTime){
            position = 0.5 * accel * t * t;
            speed = accel * t;
        }
        else if(t < accelTime + cruiseTime){
            position = accelDist + peak * (t - accelTime);
            speed = peak;
        }
        else if(t < 2 * accelTime + cruiseTime){
            double slowing {t - accelTime - cruiseTime};
            position = d - accelDist + peak * slowing - 0.5 * accel * slowing * slowing;
            speed = peak - accel * slowing;
        }
        double sign {distance < 0 ? -1.0 : 1.0};
        return {sign * position, sign * speed, t >= 2 * accelTime + cruiseTime};
    }

    //a forward drive that cruises at vFast, slows smoothly to vSlow so the last slowIn inches run at vSlow, then slows to a
    //stop. needs d >= vFast^2/2a + (vFast^2 - vSlow^2)/2a + slowIn, and slowIn >= vSlow^2/2a
    static ProfilePoint fastThenSlow(double d, double vFast, double vSlow, double slowIn, double accel, double t){
        double d1 {vFast * vFast / (2 * accel)};
        double d3 {(vFast * vFast - vSlow * vSlow) / (2 * accel)};
        double d4 {slowIn - vSlow * vSlow / (2 * accel)};
        double d2 {d - d1 - d3 - slowIn};
        double t1 {vFast / accel};
        double t2 {d2 / vFast};
        double t3 {(vFast - vSlow) / accel};
        double t4 {d4 / vSlow};
        double t5 {vSlow / accel};
        if(t < t1){
            return {0.5 * accel * t * t, accel * t, false};
        }
        double p {d1};
        t -= t1;
        if(t < t2){
            return {p + vFast * t, vFast, false};
        }
        p += d2;
        t -= t2;
        if(t < t3){
            return {p + vFast * t - 0.5 * accel * t * t, vFast - accel * t, false};
        }
        p += d3;
        t -= t3;
        if(t < t4){
            return {p + vSlow * t, vSlow, false};
        }
        p += d4;
        t -= t4;
        if(t < t5){
            return {p + vSlow * t - 0.5 * accel * t * t, vSlow - accel * t, false};
        }
        return {d, 0, true};
    }

    //output that holds a profile speed: the friction minimum plus speed times kV, out of 127
    static double feedforward(double speed, double kS, double kV){
        return speed == 0 ? 0.0 : std::copysign(kS, speed) + speed * kV;
    }

    //runs one profiled motion: each 10 ms `measure` gives progress along the profile, the lateral or angular gains correct
    //the gap to the profile, and `output` sends the result (plus the profile feedforward) to the motors. ends once the
    //profile is over and the robot has stayed within `tolerance` of the end for 100 ms, or 1.5 s after the profile ends.
    //the integral only builds once the profile is over (to push through friction at the end): while chasing a moving
    //target the error sits inside the windup range the whole move, so it would wind up and surge the robot past the end.
    //prints how far it got and its top speed, to check kV and the max speed
    template<typename Measure, typename Output>
    static void followProfile(double distance, double maxSpeed, double accel, double kS, double kV, const lemlib::ControllerSettings& gains,
                              Measure measure, Output output, std::function<ProfilePoint(double)> customProfile = nullptr,
                              double toleranceOverride = 0){
        lemlib::PID pd(gains.kP, 0, gains.kD);
        double integral {0};
        double topSpeed {0};
        //speed is measured over 100 ms: the odometry updates on its own ~10 ms clock, so one loop can see no new reading
        //and the next two at once, which would show double the real speed
        double lastProgress {measure()};
        std::uint32_t lastSpeedCheck {pros::millis()};
        std::uint32_t start {pros::millis()};
        lemlib::Pose startPose {chassis.getPose()};
        double furthest {0};    //the most progress reached in the move's direction, to show overshoot
        std::uint32_t doneAt {0};
        std::uint32_t settledSince {0};
        bool settled {false};
        while(true){
            double t {(pros::millis() - start) / 1000.0};
            ProfilePoint target {customProfile ? customProfile(t) : trapezoid(distance, maxSpeed, accel, t)};
            double progress {measure()};
            furthest = std::max(furthest, progress * (distance < 0 ? -1 : 1));
            if(pros::millis() - lastSpeedCheck >= 100){
                topSpeed = std::max(topSpeed, std::abs(progress - lastProgress) / ((pros::millis() - lastSpeedCheck) / 1000.0));
                lastProgress = progress;
                lastSpeedCheck = pros::millis();
            }
            double tolerance {toleranceOverride > 0 ? toleranceOverride : gains.smallError};
            if(target.done){
                if(doneAt == 0){
                    doneAt = pros::millis();
                }
                if(std::abs(distance - progress) > tolerance){
                    settledSince = 0;
                }
                else if(settledSince == 0){
                    settledSince = pros::millis();
                }
                settled = settledSince != 0 && pros::millis() - settledSince >= 100;
                if(settled || pros::millis() - doneAt >= 1500){
                    break;
                }
            }
            double error {target.position - progress};
            if(target.done && std::abs(error) < gains.windupRange){
                //reset on crossing the end so it can't push the robot back past it
                if(integral * error < 0){
                    integral = 0;
                }
                integral += error;
            }
            output(feedforward(target.speed, kS, kV) + pd.update(error) + gains.kI * integral);
            pros::delay(10);
        }
        leftMotorGroup.brake();
        rightMotorGroup.brake();
        //"gave up" = it never got within tolerance in the 1.5 s after the profile ended
        lemlib::Pose endPose {chassis.getPose()};
        std::printf("profiled move: %.1f of %.1f (overshot by %.1f), %s in %.1f s, top speed %.0f (profile max %.0f)\n"
                    "  from (%.1f, %.1f, %.1f) to (%.1f, %.1f, %.1f)\n",
                    measure(), distance, std::max(0.0, furthest - std::abs(distance)), settled ? "settled" : "gave up",
                    (pros::millis() - start) / 1000.0, topSpeed, maxSpeed,
                    startPose.x, startPose.y, startPose.theta, endPose.x, endPose.y, endPose.theta);
    }

    void driveStraight(double inches, double speedScale, double slowEndIn){
        lemlib::Pose start {chassis.getPose()};
        double heading {lemlib::degToRad(start.theta)};
        lemlib::PID headingPid(constants::autonHeadingKp, 0, constants::autonHeadingKd);
        double vFast {constants::autonDriveMaxSpeed * speedScale};
        double vSlow {constants::autonDriveMaxSpeed};
        double accel {constants::autonDriveAccel * speedScale};
        //the fast-then-slow profile needs room for its ramps (and a forward drive); shorter moves just run at the normal speed
        double rampIn {vFast * vFast / (2 * accel) + (vFast * vFast - vSlow * vSlow) / (2 * accel) + slowEndIn};
        std::function<ProfilePoint(double)> custom;
        if(slowEndIn > 0 && speedScale > 1 && inches >= rampIn){
            custom = [=](double t){ return fastThenSlow(inches, vFast, vSlow, slowEndIn, accel, t); };
        }
        else if(slowEndIn > 0){
            vFast = vSlow;
            accel = constants::autonDriveAccel;
        }
        followProfile(inches, vFast, accel,
                      constants::autonDriveKs,
                      constants::autonDriveKv,
                      lateralController,
                      [&]{
                          lemlib::Pose pose {chassis.getPose()};
                          return (pose.x - start.x) * std::sin(heading) + (pose.y - start.y) * std::cos(heading);
                      },
                      [&](double forward){
                          //hold the starting heading the whole way
                          double turn {headingPid.update(start.theta - chassis.getPose().theta)};
                          leftMotorGroup.move(static_cast<int>(std::clamp(forward + turn, -127.0, 127.0)));
                          rightMotorGroup.move(static_cast<int>(std::clamp(forward - turn, -127.0, 127.0)));
                      },
                      custom);
    }

    void driveInto(double inches, std::uint32_t pushMs, double speedScale, double holdHeading){
        lemlib::Pose start {chassis.getPose()};
        //turn gently toward holdHeading while driving (the aligner finishes squaring it), or keep the starting heading
        double aim {std::isnan(holdHeading) ? start.theta : start.theta + std::remainder(holdHeading - start.theta, 360.0)};
        double heading {lemlib::degToRad(start.theta)};
        lemlib::PID pd(lateralController.kP, 0, lateralController.kD);
        lemlib::PID headingPid(constants::autonHeadingKp, 0, constants::autonHeadingKd);
        std::uint32_t startMs {pros::millis()};
        double cruise {constants::autonDriveMaxSpeed * speedScale};
        double accel {constants::autonDriveAccel * speedScale};
        while(true){
            lemlib::Pose pose {chassis.getPose()};
            double progress {(pose.x - start.x) * std::sin(heading) + (pose.y - start.y) * std::cos(heading)};
            if(progress >= inches){
                break;
            }
            //a profile 100 in longer than the move never reaches its slow-down, so the robot is still at speed at the switch
            ProfilePoint target {trapezoid(inches + 100, cruise, accel, (pros::millis() - startMs) / 1000.0)};
            //a faster cruise slows to the push speed by the end of the move, without stopping
            double brakeSpeed {std::sqrt(constants::autonDriveMaxSpeed * constants::autonDriveMaxSpeed
                                         + 2 * accel * (inches - progress))};
            double forward {brakeSpeed < target.speed
                            ? feedforward(brakeSpeed, constants::autonDriveKs, constants::autonDriveKv)
                            : feedforward(target.speed, constants::autonDriveKs, constants::autonDriveKv)
                              + pd.update(target.position - progress)};
            double turn {headingPid.update(aim - pose.theta)};
            leftMotorGroup.move(static_cast<int>(std::clamp(forward + turn, -127.0, 127.0)));
            rightMotorGroup.move(static_cast<int>(std::clamp(forward - turn, -127.0, 127.0)));
            pros::delay(10);
        }
        //straight on into the aligner at the cruise power the approach was at (no PD, so it doesn't push harder once the
        //robot is held up), with no stop in between
        int power {static_cast<int>(feedforward(constants::autonDriveMaxSpeed, constants::autonDriveKs, constants::autonDriveKv))};
        leftMotorGroup.move(power);
        rightMotorGroup.move(power);
        pros::delay(pushMs);
        leftMotorGroup.brake();
        rightMotorGroup.brake();
        lemlib::Pose end {chassis.getPose()};
        std::printf("drive into goal: switched to the push after %.1f in, pose now (%.1f, %.1f, %.1f)\n",
                    inches, end.x, end.y, end.theta);
    }

    void turnTo(double heading, double speedScale, double toleranceDeg){
        double start {chassis.getPose().theta};
        //the short way round
        double degrees {std::remainder(heading - start, 360.0)};
        followProfile(degrees, constants::autonTurnMaxSpeed * speedScale, constants::autonTurnAccel * speedScale,
                      constants::autonTurnKs,
                      constants::autonTurnKv,
                      angularController,
                      [&]{ return chassis.getPose().theta - start; },
                      [](double out){
                          //clockwise positive: left forward, right back
                          int clamped {static_cast<int>(std::clamp(out, -127.0, 127.0))};
                          leftMotorGroup.move(clamped);
                          rightMotorGroup.move(-clamped);
                      },
                      nullptr, toleranceDeg);
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