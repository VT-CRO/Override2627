#include "subsystems/drivetrain.h"
#include "config/drivetrain.h"
#include "config/elevator.h"
#include "api.h"
#include "lemlib/api.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional> 
#include <utility>
#include <vector>

namespace cfg = config::drivetrain;

//Drivetrain rotation characterization (sysID): hold RIGHT alone for 1.5 s with the robot enabled and the elevator homed,
//ON THE FLOOR with room to spin in place. At each config::drivetrain::turnModelHeights elevator height it fits the turning
//model  V = kS + kV * speed + kA * acceleration  (mV, degrees, seconds) from a slow voltage ramp and a voltage step in both
//directions, then prints the turnKs / turnKv / turnKa arrays (and saves them to /usd/drive_tune.txt if an SD card is in) to
//paste into config/drivetrain.h. The turn PID and the driver heading hold are worked out from them. Press B to abort.
namespace {
    constexpr double rampMvPerSec {1000.0};         //quasistatic ramp, slow so acceleration is negligible
    constexpr double rampMaxMv {6000.0};
    constexpr double stepMv {6000.0};               //dynamic step
    constexpr uint32_t stepMs {1500};
    constexpr double minFitSpeed {5.0};             //deg/s, slower ramp samples are still stuck in static friction
    //how late sense()'s speed is on its own: the 10 ms difference is centered 5 ms back and the 0.5 low-pass adds 10 ms.
    //taken off the step's measured delay so what is left is the heading's lag, which TurnController compensates
    constexpr double speedEstimateLagS {0.015};
    constexpr double elevatorToleranceRotations {0.15};
    constexpr uint32_t elevatorTimeoutMs {5000};

struct Tuner{
    pros::MotorGroup& leftMotors;
    pros::MotorGroup& rightMotors;
    lemlib::Chassis& chassis;
    pros::v5::Controller& master;
    const Elevator& elevator;
    const std::function<void(double)>& holdElevator;

    //elevator target as a fraction of its travel, and how hard to spin there (gentler up high, like driving)
    double elevatorFraction {0};
    double voltScale {1};

    double theta {0};
    double speed {0};
    double lastTheta {0};
    uint32_t lastTime {0};
    const char* failure {""};

    void sense(){
        theta = chassis.getPose().theta;
        uint32_t now {pros::millis()};
        if(lastTime != 0 && now > lastTime){
            speed += 0.5 * ((theta - lastTheta) / ((now - lastTime) / 1000.0) - speed);
        }
        lastTheta = theta;
        lastTime = now;
    }

    //positive = clockwise (left forward, right back), matching LemLib's heading
    void spin(double mv){
        leftMotors.move_voltage(static_cast<int>(mv));
        rightMotors.move_voltage(static_cast<int>(-mv));
    }

    void stop(){
        leftMotors.brake();
        rightMotors.brake();
    }

    //one 10 ms step. false = B was pressed, stop everything
    bool tick(){
        holdElevator(elevatorFraction);
        pros::delay(10);
        sense();
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B) != 0){
            stop();
            failure = "aborted with B";
            return false;
        }
        return true;
    }

    bool coast(){
        stop();
        uint32_t start {pros::millis()};
        while(pros::millis() - start < 1000){
            if(!tick()){
                return false;
            }
        }
        return true;
    }

    //slow ramp in one direction, keeping (|speed|, |voltage|) samples once it is moving
    bool ramp(double direction, std::vector<double>& speeds, std::vector<double>& volts){
        uint32_t start {pros::millis()};
        while(true){
            if(!tick()){
                return false;
            }
            double mv {rampMvPerSec * (pros::millis() - start) / 1000.0};
            if(mv > rampMaxMv * voltScale){
                break;
            }
            spin(direction * mv);
            if(speed * direction < -minFitSpeed){
                stop();
                failure = "the heading went the wrong way (check motor directions / IMU)";
                return false;
            }
            if(speed * direction > minFitSpeed){
                speeds.push_back(std::abs(speed));
                volts.push_back(mv);
            }
        }
        return coast();
    }

    //voltage step in one direction, fitted as a first-order response delayed by lag: speed = final (1 - e^-(t - lag) / tau).
    //tau is the time constant kA / kV. lag = motor + sensor + speed estimate delay
    bool step(double direction, double& finalSpeed, double& tau, double& lag){
        std::vector<double> times;
        std::vector<double> speeds;
        uint32_t start {pros::millis()};
        spin(direction * stepMv * voltScale);
        while(pros::millis() - start < stepMs){
            if(!tick()){
                return false;
            }
            times.push_back((pros::millis() - start) / 1000.0);
            speeds.push_back(std::abs(speed));
        }
        //the last 0.3 s is the steady speed
        double sum {0};
        int count {0};
        for(size_t i {0}; i < times.size(); i++){
            if(times[i] > stepMs / 1000.0 - 0.3){
                sum += speeds[i];
                count++;
            }
        }
        finalSpeed = count > 0 ? sum / count : 0;
        //it passes 25% at lag + tau ln(4/3) and 75% at lag + tau ln 4, so two crossings give both. (the plain 63% time
        //is lag + tau, which counted the lag as inertia and read kA high)
        auto crossing {[&](double fraction){
            double level {fraction * finalSpeed};
            for(size_t i {1}; i < times.size(); i++){
                if(speeds[i - 1] < level && speeds[i] >= level){
                    return times[i - 1] + (times[i] - times[i - 1]) * (level - speeds[i - 1]) / (speeds[i] - speeds[i - 1]);
                }
            }
            return 0.0;
        }};
        double t25 {crossing(0.25)};
        double t75 {crossing(0.75)};
        tau = (t75 - t25) / std::log(3.0);
        lag = t25 - tau * std::log(4.0 / 3.0);
        if(finalSpeed < minFitSpeed || t25 <= 0 || tau <= 0){
            stop();
            failure = "the robot barely turned during the voltage step";
            return false;
        }
        if(tau > 0.4 * stepMs / 1000.0){
            stop();
            failure = "the step never reached a steady speed, the time constant is too long for the step length";
            return false;
        }
        return coast();
    }

    void emit(FILE* file, const char* text){
        std::printf("%s", text);
        if(file != nullptr){
            std::fputs(text, file);
        }
    }

    //move the elevator to fraction of its travel and wait for it, holding it there from then on
    bool moveElevator(double fraction){
        elevatorFraction = fraction;
        voltScale = 1.0 - (1.0 - cfg::elevatorTopSpeedScale) * fraction;
        double target {fraction * config::elevator::maxMotorRotations};
        uint32_t start {pros::millis()};
        while(std::abs(elevator.rotations() - target) > elevatorToleranceRotations){
            if(!tick()){
                return false;
            }
            if(pros::millis() - start > elevatorTimeoutMs){
                failure = "the elevator never reached the test height (home it with X first)";
                return false;
            }
        }
        //let the sway die down before spinning
        return coast();
    }

    //fit kS, kV, kA (and the delay, into lagSum) at the elevator's current height
    bool measure(double& kS, double& kV, double& kA, double& lagSum){
        //quasistatic: both directions pooled, least squares |V| = kS + kV * |speed|
        std::vector<double> speeds;
        std::vector<double> volts;
        if(!ramp(1, speeds, volts) || !ramp(-1, speeds, volts)){
            return false;
        }
        double n {static_cast<double>(speeds.size())};
        double sx {0}, sy {0}, sxx {0}, sxy {0};
        for(size_t i {0}; i < speeds.size(); i++){
            sx += speeds[i];
            sy += volts[i];
            sxx += speeds[i] * speeds[i];
            sxy += speeds[i] * volts[i];
        }
        double denominator {n * sxx - sx * sx};
        if(n < 20 || denominator <= 0){
            failure = "too few moving samples in the ramp";
            return false;
        }
        kV = (n * sxy - sx * sy) / denominator;
        kS = (sy - kV * sx) / n;
        if(kV <= 0){
            failure = "the ramp fit came out with no speed per volt";
            return false;
        }

        //dynamic: both directions averaged, kA = time constant * kV
        double cwSpeed {0}, cwTau {0}, cwLag {0}, ccwSpeed {0}, ccwTau {0}, ccwLag {0};
        if(!step(1, cwSpeed, cwTau, cwLag) || !step(-1, ccwSpeed, ccwTau, ccwLag)){
            return false;
        }
        kA = (cwTau + ccwTau) / 2 * kV;
        lagSum += cwLag + ccwLag;
        std::printf("  elevator %.2f: kS %.0f mV, kV %.2f mV per deg/s, kA %.3f mV per deg/s^2 (step %.0f / %.0f deg/s, "
                    "time constant %.3f / %.3f s, delay %.3f / %.3f s cw / ccw)\n",
                    elevatorFraction, kS, kV, kA, cwSpeed, ccwSpeed, cwTau, ccwTau, cwLag, ccwLag);
        return true;
    }

    void run(){
    //must hold RIGHT alone for 1.5 s so it can't start by accident (LEFT + RIGHT is the elevator auto-tune)
    uint32_t heldSince {pros::millis()};
    while(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0 && master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) == 0){
        if(pros::millis() - heldSince > 1500){
            break;
        }
        pros::delay(20);
    }
    if(pros::millis() - heldSince <= 1500){
        return;
    }
    master.rumble("-");
    std::printf("\ndrivetrain rotation sysID starting (B aborts). robot on the floor with room to spin\n");
    while(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0){
        pros::delay(20);
    }
    pros::delay(1000);

    lastTime = 0;
    speed = 0;
    sense();

    constexpr size_t count {cfg::turnModelHeights.size()};
    std::array<double, count> kS {}, kV {}, kA {};
    double lagSum {0};
    bool ok {true};
    for(size_t i {0}; ok && i < count; i++){
        ok = moveElevator(cfg::turnModelHeights[i]) && measure(kS[i], kV[i], kA[i], lagSum);
    }
    double sensorLag {lagSum / (2 * count) - speedEstimateLagS};
    if(ok && (sensorLag < 0 || sensorLag > 0.1)){
        failure = "the step's delay came out outside 0-100 ms, so the fit is off";
        ok = false;
    }
    stop();
    //back to the lowest test height (the intake height), not all the way down onto the wheels. driver control's elevator
    //update finishes the trip
    holdElevator(cfg::turnModelHeights[0]);

    if(!ok){
        std::printf("rotation sysID FAILED: %s. nothing was changed\n", failure);
        master.rumble("---");
        return;
    }

    char text[1600];
    int used {std::snprintf(text, sizeof(text), "\n//---- paste into config/drivetrain.h (rotation sysID) ----\n")};
    for(auto [name, values] : {std::pair{"turnKs", &kS}, std::pair{"turnKv", &kV}, std::pair{"turnKa", &kA}}){
        used += std::snprintf(text + used, sizeof(text) - used, "    constexpr std::array<double, %u> %s {",
                              static_cast<unsigned>(count), name);
        for(size_t i {0}; i < count; i++){
            used += std::snprintf(text + used, sizeof(text) - used, i == 0 ? "%.3f" : ", %.3f", (*values)[i]);
        }
        used += std::snprintf(text + used, sizeof(text) - used, "};\n");
    }
    //TurnController goes straight-line between heights, so check each inner one against its neighbors' line: a big
    //miss means the model bends there and a height in between would help
    double worstBend {0};
    double bendAt {0};
    for(size_t i {1}; i + 1 < count; i++){
        const auto& h {cfg::turnModelHeights};
        double t {(h[i] - h[i - 1]) / (h[i + 1] - h[i - 1])};
        for(const auto* values : {&kS, &kV, &kA}){
            double line {(*values)[i - 1] + ((*values)[i + 1] - (*values)[i - 1]) * t};
            double bend {std::abs((*values)[i] / line - 1)};
            if(bend > worstBend){
                worstBend = bend;
                bendAt = h[i];
            }
        }
    }
    std::snprintf(text + used, sizeof(text) - used,
                  "    constexpr double turnSensorLagS {%.3f};\n"
                  "//top turn speed at 12 V, lowest height: about %.0f deg/s\n"
                  "//worst miss of a straight line between heights: %.0f%% at %.2f%s\n"
                  "//-----------------------------------------------------\n",
                  sensorLag, (12000.0 - kS[0]) / kV[0], worstBend * 100, bendAt,
                  worstBend > 0.1 ? " (over 10%: add heights around it to turnModelHeights)" : "");

    FILE* file {std::fopen("/usd/drive_tune.txt", "w")};
    emit(file, text);
    if(file != nullptr){
        std::fclose(file);
    }
    master.rumble(".");
    }
};
}

void Drivetrain::rotationSysId(const std::function<void(double)>& holdElevator){
    Tuner{leftMotors, rightMotors, chassis, master, elevator, holdElevator}.run();
}
