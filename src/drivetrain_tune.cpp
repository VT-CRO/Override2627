#include "drivetrain.h"
#include "constants.h"
#include "api.h"
#include "lemlib/api.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

//Drivetrain rotation characterization (sysID): hold RIGHT alone for 1.5 s with the robot enabled, ON THE FLOOR with room
//to spin in place. It fits the turning model  V = kS + kV * speed + kA * acceleration  (mV, degrees, seconds) from a slow
//voltage ramp and a voltage step in both directions, then prints LemLib angularController gains (and saves them to
///usd/drive_tune.txt if an SD card is in) to paste into drivetrain.cpp. Press B to abort.
namespace drivetrain{
    extern pros::MotorGroup leftMotorGroup;
    extern pros::MotorGroup rightMotorGroup;

    namespace {
        constexpr double rampMvPerSec {1000.0};         //quasistatic ramp, slow so acceleration is negligible
        constexpr double rampMaxMv {6000.0};
        constexpr double stepMv {6000.0};               //dynamic step
        constexpr uint32_t stepMs {1500};
        constexpr double minFitSpeed {5.0};             //deg/s, slower ramp samples are still stuck in static friction
        constexpr double settleTimeS {0.6};             //closed loop settle time the gains are chosen for (critically damped)
        constexpr double lemlibLoopS {0.01};            //turnToHeading runs every 10 ms and LemLib's D is per loop, not per second
        constexpr double integralTimeS {0.4};           //how fast the integral makes up for friction, slower = less overshoot

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
            leftMotorGroup.move_voltage(static_cast<int>(mv));
            rightMotorGroup.move_voltage(static_cast<int>(-mv));
        }

        void stop(){
            leftMotorGroup.brake();
            rightMotorGroup.brake();
        }

        //one 10 ms step. false = B was pressed, stop everything
        bool tick(){
            pros::delay(10);
            sense();
            if(constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_B) != 0){
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
                if(mv > rampMaxMv){
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

        //voltage step in one direction: the time to reach 63% of the final speed is the time constant kA / kV
        bool step(double direction, double& finalSpeed, double& tau){
            std::vector<double> times;
            std::vector<double> speeds;
            uint32_t start {pros::millis()};
            spin(direction * stepMv);
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
            tau = 0;
            for(size_t i {0}; i < times.size(); i++){
                if(speeds[i] >= 0.632 * finalSpeed){
                    tau = times[i];
                    break;
                }
            }
            if(finalSpeed < minFitSpeed || tau <= 0){
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
    }

    void rotationSysId(){
        //must hold RIGHT alone for 1.5 s so it can't start by accident (LEFT + RIGHT is the elevator auto-tune)
        uint32_t heldSince {pros::millis()};
        while(constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0 && constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) == 0){
            if(pros::millis() - heldSince > 1500){
                break;
            }
            pros::delay(20);
        }
        if(pros::millis() - heldSince <= 1500){
            return;
        }
        constants::master.rumble("-");
        std::printf("\ndrivetrain rotation sysID starting (B aborts). robot on the floor with room to spin\n");
        while(constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0){
            pros::delay(20);
        }
        pros::delay(1000);

        lastTime = 0;
        speed = 0;
        sense();

        //quasistatic: both directions pooled, least squares |V| = kS + kV * |speed|
        std::vector<double> speeds;
        std::vector<double> volts;
        bool ok {ramp(1, speeds, volts) && ramp(-1, speeds, volts)};
        double kS {0}, kV {0};
        if(ok){
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
                ok = false;
            }
            else{
                kV = (n * sxy - sx * sy) / denominator;
                kS = (sy - kV * sx) / n;
                if(kV <= 0){
                    failure = "the ramp fit came out with no speed per volt";
                    ok = false;
                }
            }
        }

        //dynamic: both directions averaged
        double cwSpeed {0}, cwTau {0}, ccwSpeed {0}, ccwTau {0};
        ok = ok && step(1, cwSpeed, cwTau) && step(-1, ccwSpeed, ccwTau);
        stop();

        if(!ok){
            std::printf("rotation sysID FAILED: %s. nothing was changed\n", failure);
            constants::master.rumble("---");
            return;
        }

        double tau {(cwTau + ccwTau) / 2};
        double kA {tau * kV};
        //PD on the model kA * accel + kV * speed = V: critically damped, 2% settle in settleTimeS (settle = 5.83 / natural freq)
        double wn {5.83 / settleTimeS};
        double kP {kA * wn * wn};                       //mV per degree
        double kD {std::max(0.0, 2 * wn * kA - kV)};    //mV per (degree / second)
        //P alone can't beat friction within kS / kP of the target, so the turn would stall there. an integral that only
        //runs inside that band (LemLib zeroes it outside windupRange) builds up the missing kS in about integralTimeS
        double windup {std::ceil(kS / kP)};
        double kI {kP * lemlibLoopS / integralTimeS};   //mV per (degree * loop)
        //LemLib outputs -127..127 (12 V), its D is error change per loop and its I is the sum of errors per loop
        double lemKp {kP * 127.0 / 12000.0};
        double lemKi {kI * 127.0 / 12000.0};
        double lemKd {kD * 127.0 / 12000.0 / lemlibLoopS};

        char text[1600];
        std::snprintf(text, sizeof(text),
            "\n//---- paste into drivetrain.cpp (rotation sysID) ----\n"
            "static lemlib::ControllerSettings angularController(\n"
            "    %.2f,  //P\n"
            "    %.3f,  //I, only within the windup range below, to push through friction\n"
            "    %.2f,  //D\n"
            "    %.0f,  //integral only counts within this many degrees of the target\n"
            "    1,  //exits once within 1 deg for 100 ms\n"
            "    100,\n"
            "    3,  //or within 3 deg for 500 ms\n"
            "    500,\n"
            "    0  //slew off, the gains already assume the motors' own response\n"
            ");\n"
            "//measured: kS %.0f mV (= %.1f stick units, driveMinOutput is %.1f), kV %.2f mV per deg/s, kA %.3f mV per deg/s^2\n"
            "//top turn speed at 12 V about %.0f deg/s. step: %.0f / %.0f deg/s, time constant %.3f / %.3f s (cw / ccw)\n"
            "//gains are for a %.1f s critically damped turn; raise settleTimeS in drivetrain_tune.cpp if it oscillates\n"
            "//-----------------------------------------------------\n",
            lemKp, lemKi, lemKd, windup, kS, kS * 127.0 / 12000.0, constants::driveMinOutput, kV, kA,
            (12000.0 - kS) / kV, cwSpeed, ccwSpeed, cwTau, ccwTau, settleTimeS);

        FILE* file {std::fopen("/usd/drive_tune.txt", "w")};
        emit(file, text);
        if(file != nullptr){
            std::fclose(file);
        }
        constants::master.rumble(".");
    }
}
