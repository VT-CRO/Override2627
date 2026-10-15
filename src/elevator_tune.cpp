#include "elevator.h"
#include "constants.h"
#include "api.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

//Elevator auto-tune: hold LEFT + RIGHT on the controller for 1.5 s with the robot enabled, ON BLOCKS, claw empty.
//It measures the elevator's gravity hold voltage, friction, response and home stall current, then prints a block of
//constants (and saves it to /usd/elevator_tune.txt if an SD card is in) to paste into constants.h. Press B to abort.
//It only moves between 0.5 rotations above the bottom and 0.5 below the top, and re-homes when it finishes.
namespace elevator{
    extern pros::MotorGroup elevatorMotors;

    namespace {
        constexpr double margin {0.5};                  //rotations kept clear of the top while testing
        constexpr double floorGuard {0.1};              //lowest position a test may reach, so none of them slams the bottom
        constexpr double topPushMv {800.0};             //voltage above the hold voltage while searching for the top stop
        constexpr double rampMvPerLoop {60.0};          //breakaway ramp, 3000 mV/s at a 20 ms loop
        constexpr int confirmLoops {3};                 //loops of movement needed to call a breakaway
        constexpr double moveSpeed {0.05};              //rot/s that counts as moving
        constexpr double stepMv {2500.0};               //voltage above/below the hold voltage for the response test
        constexpr double settleTimeS {0.2};             //closed loop time constant the P gain is chosen for
        constexpr int stallTrials {3};

        double pos {0};
        double speed {0};
        double lastPos {0};
        uint32_t lastTime {0};
        const char* failure {""};
        bool aborted {false};

        void sense(){
            double sum {0};
            std::vector<double> positions {elevatorMotors.get_position_all()};
            for(double p : positions){
                sum += p;
            }
            pos = positions.empty() ? 0 : sum / positions.size();
            uint32_t now {pros::millis()};
            if(lastTime != 0 && now > lastTime){
                speed += 0.5 * ((pos - lastPos) / ((now - lastTime) / 1000.0) - speed);
            }
            lastPos = pos;
            lastTime = now;
        }

        //one 20 ms step. false = B was pressed, stop everything
        bool tick(){
            pros::delay(20);
            sense();
            if(constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_B) != 0){
                elevatorMotors.brake();
                failure = "aborted with B";
                aborted = true;
                return false;
            }
            return true;
        }

        bool wait(uint32_t ms){
            uint32_t start {pros::millis()};
            while(pros::millis() - start < ms){
                if(!tick()){
                    return false;
                }
            }
            return true;
        }

        //coarse move to a test position and hold there. only needs to land close, the tests don't need it exact
        //PD plus a small integral: gravity is not known yet, so the integral finds the voltage that gets it the last bit
        //of the way, and the speed term stops it overshooting and swinging back and forth
        bool goTo(double target){
            uint32_t start {pros::millis()};
            double integral {0};
            while(true){
                if(!tick()){
                    return false;
                }
                double error {target - pos};
                if(std::abs(error) < 0.2 && std::abs(speed) < 0.3){
                    elevatorMotors.brake();
                    if(!wait(400)){
                        return false;
                    }
                    if(std::abs(pos - target) > 0.5){
                        std::printf("goTo: wanted %.2f, drifted to %.2f after braking\n", target, pos);
                        failure = "the lift would not hold a test position (drifted after braking)";
                        return false;
                    }
                    return true;
                }
                if(pros::millis() - start > 10000){
                    elevatorMotors.brake();
                    std::printf("goTo: wanted %.2f, stuck at %.2f moving %.2f rot/s\n", target, pos, speed);
                    failure = "could not reach a test position";
                    return false;
                }
                integral = std::clamp(integral + 8000.0 * error * 0.02, -4000.0, 4000.0);
                double output {std::clamp(5000.0 * error - 1200.0 * speed + integral, -6000.0, 6000.0)};
                elevatorMotors.move_voltage(static_cast<int>(output));
            }
        }

        //ramp the voltage up until the lift starts rising, then back down until it starts falling.
        //halfway between the two is the voltage that holds it against gravity, half the gap is friction
        bool breakaway(double at, double& gravityMv, double& frictionMv){
            if(!goTo(at)){
                return false;
            }
            double voltage {0};
            int moving {0};
            double upMv {0};
            while(true){
                if(!tick()){
                    return false;
                }
                voltage += rampMvPerLoop;
                elevatorMotors.move_voltage(static_cast<int>(voltage));
                moving = speed > moveSpeed ? moving + 1 : 0;
                if(moving >= confirmLoops){
                    //the detection lagged the real breakaway by confirmLoops loops of ramp
                    upMv = voltage - confirmLoops * rampMvPerLoop;
                    break;
                }
                if(voltage > 9000.0 || pos < at - 0.6){
                    elevatorMotors.brake();
                    failure = "lift never started rising (or sagged too far)";
                    return false;
                }
            }
            moving = 0;
            while(true){
                if(!tick()){
                    return false;
                }
                voltage -= rampMvPerLoop;
                elevatorMotors.move_voltage(static_cast<int>(voltage));
                moving = speed < -moveSpeed ? moving + 1 : 0;
                if(moving >= confirmLoops){
                    double downMv {voltage + confirmLoops * rampMvPerLoop};
                    elevatorMotors.brake();
                    if(upMv <= downMv){
                        failure = "up and down breakaway voltages did not separate";
                        return false;
                    }
                    gravityMv = (upMv + downMv) / 2;
                    frictionMv = (upMv - downMv) / 2;
                    return true;
                }
                if(voltage < -9000.0 || pos < floorGuard || pos > constants::elevatorMaxMotorRotations - margin + 0.2){
                    elevatorMotors.brake();
                    failure = "lift never started falling (or left the test range)";
                    return false;
                }
            }
        }

        //average speed over the second half of a fixed voltage push, in rot/s
        bool pushSpeed(double mv, double& result){
            elevatorMotors.move_voltage(static_cast<int>(mv));
            if(!wait(350)){
                return false;
            }
            double sum {0};
            int count {0};
            for(int i {0}; i < 15; i++){
                if(!tick()){
                    return false;
                }
                sum += speed;
                count++;
                if(pos < margin - 0.2 || pos > constants::elevatorMaxMotorRotations - margin + 0.2){
                    break;
                }
            }
            elevatorMotors.brake();
            result = sum / count;
            return true;
        }

        //gently push up until the lift stops against the top, using the measured hold voltage so the push is the same
        //slow speed all the way up (not a fixed voltage that would be fast near the top where gravity is light)
        bool findTop(double lowAt, double highAt, double lowMv, double highMv, double& topPos){
            if(!goTo(highAt)){
                return false;
            }
            bool moved {false};
            int stalled {0};
            uint32_t start {pros::millis()};
            while(true){
                if(!tick()){
                    return false;
                }
                double t {std::clamp((pos - lowAt) / (highAt - lowAt), 0.0, 1.0)};
                double hold {lowMv + t * (highMv - lowMv)};
                elevatorMotors.move_voltage(static_cast<int>(std::min(hold + topPushMv, 7000.0)));
                moved = moved || speed > 0.1;
                stalled = (moved && std::abs(speed) < 0.03) ? stalled + 1 : 0;
                if(stalled >= 15){
                    elevatorMotors.brake();
                    topPos = pos;
                    return true;
                }
                if(pos > 1.6 * constants::elevatorMaxMotorRotations || pros::millis() - start > 15000){
                    elevatorMotors.brake();
                    failure = moved ? "no top stop found within 1.6x the current travel" : "lift never started moving up to find the top";
                    return false;
                }
            }
        }

        double median(std::vector<double> values){
            if(values.empty()){
                return 0;
            }
            std::sort(values.begin(), values.end());
            return values[values.size() / 2];
        }

        //drive down at the home voltage into the bottom and record the current once it has stopped.
        //the voltage is fixed, so the stall current plateaus even though the chain keeps tightening
        bool stallTrial(double& stallPos, double& freeMa, double& plateauMa, double& stallRpm){
            if(!goTo(2.0)){
                return false;
            }
            std::vector<double> freeCurrents;
            std::vector<double> stallCurrents;
            double rpmMax {0};
            bool contact {false};
            uint32_t start {pros::millis()};
            uint32_t contactTime {0};
            elevatorMotors.move_voltage(constants::elevatorHomeMv);
            while(true){
                if(!tick()){
                    return false;
                }
                uint32_t elapsed {pros::millis() - start};
                int32_t current {0};
                for(int32_t c : elevatorMotors.get_current_draw_all()){
                    current = std::max(current, c);
                }
                if(!contact){
                    if(elapsed > constants::elevatorHomeGraceMs){
                        if(std::abs(speed) < 0.1){
                            contact = true;
                            contactTime = pros::millis();
                        }
                        else{
                            freeCurrents.push_back(current);
                        }
                    }
                }
                else if(pros::millis() - contactTime > 1200){
                    //after settling on the stop, the last 0.8 s is the plateau
                    stallCurrents.push_back(current);
                    for(double v : elevatorMotors.get_actual_velocity_all()){
                        rpmMax = std::max(rpmMax, std::abs(v));
                    }
                    if(pros::millis() - contactTime > 2000){
                        break;
                    }
                }
                if(elapsed > 6000){
                    elevatorMotors.brake();
                    failure = "never stopped on the bottom during the stall test";
                    return false;
                }
            }
            elevatorMotors.brake();
            stallPos = pos;
            freeMa = median(freeCurrents);
            plateauMa = median(stallCurrents);
            stallRpm = rpmMax;
            return true;
        }

        void emit(FILE* file, const char* text){
            std::printf("%s", text);
            if(file != nullptr){
                std::fputs(text, file);
            }
        }
    }

    void autoTune(){
        //must hold LEFT + RIGHT for 1.5 s so it can't start by accident
        uint32_t heldSince {pros::millis()};
        while(constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) != 0 && constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0){
            if(pros::millis() - heldSince > 1500){
                break;
            }
            pros::delay(20);
        }
        if(pros::millis() - heldSince <= 1500){
            return;
        }
        constants::master.rumble("-");
        std::printf("\nelevator auto-tune starting (B aborts). robot on blocks, claw empty\n");
        while(constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) != 0 || constants::master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) != 0){
            pros::delay(20);
        }
        pros::delay(1000);

        lastTime = 0;
        aborted = false;
        sense();
        sense();
        //the current travel is only used to place the test heights. the tests assume at least ~4 rotations of travel
        const double top {constants::elevatorMaxMotorRotations};
        //the two reference heights for gravity. both are well above the slack at the bottom, which is checked below
        const double lowAt {0.3 * top};
        const double highAt {0.6 * top};
        double lowG {0}, lowF {0}, highG {0}, highF {0};
        double loadedStart {0}, topPos {0};
        double kv {0}, noise {0};
        double freeMa {0}, plateauMa {1e9}, stallRpm {0};
        double stallMin {1e9}, stallMax {-1e9};
        bool ok {true};

        //gravity and friction at the two reference heights, twice each, averaged
        for(int pass {0}; pass < 2 && ok; pass++){
            double g {0}, f {0};
            ok = breakaway(lowAt, g, f);
            if(ok){ lowG += g / 2; lowF += f / 2; }
            if(ok){
                ok = breakaway(highAt, g, f);
            }
            if(ok){ highG += g / 2; highF += f / 2; }
        }

        //the give at the bottom: down there turning the motor mostly unwinds slack instead of lifting, so the hold voltage
        //is ~0. probe two low heights and find where it climbs to half of its loaded value. a probe that can't lift
        //anything (never rises, or sags to the floor) counts as 0 because nothing is loading the motor there
        if(ok && lowG > 100.0){
            const double probeAt[2] {0.5, 1.0};
            double probeG[2] {0, 0};
            for(int i {0}; i < 2 && ok; i++){
                double g {0}, f {0};
                if(breakaway(probeAt[i], g, f)){
                    probeG[i] = g;
                }
                else if(aborted){
                    ok = false;
                }
            }
            if(ok){
                double half {0.5 * lowG};
                if(probeG[0] >= half){
                    loadedStart = 0;
                }
                else if(probeG[1] >= half){
                    loadedStart = probeAt[0] + (half - probeG[0]) / (probeG[1] - probeG[0]) * (probeAt[1] - probeAt[0]);
                }
                else{
                    loadedStart = probeAt[1] + (half - probeG[1]) / (lowG - probeG[1]) * (lowAt - probeAt[1]);
                }
                if(loadedStart > lowAt - 0.3){
                    failure = "the give at the bottom reaches the lower test height, so the gravity reading there is not trustworthy";
                    ok = false;
                }
            }
        }

        //response: speed per millivolt, pushing up and down around the middle
        if(ok){
            double midG {(lowG + highG) / 2};
            double midF {(lowF + highF) / 2};
            double up {0}, down {0};
            ok = goTo(lowAt) && pushSpeed(midG + midF + stepMv, up) && goTo(highAt) && pushSpeed(midG - midF - stepMv, down);
            if(ok){
                kv = (std::abs(up) + std::abs(down)) / 2 / stepMv;
                if(kv <= 0){
                    failure = "measured no response to the voltage step";
                    ok = false;
                }
            }
        }

        //speed noise while holding still
        if(ok){
            ok = goTo(0.5 * top);
            for(int i {0}; i < 50 && ok; i++){
                ok = tick();
                noise = std::max(noise, std::abs(speed));
            }
        }

        //highest point: push up gently until it stops against the top
        if(ok){
            ok = findTop(lowAt, highAt, lowG + lowF, highG + highF, topPos);
        }

        //home stall current, last because it re-zeroes the encoders
        for(int trial {0}; trial < stallTrials && ok; trial++){
            double stallPos {0}, free {0}, plateau {0}, rpm {0};
            //the start height must be above the give at the bottom
            if(loadedStart > 1.5){
                failure = "the give at the bottom is too large for the stall test start height";
                ok = false;
                break;
            }
            ok = stallTrial(stallPos, free, plateau, rpm);
            if(ok){
                freeMa = std::max(freeMa, free);
                plateauMa = std::min(plateauMa, plateau);
                stallRpm = std::max(stallRpm, rpm);
                stallMin = std::min(stallMin, stallPos);
                stallMax = std::max(stallMax, stallPos);
            }
        }
        if(ok && plateauMa < 1.5 * freeMa){
            failure = "stall current was not clearly above the moving current";
            ok = false;
        }

        elevatorMotors.brake();
        if(!ok){
            std::printf("auto-tune FAILED: %s. nothing was changed\n", failure);
            constants::master.rumble("---");
        }
        else{
            double kp {std::clamp(1.0 / (kv * settleTimeS), 3000.0, 30000.0)};
            double kd {0.04 * kp};
            double ki {167.0 * lowF};
            double integralMax {2 * std::max(lowF, highF) + 0.25 * std::max(std::abs(lowG), std::abs(highG))};

            char text[2200];
            std::snprintf(text, sizeof(text),
                "\n//---- paste into constants.h (elevator auto-tune) ----\n"
                "constexpr double elevatorLoadedStartRotations {%.2f};\n"
                "constexpr double elevatorGravityLowAtRotations {%.2f};\n"
                "constexpr double elevatorGravityHighAtRotations {%.2f};\n"
                "constexpr double elevatorGravityLowMv {%.0f};\n"
                "constexpr double elevatorGravityHighMv {%.0f};\n"
                "constexpr double elevatorMinVoltageMv {%.0f};\n"
                "constexpr double elevatorKp {%.0f};\n"
                "constexpr double elevatorKd {%.0f};\n"
                "constexpr double elevatorKi {%.0f};\n"
                "constexpr double elevatorIntegralMaxMv {%.0f};\n"
                "constexpr double elevatorSettleSpeed {%.2f};\n"
                "constexpr int32_t elevatorHomeStallCurrentMa {%.0f};\n"
                "constexpr double elevatorHomeStallRpm {%.1f};\n"
                "//highest point: the lift stopped at %.2f rotations (now elevatorMaxMotorRotations = %.2f). backed off 0.15 so it doesn't\n"
                "//press the stop: constexpr double elevatorMaxMotorRotations {%.2f}; (the Y/X/A levels are fractions of it, so they move too)\n"
                "//measured: friction %.0f mV (low) %.0f mV (high), %.2f rot/s per volt, moving current %.0f mA, stall current %.0f mA\n"
                "//home repeatability over %d trials: the stop position varied by %.3f rotations (this is the zero error)\n"
                "//-----------------------------------------------------\n",
                loadedStart, lowAt, highAt, lowG, highG, 1.2 * std::max(lowF, highF), kp, kd, ki, integralMax,
                std::max(0.05, 4 * noise), 0.9 * plateauMa, std::max(2.0, 1.5 * stallRpm + 0.5),
                topPos, top, topPos - 0.15,
                lowF, highF, kv * 1000, freeMa, plateauMa, stallTrials, stallMax - stallMin);

            FILE* file {std::fopen("/usd/elevator_tune.txt", "w")};
            emit(file, text);
            if(file != nullptr){
                std::fclose(file);
            }
            constants::master.rumble(".");
        }

        //the stall trials moved the zero and the other tests were free of the normal state, so home again the usual way
        forgetHome();
        homeBlocking();
    }
}
