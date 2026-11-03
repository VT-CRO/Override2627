//PC check for util/turn_controller: simulates the turning model with static friction, IMU lag and the DR4B's flex, and
//runs a set of turns. not part of the robot build (pros make only builds src/). from the project folder:
//  g++ -std=c++20 -Iinclude test/turn_controller_sim.cpp src/util/turn_controller.cpp src/util/turn_schedule.cpp -o turn_sim && ./turn_sim
//TURN_SIM_TRACE=1 prints every tick. TURN_SIM_NOASSERT=1 prints every case without stopping (for sweeping config:
//build with -I<folder holding an edited config/drivetrain.h> before -Iinclude)
#include "util/turn_controller.h"
#include "util/turn_schedule.h"
#include "config/drivetrain.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>

namespace {
//the DR4B as a torsion spring between the base and the lift's mass: a share of the turning inertia twists against the
//base at its own frequency, softer and heavier the higher it is. share 15% -> 30% from down to top is a guess. the
//frequency and damping were read by an earlier sysID on 2026-11-02 (2.85 Hz at 0.60, 2.50 Hz at the top, damping
//0.31; nothing seen at the intake height, so 6 Hz there is a guess). the IMU is on the base, so it feels the sway
constexpr double pi {3.14159265358979};
constexpr double swayDamping {0.31};
double towerShare(double h){ return 0.15 + 0.15 * h; }
double swayHz(double h){
    constexpr double low {config::drivetrain::turnLowestHeight};
    constexpr double mid {(low + 1.0) / 2};
    return h <= low ? 6.0 : h <= mid ? 6.0 + (2.85 - 6.0) * (h - low) / (mid - low) : 2.85 + (2.50 - 2.85) * (h - mid) / (1 - mid);
}

struct Result{
    bool settled;
    double timeS;
    double overshoot;
    double maxAccelStep;
    double swayAfter;    //biggest twist (deg) in the 0.5 s after it settled
};

//kAScale: the real robot's inertia vs the model (model mismatch). flex false = rigid robot
Result run(double degrees, const std::function<double(double)>& heightAt, double kAScale = 1.0, bool flex = true){
    constexpr double ctrlDt {0.01};
    constexpr int subSteps {10};
    constexpr int lagTicks {2};    //20 ms IMU / odometry lag
    double base {0}, w {0}, tower {0}, wTower {0}, volts {0};
    std::deque<double> seen(lagTicks + 1, 0.0);
    std::deque<double> seenRate(lagTicks + 1, 0.0);
    TurnController turn(turnSettings());
    //like Drivetrain: the limits come from the height at the start, the gains follow it every loop
    turn.start(0, degrees, turnLimitsAt(heightAt(0), turnModelAt(heightAt(0))));
    double sign {degrees < 0 ? -1.0 : 1.0};
    double lastARef {0}, overshoot {0}, maxStep {0}, sway {0};
    double settledAt {-1};
    for(int tick {0}; tick < 400; tick++){
        double t {tick * ctrlDt};
        double h {heightAt(t)};
        TurnModel m {turnModelAt(h)};
        double kA {m.kA * kAScale};
        double share {flex ? towerShare(h) : 0.0};
        double kTower {share * kA};
        double spring {kTower * std::pow(2 * pi * swayHz(h), 2)};
        double damper {2 * swayDamping * std::sqrt(spring * kTower)};
        //plant, 1 ms steps (semi-implicit Euler)
        for(int i {0}; i < subSteps; i++){
            double dt {ctrlDt / subSteps};
            double twist {flex ? spring * (tower - base) + damper * (wTower - w) : 0.0};
            double drive {volts + twist};
            if(!(w == 0 && std::abs(drive) <= m.kS)){
                double friction {std::copysign(m.kS, w != 0 ? w : drive)};
                double next {w + (drive - friction - m.kV * w) / (kA - kTower) * dt};
                w = (w != 0 && next * w < 0) ? 0 : next;    //friction stops it, it doesn't reverse it
            }
            base += w * dt;
            if(flex){
                wTower += -twist / kTower * dt;
                tower += wTower * dt;
            }
            else{
                tower = base;
            }
        }
        seen.push_back(base);
        seen.pop_front();
        double measured {seen.front()};
        //the base's turn rate, as late as the heading, like the IMU gyro rate Drivetrain's turn loop uses
        seenRate.push_back(w);
        seenRate.pop_front();
        double rate {seenRate.front()};
        overshoot = std::max(overshoot, (base - degrees) * sign);

        volts = turn.update(measured, rate, m, turnFeedbackWnAt(h), ctrlDt);
        if(std::getenv("TURN_SIM_TRACE") != nullptr){
            std::printf("  t %.2f base %.2f twist %.2f w %.0f V %.0f aRef %.0f\n", t, base, tower - base, w, volts,
                        turn.referenceAccel());
        }
        assert(std::abs(volts) <= 12000.0);
        double aRef {turn.referenceAccel()};
        //the plan's acceleration should never step: constant jerk pieces only
        if(turn.profiling()){
            maxStep = std::max(maxStep, std::abs(aRef - lastARef));
        }
        lastARef = aRef;
        if(settledAt < 0 && turn.settled()){
            settledAt = t;
        }
        if(settledAt >= 0){
            sway = std::max(sway, std::abs(tower - base));
            if(t - settledAt >= 0.5){
                break;
            }
        }
    }
    return {settledAt >= 0, settledAt >= 0 ? settledAt : 4.0, overshoot, maxStep, sway};
}

void check(const char* name, const Result& r){
    std::printf("%-30s %s in %.2f s, overshoot %.2f deg, biggest accel step %.0f deg/s^2 per tick, sway after %.2f deg\n",
                name, r.settled ? "settled" : "TIMED OUT", r.timeS, r.overshoot, r.maxAccelStep, r.swayAfter);
    if(std::getenv("TURN_SIM_NOASSERT")) return;
    assert(r.settled);
    assert(r.overshoot < 1.0);
    //jerk cap 30000 deg/s^3 down is 300 per 10 ms tick; a step would be thousands
    assert(r.maxAccelStep < 400.0);
}
}

int main(){
    auto fixed {[](double h){ return [h](double){ return h; }; }};
    check("90 deg, elevator down", run(90, fixed(0)));
    check("-90 deg, elevator down", run(-90, fixed(0)));
    check("180 deg, elevator down", run(180, fixed(0)));
    check("10 deg, elevator down", run(10, fixed(0)));
    check("90 deg, elevator middle", run(90, fixed(0.5)));
    check("90 deg, elevator top", run(90, fixed(1)));
    check("10 deg, elevator top", run(10, fixed(1)));
    check("180 deg, rising 0 to 1", run(180, [](double t){ return std::min(1.0, t / 0.3); }));
    check("90 deg, 20% more inertia", run(90, fixed(0), 1.2));
    check("90 deg, 20% less inertia", run(90, fixed(0), 0.8));
    check("90 deg, top, rigid robot", run(90, fixed(1), 1.0, false));
    std::printf("all passed\n");
}
