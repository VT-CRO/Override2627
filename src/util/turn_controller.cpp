#include "util/turn_controller.h"
#include <algorithm>
#include <cmath>

TurnController::TurnController(const TurnSettings& settings) : settings(settings) {}

void TurnController::start(double thetaDeg, double targetDeg, const TurnLimits& limits){
    startPos = thetaDeg;
    target = targetDeg;
    sign = targetDeg >= thetaDeg ? 1.0 : -1.0;
    jerk = limits.jerk;
    elapsed = 0;
    posRef = thetaDeg;
    vRef = 0;
    aRef = 0;
    lastError = targetDeg - thetaDeg;
    endIntegral = 0;
    settledForS = 0;

    //speeding up from rest to v: the acceleration ramps to min(accel, sqrt(j v)) (short speed-ups never reach the cap),
    //holds, and ramps back to 0, taking v / a + a / j. it is symmetric, so the average speed is v / 2
    auto speedUpDistance {[&](double v, double& a, double& t){
        a = std::min(limits.accel, std::sqrt(jerk * v));
        t = a > 0 ? v / a + a / jerk : 0;
        return v * t / 2;
    }};
    double distance {std::abs(lastError)};
    double a {0}, t {0};
    peak = limits.speed;
    if(2 * speedUpDistance(peak, a, t) > distance){
        //too short to reach top speed: the highest peak whose speed-up and slow-down just fit (they grow with it)
        double low {0}, high {peak};
        for(int i {0}; i < 50; i++){
            double mid {(low + high) / 2};
            (2 * speedUpDistance(mid, a, t) < distance ? low : high) = mid;
        }
        peak = low;
    }
    double halfDistance {speedUpDistance(peak, a, t)};
    peakAccel = a;
    accelS = t;
    rampS = jerk > 0 ? peakAccel / jerk : 0;
    cruiseS = peak > 0 ? std::max(0.0, distance - 2 * halfDistance) / peak : 0;
    done = distance == 0 || peak <= 0;
}

TurnController::State TurnController::speedUp(double t) const {
    double holdS {std::max(0.0, accelS - 2 * rampS)};
    if(t <= rampS){
        return {jerk * t * t * t / 6, jerk * t * t / 2, jerk * t};
    }
    double p1 {jerk * rampS * rampS * rampS / 6};
    double v1 {jerk * rampS * rampS / 2};
    if(t <= rampS + holdS){
        double h {t - rampS};
        return {p1 + v1 * h + peakAccel * h * h / 2, v1 + peakAccel * h, peakAccel};
    }
    double p2 {p1 + v1 * holdS + peakAccel * holdS * holdS / 2};
    double v2 {v1 + peakAccel * holdS};
    double r {std::min(t, accelS) - rampS - holdS};
    return {p2 + v2 * r + peakAccel * r * r / 2 - jerk * r * r * r / 6, v2 + peakAccel * r - jerk * r * r / 2,
            peakAccel - jerk * r};
}

double TurnController::update(double thetaDeg, double rateDegPerS, const TurnModel& m, double feedbackWn, double dtS){
    elapsed += dtS;
    if(!done){
        double total {2 * accelS + cruiseS};
        State s {0, 0, 0};
        if(elapsed >= total){
            done = true;
        }
        else if(elapsed <= accelS){
            s = speedUp(elapsed);
        }
        else if(elapsed <= accelS + cruiseS){
            State top {speedUp(accelS)};
            s = {top.pos + peak * (elapsed - accelS), peak, 0};
        }
        else{
            //the slow-down is the speed-up played backward from the target
            State back {speedUp(total - elapsed)};
            double distance {std::abs(target - startPos)};
            s = {distance - back.pos, back.vel, -back.acc};
        }
        posRef = done ? target : startPos + sign * s.pos;
        vRef = sign * s.vel;
        aRef = sign * s.acc;
    }

    lastError = target - thetaDeg;
    double wn {feedbackWn};
    double kP {m.kA * wn * wn};
    double kD {settings.dScale * std::max(0.0, 2 * wn * m.kA - m.kV)};
    //the heading reads sensorLagS late, so judge it where it will be by now: without this, at 500 deg/s the lag looks
    //like a 10 deg tracking error and the P term fights the feedforward
    double predicted {thetaDeg + rateDegPerS * settings.sensorLagS};
    double predictedRate {rateDegPerS + aRef * settings.sensorLagS};
    //kS is part of the model's feedforward, so it only rides on the plan's motion. on the way down it fades out below
    //ksFadeDegPerS: switched off at once when the plan ended, it stepped the output by the whole kS (1.3-1.7 V in the
    //2026-11-02 RIGHT-tap logs) while the robot was still rolling. on the way up it stays whole to break friction
    double out {m.kV * vRef + m.kA * aRef + kP * (posRef - predicted) + kD * (vRef - predictedRate)};
    if(vRef != 0){
        bool slowing {elapsed > accelS + cruiseS};
        double share {slowing ? std::min(1.0, std::abs(vRef) / settings.ksFadeDegPerS) : 1.0};
        out += std::copysign(m.kS * share, vRef);
    }

    //once the plan has ended, P alone can't beat static friction within kS / kP of the target (about 6 deg with the
    //measured model), so the robot could stall short for good. this integral builds up until it breaks free, and
    //starts over each time the error changes side so it doesn't carry the robot past
    if(done && std::abs(lastError) < settings.endIntegralZoneDeg){
        if(lastError * endIntegral < 0){
            endIntegral = 0;
        }
        endIntegral = std::clamp(endIntegral + kP / settings.endIntegralTimeS * lastError * dtS, -settings.maxMv,
                                 settings.maxMv);
    }
    else{
        endIntegral = 0;
    }
    out += endIntegral;

    bool onTarget {done && std::abs(lastError) < settings.exitDeg && std::abs(rateDegPerS) < settings.exitRateDegPerS};
    settledForS = onTarget ? settledForS + dtS : 0;
    return std::clamp(out, -settings.maxMv, settings.maxMv);
}

bool TurnController::settled() const {
    return settledForS >= settings.exitS;
}
