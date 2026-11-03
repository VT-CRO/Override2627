#include "util/turn_schedule.h"
#include "config/drivetrain.h"
#include <algorithm>
#include <cstddef>

namespace cfg = config::drivetrain;

namespace {
constexpr double maxMv {12000.0};

double blend(double low, double high, double height){
    return low + (high - low) * std::clamp(height, 0.0, 1.0);
}
}

TurnModel turnModelAt(double height){
    const auto& at {cfg::turnModelHeights};
    height = std::clamp(height, 0.0, 1.0);
    std::size_t i {1};
    while(i + 1 < at.size() && height > at[i]){
        i++;
    }
    double t {std::clamp((height - at[i - 1]) / (at[i] - at[i - 1]), 0.0, 1.0)};
    auto lerp {[t, i](const auto& v){ return v[i - 1] + (v[i] - v[i - 1]) * t; }};
    return {lerp(cfg::turnKs), lerp(cfg::turnKv), lerp(cfg::turnKa)};
}

TurnLimits turnLimitsAt(double height, const TurnModel& m, double speedFraction){
    return {
        std::clamp(speedFraction, 0.05, 1.0) * cfg::turnSpeedFraction * (maxMv - m.kS) / m.kV,
        std::min(cfg::turnAccelFraction * (maxMv - m.kS) / m.kA, blend(cfg::turnMaxAccelLow, cfg::turnMaxAccelHigh, height)),
        blend(cfg::turnMaxJerkLow, cfg::turnMaxJerkHigh, height),
    };
}

double turnFeedbackWnAt(double height){
    //straight-line over the measured span, intake height to top (the intake value below it)
    double low {cfg::turnModelHeights.front()};
    return blend(cfg::turnFeedbackWnLow, cfg::turnFeedbackWnHigh, (height - low) / (1.0 - low));
}

TurnSettings turnSettings(){
    return {
        cfg::turnDScale,
        cfg::turnSensorLagS,
        cfg::turnEndIntegralTimeS,
        cfg::turnEndIntegralZoneDeg,
        cfg::turnKsFadeDegPerS,
        cfg::turnExitDeg,
        cfg::turnExitRateDegPerS,
        cfg::turnExitMs / 1000.0,
        maxMv,
    };
}
