#pragma once

//turning model V = kS + kV * speed + kA * acceleration (mV, deg/s, deg/s^2)
struct TurnModel{
    double kS, kV, kA;
};

//limits on the planned motion (deg/s, deg/s^2, deg/s^3)
struct TurnLimits{
    double speed, accel, jerk;
};

//fixed settings
struct TurnSettings{
    double dScale;              //D = dScale (2 kA wn - kV), see update
    double sensorLagS;          //how late the heading and its rate arrive, judged forward by this much
    double endIntegralTimeS;    //end integral: grows by P error / this per second (see update)
    double endIntegralZoneDeg;  //and only runs this close to the target
    double ksFadeDegPerS;       //kS fades out below this plan speed on the way down (see update)
    double exitDeg;             //settled: within exitDeg, slower than exitRateDegPerS, for exitS
    double exitRateDegPerS;
    double exitS;
    double maxMv;               //output limit (12000 = 12 V)
};

//profiled in-place turn controller. self-contained: no config, motors, sensors or LemLib, everything comes in through
//the settings, start() and update(), so it also runs on a PC (test/turn_controller_sim.cpp).
//start() plans a rest-to-rest S-curve in time: jerk +j, hold the acceleration, jerk -j up to the peak speed, cruise, and
//the mirror image down, every piece constant jerk, so the acceleration never steps and ends at 0 right at the target.
//update() reads the plan at the elapsed time. output = model feedforward on the plan + critically damped PD on the plan
//error, then a small integral once the plan has ended to finish against friction. the model and feedbackWn are passed
//to every update, so the caller can change the gains mid-turn (Drivetrain schedules them by elevator height, util/turn_schedule.h); the
//limits are fixed when the turn is planned
class TurnController{
public:
    explicit TurnController(const TurnSettings& settings);

    //thetaDeg / targetDeg in the same continuous frame (LemLib's pose theta)
    void start(double thetaDeg, double targetDeg, const TurnLimits& limits);

    //one control step: returns the turn voltage in mV, positive = clockwise (left forward, right back). the feedback is
    //a critically damped PD from the model at natural frequency feedbackWn (rad/s): P = kA wn^2, D = dScale (2 kA wn - kV)
    double update(double thetaDeg, double rateDegPerS, const TurnModel& model, double feedbackWn, double dtS);

    //the plan has ended and the robot has stayed on the target (settings exit*)
    bool settled() const;

    //true while the plan is still moving
    bool profiling() const { return !done; }

    //the plan's length, for printouts
    double plannedS() const { return 2 * accelS + cruiseS; }

    double error() const { return lastError; }

    //the plan's acceleration, for printouts and the sim
    double referenceAccel() const { return aRef; }

private:
    struct State{
        double pos, vel, acc;
    };

    //the speed-up half at t (0 to accelS): distance, speed and acceleration from rest
    State speedUp(double t) const;

    TurnSettings settings;
    double startPos {0};
    double target {0};
    double sign {1};
    //the plan: peak speed, peak acceleration, jerk, length of each ramp, of the speed-up half, and of the cruise
    double peak {0};
    double peakAccel {0};
    double jerk {1};
    double rampS {0};
    double accelS {0};
    double cruiseS {0};
    double elapsed {0};
    double posRef {0};
    double vRef {0};
    double aRef {0};
    bool done {true};
    double lastError {0};
    double endIntegral {0};
    double settledForS {0};
};
