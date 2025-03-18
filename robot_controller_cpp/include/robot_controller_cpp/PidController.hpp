#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <chrono>

class PidController
{
public:
    // Constructor
    PidController(double Kp, double Ki, double Kd);

    // Compute PID output
    double compute(double error, std::chrono::steady_clock::time_point current_time);

private:
    // PID coefficients
    double Kp_;
    double Ki_;
    double Kd_;

    // PID state
    double previous_error_;
    double integral_;
    std::chrono::steady_clock::time_point last_time_;
};

#endif // PID_CONTROLLER_HPP
