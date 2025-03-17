#include "PidController.hpp"
#include <chrono>

PidController::PidController(double Kp, double Ki, double Kd)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), previous_error_(0.0), integral_(0.0)
{
    last_time_ = std::chrono::steady_clock::now(); // Initialize time
}

double PidController::compute(double error)
{
    // Get current time
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time_;
    double dt = elapsed_time.count(); // Duration in seconds

    // Prevent division by zero
    if (dt <= 0.0)
    {
        dt = 1e-6; // Small value
    }

    // Proportional term
    double P = Kp_ * error;

    // Integral term
    integral_ += error * dt;
    double I = Ki_ * integral_;

    // Derivative term
    double derivative = (error - previous_error_) / dt;
    double D = Kd_ * derivative;

    // Save error and time for next iteration
    previous_error_ = error;
    last_time_ = current_time;

    // PID output
    return P + I + D;
}
