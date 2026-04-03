#pragma once
#include "types.h"

namespace auto_parking {

struct PIDParams {
    double kp = 1.0;
    double ki = 0.1;
    double kd = 0.05;
    double integral_max = 2.0;   // anti-windup
    double output_max = 5.0;     // max output
};

class MotionController {
public:
    MotionController(double lookahead_base = 2.0, double lookahead_k = 0.8,
                     PIDParams pid_params = {});

    // Compute control command to follow the path
    ControlCommand compute(Pose2D current_pose,
                          const std::vector<PathPoint>& path,
                          double current_speed);

    // Reset controller state (call when starting new path)
    void reset();

    // Check if vehicle has reached end of path
    bool isFinished() const;

    // Set PID parameters
    void setPIDParams(PIDParams params);

    // Set lookahead distance parameters
    void setLookahead(double base, double k);

private:
    // Pure Pursuit lateral control
    double purePursuit(Pose2D current_pose,
                       const std::vector<PathPoint>& path);

    // PID longitudinal control
    double pidControl(double target_speed, double current_speed, double dt);

    // Find the lookahead target point on the path
    size_t findLookaheadPoint(Pose2D current_pose,
                              const std::vector<PathPoint>& path);

    double lookahead_base_;
    double lookahead_k_;
    PIDParams pid_params_;

    // PID state
    double integral_error_ = 0.0;
    double prev_error_ = 0.0;
    double prev_dt_ = 0.01;

    // State tracking
    size_t current_target_idx_ = 0;
    bool finished_ = false;
};

}  // namespace auto_parking
