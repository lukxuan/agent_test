#include "auto_parking/motion_controller.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace auto_parking {

MotionController::MotionController(double lookahead_base, double lookahead_k,
                                   PIDParams pid_params)
    : lookahead_base_(lookahead_base)
    , lookahead_k_(lookahead_k)
    , pid_params_(pid_params) {}

ControlCommand MotionController::compute(Pose2D current_pose,
                                         const std::vector<PathPoint>& path,
                                         double current_speed) {
    ControlCommand cmd;

    // If path is empty or already finished, return zero command
    if (path.empty() || finished_) {
        finished_ = true;
        return cmd;
    }

    // Find the closest path point to current position for tracking progress
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i) {
        double dist = current_pose.position.distanceTo(path[i].pose.position);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // Get target speed from the closest path point
    double target_speed = path[closest_idx].speed;

    // Get curvature from the closest path point
    cmd.curvature = path[closest_idx].curvature;

    // Lateral control via pure pursuit
    cmd.steering_angle = purePursuit(current_pose, path);

    // Longitudinal control via PID
    cmd.speed = pidControl(target_speed, current_speed, 0.01);

    // Check if we've reached the last point on the path (distance < 0.1m)
    double dist_to_end = current_pose.position.distanceTo(
        path.back().pose.position);
    if (dist_to_end < 0.1 && closest_idx >= path.size() - 2) {
        finished_ = true;
    }

    return cmd;
}

void MotionController::reset() {
    integral_error_ = 0.0;
    prev_error_ = 0.0;
    prev_dt_ = 0.01;
    current_target_idx_ = 0;
    finished_ = false;
}

bool MotionController::isFinished() const {
    return finished_;
}

void MotionController::setPIDParams(PIDParams params) {
    pid_params_ = params;
}

void MotionController::setLookahead(double base, double k) {
    lookahead_base_ = base;
    lookahead_k_ = k;
}

double MotionController::purePursuit(Pose2D current_pose,
                                     const std::vector<PathPoint>& path) {
    // Get the lookahead target point index
    size_t target_idx = findLookaheadPoint(current_pose, path);

    // If no valid target point, return 0 (no steering)
    if (target_idx >= path.size()) {
        return 0.0;
    }

    const Point2D& target = path[target_idx].pose.position;

    // Transform target point to vehicle frame
    double dx = target.x - current_pose.position.x;
    double dy = target.y - current_pose.position.y;

    double cos_yaw = std::cos(current_pose.yaw);
    double sin_yaw = std::sin(current_pose.yaw);

    double local_x =  dx * cos_yaw + dy * sin_yaw;
    double local_y = -dx * sin_yaw + dy * cos_yaw;

    // Calculate lateral distance to target
    double ld = std::sqrt(local_x * local_x + local_y * local_y);

    // Guard against zero lookahead distance
    if (ld < 1e-6) {
        return 0.0;
    }

    // Calculate heading to target in vehicle frame
    double alpha = std::atan2(local_y, local_x);

    // Default wheelbase for pure pursuit
    constexpr double L = 2.8;

    // Steering angle: delta = atan(2 * L * sin(alpha) / ld)
    double delta = std::atan(2.0 * L * std::sin(alpha) / ld);

    // Clamp steering to [-35 degrees, +35 degrees]
    constexpr double kMaxSteering = 35.0 * kDeg2Rad;
    delta = std::clamp(delta, -kMaxSteering, kMaxSteering);

    return delta;
}

double MotionController::pidControl(double target_speed, double current_speed,
                                    double dt) {
    double error = target_speed - current_speed;

    // Integral with anti-windup clamping
    integral_error_ += error * dt;
    integral_error_ = std::clamp(integral_error_,
                                  -pid_params_.integral_max,
                                   pid_params_.integral_max);

    // Derivative
    double derivative = 0.0;
    if (prev_dt_ > 1e-6) {
        derivative = (error - prev_error_) / prev_dt_;
    }

    // PID output
    double output = pid_params_.kp * error
                  + pid_params_.ki * integral_error_
                  + pid_params_.kd * derivative;

    // Clamp output
    output = std::clamp(output, -pid_params_.output_max, pid_params_.output_max);

    // Update state
    prev_error_ = error;
    prev_dt_ = dt;

    return output;
}

size_t MotionController::findLookaheadPoint(Pose2D current_pose,
                                             const std::vector<PathPoint>& path) {
    if (path.empty()) {
        return 0;
    }

    // Compute adaptive lookahead distance
    // Use speed from the current closest point as the speed estimate
    double speed = 0.0;
    if (current_target_idx_ < path.size()) {
        speed = std::abs(path[current_target_idx_].speed);
    }
    double lookahead_dist = lookahead_base_ + lookahead_k_ * speed;

    // Starting from current_target_idx_, find the first path point
    // whose distance from current_pose >= lookahead_dist
    for (size_t i = current_target_idx_; i < path.size(); ++i) {
        double dist = current_pose.position.distanceTo(path[i].pose.position);
        if (dist >= lookahead_dist) {
            current_target_idx_ = i;
            return i;
        }
    }

    // If no point found beyond lookahead distance, use the last point
    size_t last_idx = path.size() - 1;
    current_target_idx_ = last_idx;
    return last_idx;
}

}  // namespace auto_parking
