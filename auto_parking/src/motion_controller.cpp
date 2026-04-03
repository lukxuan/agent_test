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

    // Find the closest path point to current position for tracking progress.
    // - Use squared distances (avoid sqrt in distanceTo()).
    // - For long paths, scan a local window around the previous closest point.
    // - Near the end, fall back to full scan for reliable "finished" detection.
    const size_t n = path.size();
    closest_idx_ = (closest_idx_ < n) ? closest_idx_ : 0;

    auto distSq = [&](const Point2D& a, const Point2D& b) -> double {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        return dx * dx + dy * dy;
    };

    const Point2D& end_pos = path.back().pose.position;
    const double dx_end = current_pose.position.x - end_pos.x;
    const double dy_end = current_pose.position.y - end_pos.y;
    const double dist_to_end_sq = dx_end * dx_end + dy_end * dy_end;

    constexpr size_t kFullScanLimit = 200;
    constexpr size_t kBack = 30;
    constexpr size_t kAhead = 150;
    constexpr double kEndThreshold = 0.1; // meters
    constexpr double kEndThresholdSq = kEndThreshold * kEndThreshold;
    constexpr double kNearEndFallbackDist = 0.6; // meters
    constexpr double kNearEndFallbackDistSq =
        kNearEndFallbackDist * kNearEndFallbackDist;

    size_t closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();

    const bool do_full_scan =
        (n <= kFullScanLimit) || (dist_to_end_sq <= kNearEndFallbackDistSq);

    if (do_full_scan) {
        for (size_t i = 0; i < n; ++i) {
            const double d2 = distSq(current_pose.position, path[i].pose.position);
            if (d2 < min_dist_sq) {
                min_dist_sq = d2;
                closest_idx = i;
            }
        }
    } else {
        const size_t start = (closest_idx_ > kBack) ? (closest_idx_ - kBack) : 0;
        const size_t end = std::min(n - 1, closest_idx_ + kAhead);
        for (size_t i = start; i <= end; ++i) {
            const double d2 = distSq(current_pose.position, path[i].pose.position);
            if (d2 < min_dist_sq) {
                min_dist_sq = d2;
                closest_idx = i;
            }
        }
    }

    closest_idx_ = closest_idx;
    lookahead_start_idx_ = closest_idx_;

    double target_speed = path[closest_idx_].speed;
    cmd.curvature = path[closest_idx_].curvature;

    cmd.steering_angle = purePursuit(current_pose, path);
    cmd.speed = pidControl(target_speed, current_speed, 0.01);

    if (dist_to_end_sq < kEndThresholdSq && closest_idx_ >= n - 2) {
        finished_ = true;
    }

    return cmd;
}

void MotionController::reset() {
    integral_error_ = 0.0;
    prev_error_ = 0.0;
    prev_dt_ = 0.01;
    closest_idx_ = 0;
    lookahead_start_idx_ = 0;
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
    size_t target_idx = findLookaheadPoint(current_pose, path);

    if (target_idx >= path.size()) {
        return 0.0;
    }

    const Point2D& target = path[target_idx].pose.position;

    double dx = target.x - current_pose.position.x;
    double dy = target.y - current_pose.position.y;

    double cos_yaw = std::cos(current_pose.yaw);
    double sin_yaw = std::sin(current_pose.yaw);

    double local_x =  dx * cos_yaw + dy * sin_yaw;
    double local_y = -dx * sin_yaw + dy * cos_yaw;

    double ld = std::sqrt(local_x * local_x + local_y * local_y);

    if (ld < 1e-6) {
        return 0.0;
    }

    double alpha = std::atan2(local_y, local_x);

    constexpr double L = 2.8;
    double delta = std::atan(2.0 * L * std::sin(alpha) / ld);

    constexpr double kMaxSteering = 35.0 * kDeg2Rad;
    delta = std::clamp(delta, -kMaxSteering, kMaxSteering);

    return delta;
}

double MotionController::pidControl(double target_speed, double current_speed,
                                    double dt) {
    double error = target_speed - current_speed;

    integral_error_ += error * dt;
    integral_error_ = std::clamp(integral_error_,
                                  -pid_params_.integral_max,
                                   pid_params_.integral_max);

    double derivative = 0.0;
    if (prev_dt_ > 1e-6) {
        derivative = (error - prev_error_) / prev_dt_;
    }

    double output = pid_params_.kp * error
                  + pid_params_.ki * integral_error_
                  + pid_params_.kd * derivative;

    output = std::clamp(output, -pid_params_.output_max, pid_params_.output_max);

    prev_error_ = error;
    prev_dt_ = dt;

    return output;
}

size_t MotionController::findLookaheadPoint(Pose2D current_pose,
                                             const std::vector<PathPoint>& path) {
    if (path.empty()) {
        return 0;
    }

    double speed = 0.0;
    if (closest_idx_ < path.size()) {
        speed = std::abs(path[closest_idx_].speed);
    }
    double lookahead_dist = lookahead_base_ + lookahead_k_ * speed;
    const double lookahead_dist_sq = lookahead_dist * lookahead_dist;

    lookahead_start_idx_ = std::min(lookahead_start_idx_, path.size() - 1);
    for (size_t i = lookahead_start_idx_; i < path.size(); ++i) {
        const Point2D& p = path[i].pose.position;
        const double dx = current_pose.position.x - p.x;
        const double dy = current_pose.position.y - p.y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq >= lookahead_dist_sq) {
            lookahead_start_idx_ = i;
            return i;
        }
    }

    size_t last_idx = path.size() - 1;
    lookahead_start_idx_ = last_idx;
    return last_idx;
}

}  // namespace auto_parking
