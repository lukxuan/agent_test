#include "auto_parking/vehicle_model.h"
#include <limits>
#include <algorithm>

namespace auto_parking {

VehicleModel::VehicleModel(VehicleParams params)
    : params_(std::move(params)) {}

Pose2D VehicleModel::update(Pose2D current, ControlCommand cmd, double dt) const {
    // Clamp inputs to vehicle limits
    double delta = clampSteering(cmd.steering_angle);
    double v = clampSpeed(cmd.speed);

    // Bicycle model kinematics (Euler integration)
    // dx = v * cos(yaw) * dt
    // dy = v * sin(yaw) * dt
    // dyaw = v * tan(delta) / wheelbase * dt
    Pose2D next;
    next.position.x = current.position.x + v * std::cos(current.yaw) * dt;
    next.position.y = current.position.y + v * std::sin(current.yaw) * dt;
    next.yaw = current.yaw + (v * std::tan(delta) / params_.wheelbase) * dt;

    return next;
}

const VehicleParams& VehicleModel::getParams() const {
    return params_;
}

double VehicleModel::turningRadius(double steering_angle) const {
    double tan_delta = std::tan(steering_angle);
    if (std::abs(tan_delta) < 1e-6) {
        return std::numeric_limits<double>::infinity();
    }
    return params_.wheelbase / tan_delta;
}

double VehicleModel::clampSteering(double steering_angle) const {
    return std::clamp(steering_angle, -params_.max_steering_angle, params_.max_steering_angle);
}

double VehicleModel::clampSpeed(double speed) const {
    if (speed >= 0.0) {
        return std::clamp(speed, 0.0, params_.max_speed);
    } else {
        return std::clamp(speed, -params_.max_reverse_speed, 0.0);
    }
}

std::vector<Point2D> VehicleModel::getFootprint(Pose2D pose) const {
    // Vehicle corners in vehicle-local frame (rear axle as origin):
    //   - rear_overhang behind the rear axle
    //   - (wheelbase + front_overhang) ahead of the rear axle
    //   - half width on each side
    double rear = -params_.rear_overhang;
    double front = params_.wheelbase + params_.front_overhang;
    double half_w = params_.width / 2.0;

    // 4 corners in local frame: rear-left, front-left, front-right, rear-right
    std::vector<Point2D> local_corners = {
        {rear,  half_w},
        {front, half_w},
        {front, -half_w},
        {rear,  -half_w}
    };

    // Transform each corner to world frame
    std::vector<Point2D> world_corners;
    world_corners.reserve(4);
    for (const auto& corner : local_corners) {
        Point2D wc;
        double cos_yaw = std::cos(pose.yaw);
        double sin_yaw = std::sin(pose.yaw);
        wc.x = pose.position.x + corner.x * cos_yaw - corner.y * sin_yaw;
        wc.y = pose.position.y + corner.x * sin_yaw + corner.y * cos_yaw;
        world_corners.push_back(wc);
    }

    return world_corners;
}

}  // namespace auto_parking
