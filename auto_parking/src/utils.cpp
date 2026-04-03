#include "auto_parking/types.h"
#include <cmath>
#include <algorithm>

namespace auto_parking {

std::string parkingStateToString(ParkingState state) {
    switch (state) {
        case ParkingState::IDLE:       return "IDLE";
        case ParkingState::SEARCHING:  return "SEARCHING";
        case ParkingState::PLANNING:   return "PLANNING";
        case ParkingState::EXECUTING:  return "EXECUTING";
        case ParkingState::COMPLETED:  return "COMPLETED";
        case ParkingState::FAILED:     return "FAILED";
        default:                       return "UNKNOWN";
    }
}

double normalizeAngle(double angle) {
    while (angle > kPi)  angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

Point2D rotatePoint(const Point2D& p, double angle) {
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);
    return Point2D{
        p.x * cos_a - p.y * sin_a,
        p.x * sin_a + p.y * cos_a
    };
}

Point2D transformPoint(const Point2D& p, const Pose2D& frame) {
    Point2D rotated = rotatePoint(p, frame.yaw);
    return Point2D{
        rotated.x + frame.position.x,
        rotated.y + frame.position.y
    };
}

bool checkCollision(const Pose2D& vehicle_pose, const VehicleParams& vehicle,
                    const std::vector<Point2D>& obstacles) {
    // Build vehicle AABB in world frame
    double rear = -vehicle.rear_overhang;
    double front = vehicle.wheelbase + vehicle.front_overhang;
    double half_w = vehicle.width / 2.0;

    // Vehicle corners in local frame
    std::vector<Point2D> local_corners = {
        {rear,  half_w}, {front, half_w},
        {front, -half_w}, {rear,  -half_w}
    };

    double cos_yaw = std::cos(vehicle_pose.yaw);
    double sin_yaw = std::sin(vehicle_pose.yaw);

    // Build vehicle AABB
    double vx_min = std::numeric_limits<double>::max();
    double vx_max = std::numeric_limits<double>::lowest();
    double vy_min = std::numeric_limits<double>::max();
    double vy_max = std::numeric_limits<double>::lowest();

    for (const auto& c : local_corners) {
        double wx = vehicle_pose.position.x + c.x * cos_yaw - c.y * sin_yaw;
        double wy = vehicle_pose.position.y + c.x * sin_yaw + c.y * cos_yaw;
        vx_min = std::min(vx_min, wx);
        vx_max = std::max(vx_max, wx);
        vy_min = std::min(vy_min, wy);
        vy_max = std::max(vy_max, wy);
    }

    // Check each obstacle point against vehicle AABB
    for (const auto& obs : obstacles) {
        if (obs.x >= vx_min && obs.x <= vx_max &&
            obs.y >= vy_min && obs.y <= vy_max) {
            return true;  // collision
        }
    }

    return false;  // no collision
}

}  // namespace auto_parking
