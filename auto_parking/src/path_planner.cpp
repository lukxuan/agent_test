#include "auto_parking/path_planner.h"
#include "auto_parking/vehicle_model.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace auto_parking {

// ---------------------------------------------------------------------------
// PathPlanner base class implementations
// ---------------------------------------------------------------------------

bool PathPlanner::validatePath(
    const std::vector<PathPoint>& path,
    VehicleParams params,
    const ParkingSlot& target) const {

    for (const auto& point : path) {
        if (checkPoseCollision(point.pose, params, target)) {
            return false;  // collision detected
        }
    }
    return true;  // path is collision-free
}

std::vector<PathPoint> PathPlanner::generateArc(
    Pose2D start, double radius, double angle,
    double direction, double speed, VehicleParams params) const {

    std::vector<PathPoint> result;

    // Guard against zero radius
    if (std::abs(radius) < 1e-6) {
        return result;
    }

    // Step size for angle discretization (radians per step)
    constexpr double kAngleStep = 0.05;

    // Determine number of steps and actual step size
    int num_steps = static_cast<int>(std::abs(angle) / kAngleStep);
    if (num_steps < 1) {
        num_steps = 1;
    }
    double actual_step = angle / static_cast<double>(num_steps);

    // Curvature = 1 / radius (signed)
    double curvature = 1.0 / radius;

    // Steering angle for this curvature: delta = atan(L / R)
    // But with the sign convention: positive angle = left turn
    double steering_angle = std::atan(params.wheelbase / std::abs(radius));
    if (angle < 0.0) {
        steering_angle = -steering_angle;  // right turn
    }

    Pose2D current = start;

    for (int i = 0; i <= num_steps; ++i) {
        PathPoint pt;
        pt.pose = current;
        pt.curvature = curvature;
        pt.speed = speed;
        result.push_back(pt);

        if (i < num_steps) {
            // Update position along the arc using bicycle model
            // direction: +1.0 = forward, -1.0 = reverse
            double arc_len = std::abs(actual_step) * std::abs(radius);
            double ds = actual_step * std::abs(radius);  // signed arc length

            // Position update using arc parametrization
            // If turning left (positive angle), center is to the left of the vehicle
            // If turning right (negative angle), center is to the right
            double turn_sign = (angle > 0.0) ? 1.0 : -1.0;

            // Center of the turning circle
            double cx = current.position.x + turn_sign * radius * (-std::sin(current.yaw));
            double cy = current.position.y + turn_sign * radius * (std::cos(current.yaw));

            // Angle on the circle relative to center
            double theta_start = std::atan2(current.position.y - cy,
                                             current.position.x - cx);
            double theta_new = theta_start + actual_step;

            // New position
            current.position.x = cx + std::abs(radius) * std::cos(theta_new);
            current.position.y = cy + std::abs(radius) * std::sin(theta_new);

            // New heading
            current.yaw = current.yaw + actual_step;

            // Normalize yaw
            current.yaw = normalizeAngle(current.yaw);
        }
    }

    return result;
}

std::vector<PathPoint> PathPlanner::generateStraight(
    Pose2D start, double distance,
    double speed, VehicleParams params) const {

    std::vector<PathPoint> result;

    // Step size for distance discretization (meters per step)
    constexpr double kDistStep = 0.1;

    int num_steps = static_cast<int>(std::abs(distance) / kDistStep);
    if (num_steps < 1) {
        num_steps = 1;
    }
    double actual_step = distance / static_cast<double>(num_steps);

    Pose2D current = start;

    for (int i = 0; i <= num_steps; ++i) {
        PathPoint pt;
        pt.pose = current;
        pt.curvature = 0.0;  // straight line
        pt.speed = speed;
        result.push_back(pt);

        if (i < num_steps) {
            // Update position along straight line
            current.position.x += actual_step * std::cos(current.yaw);
            current.position.y += actual_step * std::sin(current.yaw);
            // Yaw stays the same for straight motion
        }
    }

    return result;
}

void PathPlanner::transformToAbsolute(std::vector<PathPoint>& path,
                                       Pose2D origin) const {
    for (auto& point : path) {
        Point2D local = point.pose.position;
        Point2D absolute;

        // Rotate local point by origin yaw, then translate
        double cos_yaw = std::cos(origin.yaw);
        double sin_yaw = std::sin(origin.yaw);
        absolute.x = origin.position.x + local.x * cos_yaw - local.y * sin_yaw;
        absolute.y = origin.position.y + local.x * sin_yaw + local.y * cos_yaw;

        point.pose.position = absolute;
        point.pose.yaw = normalizeAngle(point.pose.yaw + origin.yaw);
    }
}

bool PathPlanner::checkPoseCollision(Pose2D pose, VehicleParams params,
                                      const ParkingSlot& target) const {

    // Get the vehicle footprint corners in world frame
    // We compute them inline here to avoid needing a VehicleModel instance
    double rear = -params.rear_overhang;
    double front = params.wheelbase + params.front_overhang;
    double half_w = params.width / 2.0;

    // Vehicle corners in local frame (rear-left, front-left, front-right, rear-right)
    std::vector<Point2D> local_corners = {
        {rear,  half_w},
        {front, half_w},
        {front, -half_w},
        {rear,  -half_w}
    };

    // Transform to world frame
    std::vector<Point2D> world_corners;
    world_corners.reserve(4);
    double cos_yaw = std::cos(pose.yaw);
    double sin_yaw = std::sin(pose.yaw);

    for (const auto& c : local_corners) {
        Point2D wc;
        wc.x = pose.position.x + c.x * cos_yaw - c.y * sin_yaw;
        wc.y = pose.position.y + c.x * sin_yaw + c.y * cos_yaw;
        world_corners.push_back(wc);
    }

    // Build slot boundary min/max for simple AABB collision check
    double slot_min_x = std::min({target.corners[0].x, target.corners[1].x,
                                   target.corners[2].x, target.corners[3].x});
    double slot_max_x = std::max({target.corners[0].x, target.corners[1].x,
                                   target.corners[2].x, target.corners[3].x});
    double slot_min_y = std::min({target.corners[0].y, target.corners[1].y,
                                   target.corners[2].y, target.corners[3].y});
    double slot_max_y = std::max({target.corners[0].y, target.corners[1].y,
                                   target.corners[2].y, target.corners[3].y});

    // Add a small margin around the slot boundaries to detect proximity violations
    constexpr double kMargin = 0.1;

    // For parking, we check that no vehicle corner is outside the slot boundary
    // (beyond the margin). This prevents the vehicle from hitting obstacles
    // outside the parking slot.
    for (const auto& corner : world_corners) {
        if (corner.x < slot_min_x - kMargin ||
            corner.x > slot_max_x + kMargin ||
            corner.y < slot_min_y - kMargin ||
            corner.y > slot_max_y + kMargin) {
            return true;  // collision: vehicle extends beyond slot boundaries
        }
    }

    return false;  // no collision
}

size_t PathPlanner::findClosestPoint(const std::vector<PathPoint>& path,
                                      Pose2D pose) const {
    if (path.empty()) {
        return 0;
    }

    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    for (size_t i = 0; i < path.size(); ++i) {
        double dist = pose.position.distanceTo(path[i].pose.position);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    return closest_idx;
}

// ---------------------------------------------------------------------------
// Factory function
// ---------------------------------------------------------------------------

std::unique_ptr<PathPlanner> createPlanner(ParkingSlot::Type type) {
    switch (type) {
        case ParkingSlot::Type::PARALLEL:
            return std::make_unique<ParallelPlanner>();
        case ParkingSlot::Type::PERPENDICULAR:
            return std::make_unique<PerpendicularPlanner>();
        case ParkingSlot::Type::DIAGONAR:
            return std::make_unique<DiagonalPlanner>();
        default:
            return std::make_unique<ParallelPlanner>();
    }
}

}  // namespace auto_parking
