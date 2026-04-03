#include "auto_parking/path_planner.h"
#include <cmath>
#include <algorithm>

namespace auto_parking {

std::vector<PathPoint> DiagonalPlanner::plan(
    Pose2D start,
    const ParkingSlot& target,
    VehicleParams params) {

    std::vector<PathPoint> full_path;

    // ---------------------------------------------------------------
    // Step 1: Calculate geometry
    // ---------------------------------------------------------------
    double R = params.wheelbase / std::tan(params.max_steering_angle);

    // The diagonal slot has an angle (e.g., 30-60 degrees from driving direction)
    // Use the slot's angle to determine the arc sweep needed
    double slot_angle = std::abs(target.angle);
    if (slot_angle <= 1e-6) {
        slot_angle = kPi / 4.0;  // default 45 degrees if not set
    }

    double forward_speed = 1.0;    // m/s
    double reverse_speed = -1.0;   // m/s

    // ---------------------------------------------------------------
    // Phase 1: Forward straight to approach position
    // ---------------------------------------------------------------
    // Calculate approach distance based on slot angle and turning radius
    // We need to position the vehicle so that the rear axle is at the
    // right distance from the slot entrance for a successful arc entry.
    double approach_dist = R * std::sin(slot_angle) + params.rear_overhang;

    // Distance from vehicle to slot entrance along the driving direction
    double dx_to_slot = target.center.x - start.position.x;
    double forward_offset = approach_dist - dx_to_slot;

    if (forward_offset > 0.1) {
        auto fwd = generateStraight(start, forward_offset, forward_speed, params);
        full_path.insert(full_path.end(), fwd.begin(), fwd.end());
    }

    Pose2D phase2_start = full_path.empty() ? start : full_path.back().pose;

    // ---------------------------------------------------------------
    // Phase 2: Reverse arc into the diagonal slot
    // ---------------------------------------------------------------
    // The arc angle should match the slot's diagonal angle
    // Steer right (negative angle), drive backward
    double arc_angle = slot_angle;

    // Clamp arc angle to a reasonable range
    constexpr double kMinArcAngle = 15.0 * kDeg2Rad;
    constexpr double kMaxArcAngle = 75.0 * kDeg2Rad;
    arc_angle = std::clamp(arc_angle, kMinArcAngle, kMaxArcAngle);

    auto arc = generateArc(phase2_start, R, -arc_angle, -1.0, reverse_speed, params);
    full_path.insert(full_path.end(), arc.begin(), arc.end());

    // ---------------------------------------------------------------
    // Phase 3: Reverse straight to slot center
    // ---------------------------------------------------------------
    Pose2D phase3_start = arc.empty() ? phase2_start : arc.back().pose;

    // Calculate remaining distance to the slot center
    double dx = target.center.x - phase3_start.position.x;
    double dy = target.center.y - phase3_start.position.y;

    // Project displacement onto the vehicle's current heading
    double remain_dist = dx * std::cos(phase3_start.yaw) +
                         dy * std::sin(phase3_start.yaw);

    if (std::abs(remain_dist) > 0.05) {
        // Speed magnitude scales with remaining distance for smoother tracking.
        // Keep it bounded to avoid overshoot and controller oscillations.
        const double abs_remain = std::abs(remain_dist);
        const double speed_mag = std::clamp(abs_remain * 0.2, 0.2, 0.5);
        const double adj_speed = (remain_dist > 0) ? speed_mag
                                                     : reverse_speed * speed_mag;
        auto straight = generateStraight(phase3_start, remain_dist, adj_speed, params);
        full_path.insert(full_path.end(), straight.begin(), straight.end());
    }

    return full_path;
}

}  // namespace auto_parking
