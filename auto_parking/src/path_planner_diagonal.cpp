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
    double slot_angle = target.angle;
    if (slot_angle <= 0.0) {
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
    arc_angle = std::clamp(arc_angle, 15.0 * kDeg2Rad, 75.0 * kDeg2Rad);

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
        double adj_speed = (remain_dist > 0) ? 0.5 : reverse_speed * 0.5;
        auto straight = generateStraight(phase3_start, remain_dist, adj_speed, params);
        full_path.insert(full_path.end(), straight.begin(), straight.end());
    }

    // ---------------------------------------------------------------
    // Phase 4: Optional small adjustment to correct heading alignment
    // ---------------------------------------------------------------
    Pose2D phase4_start = full_path.empty() ? start : full_path.back().pose;

    // Check heading error relative to slot orientation
    double heading_error = normalizeAngle(target.angle + kPi - phase4_start.yaw);

    // If heading error is significant, do a small corrective arc
    if (std::abs(heading_error) > 2.0 * kDeg2Rad) {
        // Clamp correction to avoid large movements
        double correction = std::clamp(heading_error * 0.5, -0.3, 0.3);
        double corr_speed = (correction > 0) ? forward_speed * 0.3 : reverse_speed * 0.3;
        double corr_dir = (correction > 0) ? 1.0 : -1.0;

        auto corr1 = generateArc(phase4_start, R, correction,
                                  corr_dir, corr_speed, params);
        full_path.insert(full_path.end(), corr1.begin(), corr1.end());

        Pose2D corr1_end = corr1.empty() ? phase4_start : corr1.back().pose;
        // Undo the correction to straighten the vehicle
        auto corr2 = generateArc(corr1_end, R, -correction,
                                  -corr_dir, -corr_speed, params);
        full_path.insert(full_path.end(), corr2.begin(), corr2.end());
    }

    return full_path;
}

}  // namespace auto_parking
