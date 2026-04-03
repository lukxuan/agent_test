#include "auto_parking/path_planner.h"
#include <cmath>
#include <algorithm>

namespace auto_parking {

std::vector<PathPoint> PerpendicularPlanner::plan(
    Pose2D start,
    const ParkingSlot& target,
    VehicleParams params) {

    std::vector<PathPoint> full_path;

    // ---------------------------------------------------------------
    // Step 1: Calculate geometry
    // ---------------------------------------------------------------
    double R = params.wheelbase / std::tan(params.max_steering_angle);

    // Target position: center of the parking slot
    // For perpendicular parking, the vehicle needs to end up inside the
    // slot, aligned with the slot orientation (typically perpendicular to
    // the driving direction).

    // Calculate the approach distance
    // The vehicle needs to be positioned so that the rear axle is
    // approximately R away from the slot entrance to allow the arc to fit.
    double approach_dist = params.rear_overhang + R;

    // Calculate how far forward/backward the vehicle needs to go
    // to reach the approach position relative to the slot center
    double dx_to_slot = target.center.x - start.position.x;
    double forward_offset = approach_dist - dx_to_slot;

    double forward_speed = 1.0;    // m/s
    double reverse_speed = -1.0;   // m/s

    // ---------------------------------------------------------------
    // Phase 1: Forward straight to align rear axle with approach position
    // ---------------------------------------------------------------
    if (forward_offset > 0.1) {
        auto fwd = generateStraight(start, forward_offset, forward_speed, params);
        full_path.insert(full_path.end(), fwd.begin(), fwd.end());
    }

    Pose2D phase2_start = full_path.empty() ? start : full_path.back().pose;

    // ---------------------------------------------------------------
    // Phase 2: Reverse straight to create room for the turning arc
    // ---------------------------------------------------------------
    // Move backward a short distance to create space for the arc maneuver
    double prep_reverse_dist = 0.5;
    auto prep_rev = generateStraight(phase2_start, -prep_reverse_dist,
                                      reverse_speed, params);
    full_path.insert(full_path.end(), prep_rev.begin(), prep_rev.end());

    // ---------------------------------------------------------------
    // Phase 3: Reverse right arc into slot (~90 degrees)
    // ---------------------------------------------------------------
    Pose2D phase3_start = prep_rev.empty() ? phase2_start : prep_rev.back().pose;

    // Arc angle: approximately 90 degrees (pi/2) to turn into perpendicular slot
    double arc_angle = kPi / 2.0;

    // Steer right (negative angle = right turn), drive backward
    auto arc = generateArc(phase3_start, R, -arc_angle, -1.0, reverse_speed, params);
    full_path.insert(full_path.end(), arc.begin(), arc.end());

    // ---------------------------------------------------------------
    // Phase 4: Reverse straight into slot center
    // ---------------------------------------------------------------
    Pose2D phase4_start = arc.empty() ? phase3_start : arc.back().pose;

    // Calculate remaining distance to the slot center
    double dx = target.center.x - phase4_start.position.x;
    double dy = target.center.y - phase4_start.position.y;

    // Project onto the vehicle's new heading (should be pointing into the slot)
    double remain_dist = dx * std::cos(phase4_start.yaw) +
                         dy * std::sin(phase4_start.yaw);

    // For perpendicular parking, after the 90-degree arc, the vehicle
    // is roughly aligned with the slot. Drive straight to center.
    // If remain_dist is positive (we need to go forward) or negative (reverse),
    // adjust accordingly.
    if (std::abs(remain_dist) > 0.05) {
        double adj_speed = (remain_dist > 0) ? 0.5 : reverse_speed * 0.5;
        auto straight = generateStraight(phase4_start, remain_dist, adj_speed, params);
        full_path.insert(full_path.end(), straight.begin(), straight.end());
    }

    // ---------------------------------------------------------------
    // Phase 5: Optional small adjustment to center laterally
    // ---------------------------------------------------------------
    Pose2D phase5_start = full_path.empty() ? start : full_path.back().pose;

    // Check lateral offset to slot center
    double lat_dx = target.center.x - phase5_start.position.x;
    double lat_dy = target.center.y - phase5_start.position.y;

    // Lateral error is the component perpendicular to the vehicle heading
    double lateral_error = -lat_dx * std::sin(phase5_start.yaw) +
                            lat_dy * std::cos(phase5_start.yaw);

    // If lateral error is significant, do a small forward+reverse adjustment
    if (std::abs(lateral_error) > 0.05) {
        // Small forward movement with steering to correct lateral position
        double correction_angle = std::clamp(lateral_error * 0.3, -0.2, 0.2);
        auto corr_arc = generateArc(phase5_start, R, correction_angle,
                                     1.0, forward_speed * 0.5, params);
        full_path.insert(full_path.end(), corr_arc.begin(), corr_arc.end());

        Pose2D corr_end = corr_arc.empty() ? phase5_start : corr_arc.back().pose;
        // Reverse back to straighten out
        auto corr_back = generateArc(corr_end, R, -correction_angle,
                                      -1.0, reverse_speed * 0.5, params);
        full_path.insert(full_path.end(), corr_back.begin(), corr_back.end());
    }

    return full_path;
}

}  // namespace auto_parking
