#include "auto_parking/path_planner.h"
#include <cmath>
#include <algorithm>

namespace auto_parking {

std::vector<PathPoint> ParallelPlanner::plan(
    Pose2D start,
    const ParkingSlot& target,
    VehicleParams params) {

    std::vector<PathPoint> full_path;

    // ---------------------------------------------------------------
    // Step 1: Calculate geometry
    // ---------------------------------------------------------------
    // Minimum turning radius at max steering angle
    double R = params.wheelbase / std::tan(params.max_steering_angle);

    // We need the vehicle to end up centered in the slot.
    // Transform the target center into the vehicle's local frame to compute
    // the lateral displacement needed.
    // In the slot frame, the vehicle drives along x-axis.
    // The lateral offset is the difference in y between the vehicle and the
    // slot center, measured perpendicular to the driving direction.

    // For parallel parking, the slot is alongside the vehicle.
    // The lateral displacement is how far the vehicle needs to move sideways
    // into the slot. We use the slot depth as the target lateral offset
    // (vehicle needs to move to the right by approximately slot depth).

    double lateral_disp = target.depth;

    // Ensure lateral displacement is reasonable and positive
    lateral_disp = std::abs(lateral_disp);
    if (lateral_disp < 0.1) {
        lateral_disp = 0.1;  // minimum lateral movement
    }

    // Ensure we don't need more lateral movement than 2*R (geometry limit)
    double max_lateral = 2.0 * R;
    if (lateral_disp > max_lateral) {
        // Reduce the lateral displacement to what is geometrically possible
        lateral_disp = max_lateral * 0.95;
    }

    // ---------------------------------------------------------------
    // Step 2: Calculate the arc angle for the two-arc maneuver
    // ---------------------------------------------------------------
    // From the geometry of two equal-radius arcs with opposite curvature:
    //   lateral displacement d = 2 * R * (1 - cos(alpha))
    // Solving for alpha: alpha = acos(1 - d / (2*R))
    double arg = 1.0 - lateral_disp / (2.0 * R);
    // Clamp to valid acos range [-1, 1]
    arg = std::clamp(arg, -1.0, 1.0);
    double alpha = std::acos(arg);

    // Limit alpha to a reasonable range (at most ~70 degrees)
    double max_alpha = 70.0 * kDeg2Rad;
    if (alpha > max_alpha) {
        alpha = max_alpha;
    }
    // Also ensure it is not too small
    if (alpha < 10.0 * kDeg2Rad) {
        alpha = 10.0 * kDeg2Rad;
    }

    // ---------------------------------------------------------------
    // Step 3: Phase 1 - Reverse right arc
    // ---------------------------------------------------------------
    // Steer right (negative angle), drive backward
    // The vehicle turns to the right while reversing into the slot
    double reverse_speed = -1.0;  // m/s (reverse)
    auto arc1 = generateArc(start, R, -alpha, -1.0, reverse_speed, params);
    full_path.insert(full_path.end(), arc1.begin(), arc1.end());

    // ---------------------------------------------------------------
    // Step 4: Phase 2 - Reverse left arc
    // ---------------------------------------------------------------
    // Start from the end of phase 1
    Pose2D phase2_start = arc1.empty() ? start : arc1.back().pose;
    // Steer left (positive angle), drive backward
    auto arc2 = generateArc(phase2_start, R, alpha, -1.0, reverse_speed, params);
    full_path.insert(full_path.end(), arc2.begin(), arc2.end());

    // ---------------------------------------------------------------
    // Step 5: Phase 3 - Straight reverse adjustment
    // ---------------------------------------------------------------
    // Calculate remaining distance to align with target position
    Pose2D phase3_start = arc2.empty() ? phase2_start : arc2.back().pose;

    // The vehicle has moved laterally into the slot via the two arcs.
    // Now it needs to move straight backward to align longitudinally
    // with the target center.

    // Calculate the longitudinal distance between the vehicle and the target
    double dx = target.center.x - phase3_start.position.x;
    double dy = target.center.y - phase3_start.position.y;

    // Project the displacement onto the vehicle's current heading
    double straight_dist = dx * std::cos(phase3_start.yaw) +
                           dy * std::sin(phase3_start.yaw);

    // If we need to go backward, straight_dist should be negative
    // If we need to go forward, it should be positive
    if (straight_dist > 0.0) {
        // Vehicle needs to go forward slightly - rare but handle it
        auto straight = generateStraight(phase3_start, straight_dist,
                                          0.5, params);
        full_path.insert(full_path.end(), straight.begin(), straight.end());
    } else if (straight_dist < -0.05) {
        // Vehicle needs to go backward to reach target
        auto straight = generateStraight(phase3_start, straight_dist,
                                          reverse_speed * 0.5, params);
        full_path.insert(full_path.end(), straight.begin(), straight.end());
    }

    return full_path;
}

}  // namespace auto_parking
