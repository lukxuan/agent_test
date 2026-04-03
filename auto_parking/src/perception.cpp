#include "auto_parking/perception.h"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace auto_parking {

std::optional<ParkingSlot> Perception::detectParkingSlot(
    const SensorData& data,
    VehicleParams params,
    ParkingSlot::Type expected_type) {

    if (data.ultrasonic.empty()) {
        return std::nullopt;
    }

    // Find a gap in the ultrasonic readings
    auto gap = findGap(data.ultrasonic);
    if (!gap.has_value()) {
        return std::nullopt;
    }

    // Build the parking slot from the detected gap
    ParkingSlot slot = buildSlotFromGap(gap->first, gap->second,
                                         data.ultrasonic, params, expected_type);
    return slot;
}

void Perception::setGapThreshold(double threshold) {
    gap_threshold_ = std::max(0.0, threshold);
}

void Perception::setObstacleThreshold(double threshold) {
    obstacle_threshold_ = std::max(0.0, threshold);
}

std::optional<std::pair<int, int>> Perception::findGap(
    const std::vector<double>& data) const {

    int start = -1;
    int end = -1;

    // Find the longest consecutive sequence of readings above gap_threshold_
    // (high distance = no obstacle = potential parking slot gap)
    int current_start = -1;
    int current_end = -1;
    int best_length = 0;

    for (int i = 0; i < static_cast<int>(data.size()); ++i) {
        if (data[i] > gap_threshold_) {
            // This reading is above the gap threshold (no obstacle detected)
            if (current_start < 0) {
                current_start = i;
            }
            current_end = i;
        } else {
            // Obstacle detected, end the current gap
            if (current_start >= 0) {
                int length = current_end - current_start + 1;
                if (length > best_length) {
                    best_length = length;
                    start = current_start;
                    end = current_end;
                }
                current_start = -1;
                current_end = -1;
            }
        }
    }

    // Check the last gap that may extend to the end of the array
    if (current_start >= 0) {
        int length = current_end - current_start + 1;
        if (length > best_length) {
            best_length = length;
            start = current_start;
            end = current_end;
        }
    }

    // We need at least a minimum number of consecutive readings to constitute a valid gap
    // (e.g., at least 5 readings * 0.1m spacing = 0.5m minimum gap)
    const int min_readings = 5;
    if (start >= 0 && (end - start + 1) >= min_readings) {
        return std::make_pair(start, end);
    }

    return std::nullopt;
}

ParkingSlot Perception::buildSlotFromGap(int start_idx, int end_idx,
                                          const std::vector<double>& data,
                                          VehicleParams params,
                                          ParkingSlot::Type type) const {

    ParkingSlot slot;
    slot.type = type;

    // Assume 0.1m spacing between consecutive ultrasonic sensor readings
    constexpr double sensor_spacing = 0.1;

    // Calculate the width of the slot from the number of gap readings
    int gap_count = end_idx - start_idx + 1;
    slot.width = static_cast<double>(gap_count) * sensor_spacing;

    // Calculate the depth of the slot from the average distance reading
    // within the gap (how far away the back wall is)
    double sum_distance = 0.0;
    int count = 0;
    for (int i = start_idx; i <= end_idx; ++i) {
        if (data[i] < 10.0) {  // ignore outlier readings (out of range)
            sum_distance += data[i];
            ++count;
        }
    }
    double avg_distance = (count > 0) ? (sum_distance / count) : 0.0;
    slot.depth = avg_distance;

    // Set the slot angle based on type
    switch (type) {
        case ParkingSlot::Type::PARALLEL:
            slot.angle = 0.0;  // parallel to driving direction
            break;
        case ParkingSlot::Type::PERPENDICULAR:
            slot.angle = kPi / 2.0;  // perpendicular to driving direction
            break;
        case ParkingSlot::Type::DIAGONAR:
            slot.angle = kPi / 4.0;  // 45 degrees (diagonal)
            break;
    }

    // Calculate the slot center position relative to the vehicle
    // The sensors are along the right side of the vehicle.
    // The gap starts at distance: start_idx * sensor_spacing from the front of sensor range
    // and the depth extends to the right of the vehicle.

    // Position along the vehicle's longitudinal axis (x)
    // Center of the gap in sensor coordinates
    double gap_center_along = static_cast<double>(start_idx + gap_count / 2) * sensor_spacing;

    // The center of the slot in world coordinates:
    // x: along vehicle direction, offset from vehicle position
    // y: lateral distance (depth of the slot from the vehicle side)
    // We assume sensors are along the right side at y = -params.width / 2
    slot.center.x = gap_center_along;
    slot.center.y = params.width / 2.0 + slot.depth / 2.0;

    // Calculate the 4 corners of the parking slot (counter-clockwise)
    // For a parallel slot, the slot is alongside the vehicle.
    // The slot extends along x and into y (to the right side).
    double half_width = slot.width / 2.0;
    double half_depth = slot.depth / 2.0;

    double cx = slot.center.x;
    double cy = slot.center.y;

    if (type == ParkingSlot::Type::PARALLEL) {
        // Slot is parallel to driving direction
        // Corners: bottom-left, top-left, top-right, bottom-right (CCW)
        slot.corners[0] = {cx - half_width, cy - half_depth};  // bottom-left
        slot.corners[1] = {cx + half_width, cy - half_depth};  // bottom-right (near vehicle)
        slot.corners[2] = {cx + half_width, cy + half_depth};  // top-right (far from vehicle)
        slot.corners[3] = {cx - half_width, cy + half_depth};  // top-left
    } else if (type == ParkingSlot::Type::PERPENDICULAR) {
        // Slot is perpendicular to driving direction
        // The slot opening faces the vehicle (along x-axis)
        // Corners: near-left, far-left, far-right, near-right (CCW)
        slot.corners[0] = {cx - half_width, cy - half_depth};
        slot.corners[1] = {cx - half_width, cy + half_depth};
        slot.corners[2] = {cx + half_width, cy + half_depth};
        slot.corners[3] = {cx + half_width, cy - half_depth};
    } else {
        // Diagonal slot - rotated by slot.angle
        // Calculate corners as if perpendicular, then rotate by slot.angle
        Point2D raw_corners[4] = {
            {cx - half_width, cy - half_depth},
            {cx - half_width, cy + half_depth},
            {cx + half_width, cy + half_depth},
            {cx + half_width, cy - half_depth}
        };
        // Rotate each corner around the center by the slot angle
        for (int i = 0; i < 4; ++i) {
            double dx = raw_corners[i].x - cx;
            double dy = raw_corners[i].y - cy;
            double cos_a = std::cos(slot.angle);
            double sin_a = std::sin(slot.angle);
            slot.corners[i].x = cx + dx * cos_a - dy * sin_a;
            slot.corners[i].y = cy + dx * sin_a + dy * cos_a;
        }
    }

    return slot;
}

}  // namespace auto_parking
