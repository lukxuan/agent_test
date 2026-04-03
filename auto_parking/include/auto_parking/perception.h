#pragma once
#include "types.h"

namespace auto_parking {

// Detect parking slot from ultrasonic sensor data
// The ultrasonic array represents distance readings along one side of the vehicle
// Gap pattern analysis identifies parking slot boundaries
class Perception {
public:
    // Detect parking slot from sensor data
    // Returns detected slot or std::nullopt if no slot found
    std::optional<ParkingSlot> detectParkingSlot(
        const SensorData& data,
        VehicleParams params,
        ParkingSlot::Type expected_type = ParkingSlot::Type::PARALLEL);

    // Set detection thresholds
    void setGapThreshold(double threshold);   // minimum gap to consider as slot entry
    void setObstacleThreshold(double threshold); // distance below this = obstacle detected

private:
    double gap_threshold_ = 1.5;        // minimum gap size (m)
    double obstacle_threshold_ = 0.5;   // obstacle detection distance (m)

    // Analyze sensor data to find gap pattern
    std::optional<std::pair<int, int>> findGap(const std::vector<double>& data) const;

    // Build ParkingSlot from gap indices
    ParkingSlot buildSlotFromGap(int start_idx, int end_idx,
                                  const std::vector<double>& data,
                                  VehicleParams params,
                                  ParkingSlot::Type type) const;
};

}  // namespace auto_parking
