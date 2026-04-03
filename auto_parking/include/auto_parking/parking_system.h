#pragma once
#include "types.h"
#include "perception.h"
#include "parking_scenario.h"

namespace auto_parking {

class ParkingSystem {
public:
    explicit ParkingSystem(VehicleParams params);

    // Initialize and start parking with a known parking slot
    void startParking(ParkingSlot slot);

    // Main update loop - call repeatedly with current vehicle state
    ControlCommand update(Pose2D current_pose, double current_speed);

    // Get current state
    ParkingState getState() const;

    // Get planned path
    const std::vector<PathPoint>& getPath() const;

    // Get vehicle model (for simulation)
    const VehicleModel& getVehicleModel() const;

    // Update vehicle pose using vehicle model (for simulation)
    Pose2D simulateStep(Pose2D current, ControlCommand cmd, double dt);

    // Reset system
    void reset();

private:
    VehicleParams params_;
    VehicleModel vehicle_model_;
    Perception perception_;
    ParkingScenario scenario_;
    ParkingState state_ = ParkingState::IDLE;
};

}  // namespace auto_parking
