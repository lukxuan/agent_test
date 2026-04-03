#pragma once
#include "types.h"
#include "path_planner.h"
#include "motion_controller.h"
#include <memory>

namespace auto_parking {

class ParkingScenario {
public:
    ParkingScenario(VehicleParams params);

    // Start parking into the given slot
    void start(ParkingSlot slot);

    // Update with current vehicle state
    ControlCommand update(Pose2D current_pose, double current_speed);

    // Get current parking state
    ParkingState getState() const;

    // Get planned path
    const std::vector<PathPoint>& getPath() const;

    // Reset to idle
    void reset();

private:
    VehicleParams params_;
    ParkingState state_ = ParkingState::IDLE;
    ParkingSlot target_slot_;
    std::vector<PathPoint> planned_path_;
    std::unique_ptr<PathPlanner> planner_;
    MotionController controller_;
    size_t path_progress_ = 0;
    double timeout_ = 60.0;  // max parking time (seconds)
    double elapsed_time_ = 0.0;
    double dt_ = 0.01;       // control loop period (seconds)
};

}  // namespace auto_parking
