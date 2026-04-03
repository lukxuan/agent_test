#include "auto_parking/parking_scenario.h"
#include <cmath>

namespace auto_parking {

ParkingScenario::ParkingScenario(VehicleParams params)
    : params_(std::move(params))
    , controller_() {}

void ParkingScenario::start(ParkingSlot slot) {
    target_slot_ = std::move(slot);
    state_ = ParkingState::SEARCHING;
    planned_path_.clear();
    planner_.reset();
    elapsed_time_ = 0.0;
    path_progress_ = 0;
    controller_.reset();
}

ControlCommand ParkingScenario::update(Pose2D current_pose, double current_speed) {
    ControlCommand cmd;  // zero-initialized

    switch (state_) {
        case ParkingState::SEARCHING: {
            // First update provides the real start pose - trigger planning
            // Create planner for the target slot type
            planner_ = createPlanner(target_slot_.type);
            planned_path_ = planner_->plan(current_pose, target_slot_, params_);

            if (!planned_path_.empty()) {
                state_ = ParkingState::EXECUTING;
                controller_.reset();
                elapsed_time_ = 0.0;
            } else {
                state_ = ParkingState::FAILED;
            }
            break;
        }

        case ParkingState::PLANNING: {
            // Should not normally be reached from update(), treat as failure
            state_ = ParkingState::FAILED;
            break;
        }

        case ParkingState::EXECUTING: {
            // Compute control command via motion controller
            cmd = controller_.compute(current_pose, planned_path_, current_speed);

            // Track elapsed time
            elapsed_time_ += dt_;

            // Check completion
            if (controller_.isFinished()) {
                state_ = ParkingState::COMPLETED;
            }

            // Check timeout
            if (elapsed_time_ > timeout_) {
                state_ = ParkingState::FAILED;
            }
            break;
        }

        case ParkingState::COMPLETED:
        case ParkingState::FAILED:
        case ParkingState::IDLE:
        default:
            // Return zero command for terminal states
            break;
    }

    return cmd;
}

ParkingState ParkingScenario::getState() const {
    return state_;
}

const std::vector<PathPoint>& ParkingScenario::getPath() const {
    return planned_path_;
}

void ParkingScenario::reset() {
    state_ = ParkingState::IDLE;
    target_slot_ = ParkingSlot{};
    planned_path_.clear();
    planner_.reset();
    controller_.reset();
    path_progress_ = 0;
    elapsed_time_ = 0.0;
}

}  // namespace auto_parking
