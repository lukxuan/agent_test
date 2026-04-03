#include "auto_parking/parking_system.h"

namespace auto_parking {

ParkingSystem::ParkingSystem(VehicleParams params)
    : params_(std::move(params))
    , vehicle_model_(params_)
    , perception_()
    , scenario_(params_) {}

void ParkingSystem::startParking(ParkingSlot slot) {
    scenario_.start(std::move(slot));
    state_ = ParkingState::SEARCHING;
}

ControlCommand ParkingSystem::update(Pose2D current_pose, double current_speed) {
    ControlCommand cmd = scenario_.update(current_pose, current_speed);
    state_ = scenario_.getState();
    return cmd;
}

ParkingState ParkingSystem::getState() const {
    return state_;
}

const std::vector<PathPoint>& ParkingSystem::getPath() const {
    return scenario_.getPath();
}

const VehicleModel& ParkingSystem::getVehicleModel() const {
    return vehicle_model_;
}

Pose2D ParkingSystem::simulateStep(Pose2D current, ControlCommand cmd, double dt) {
    return vehicle_model_.update(current, cmd, dt);
}

void ParkingSystem::reset() {
    scenario_.reset();
    state_ = ParkingState::IDLE;
}

}  // namespace auto_parking
