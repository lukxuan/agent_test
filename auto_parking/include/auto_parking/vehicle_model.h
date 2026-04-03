#pragma once
#include "types.h"

namespace auto_parking {

class VehicleModel {
public:
    explicit VehicleModel(VehicleParams params);

    // Update vehicle pose using bicycle model (Euler integration)
    Pose2D update(Pose2D current, ControlCommand cmd, double dt) const;

    // Get vehicle parameters
    const VehicleParams& getParams() const;

    // Calculate turning radius for given steering angle
    double turningRadius(double steering_angle) const;

    // Clamp steering angle to vehicle limits
    double clampSteering(double steering_angle) const;

    // Clamp speed to vehicle limits
    double clampSpeed(double speed) const;

    // Get vehicle footprint corners in world frame
    std::vector<Point2D> getFootprint(Pose2D pose) const;

private:
    VehicleParams params_;
};

}  // namespace auto_parking
