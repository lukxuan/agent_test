#pragma once
#include "types.h"
#include <memory>

namespace auto_parking {

// Abstract base class for path planners
class PathPlanner {
public:
    virtual ~PathPlanner() = default;

    // Generate parking path from start pose to target slot
    virtual std::vector<PathPoint> plan(
        Pose2D start,
        const ParkingSlot& target,
        VehicleParams params) = 0;

    // Validate path for collisions with slot boundaries
    virtual bool validatePath(
        const std::vector<PathPoint>& path,
        VehicleParams params,
        const ParkingSlot& target) const;

    // Get planner name
    virtual std::string name() const = 0;

protected:
    // Generate arc path points
    std::vector<PathPoint> generateArc(
        Pose2D start, double radius, double angle,
        double direction, double speed, VehicleParams params) const;

    // Generate straight path points
    std::vector<PathPoint> generateStraight(
        Pose2D start, double distance,
        double speed, VehicleParams params) const;

    // Transform path points to absolute coordinates
    void transformToAbsolute(std::vector<PathPoint>& path, Pose2D origin) const;

    // Check if a single pose collides with slot boundaries
    bool checkPoseCollision(Pose2D pose, VehicleParams params, const ParkingSlot& target) const;

    // Find closest path point index
    size_t findClosestPoint(const std::vector<PathPoint>& path, Pose2D pose) const;
};

// Parallel parking planner: two-arc + straight line
class ParallelPlanner : public PathPlanner {
public:
    std::vector<PathPoint> plan(
        Pose2D start,
        const ParkingSlot& target,
        VehicleParams params) override;

    std::string name() const override { return "ParallelPlanner"; }
};

// Perpendicular parking planner: forward adjust + reverse arc
class PerpendicularPlanner : public PathPlanner {
public:
    std::vector<PathPoint> plan(
        Pose2D start,
        const ParkingSlot& target,
        VehicleParams params) override;

    std::string name() const override { return "PerpendicularPlanner"; }
};

// Diagonal parking planner: angle-adaptive arc
class DiagonalPlanner : public PathPlanner {
public:
    std::vector<PathPoint> plan(
        Pose2D start,
        const ParkingSlot& target,
        VehicleParams params) override;

    std::string name() const override { return "DiagonalPlanner"; }
};

// Factory function to create appropriate planner
std::unique_ptr<PathPlanner> createPlanner(ParkingSlot::Type type);

}  // namespace auto_parking
