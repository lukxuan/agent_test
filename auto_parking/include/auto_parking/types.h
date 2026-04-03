#pragma once
#include <vector>
#include <cmath>
#include <optional>
#include <string>

namespace auto_parking {

// Constants
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

struct Point2D {
    double x = 0.0;
    double y = 0.0;
    double distanceTo(const Point2D& other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
};

struct Pose2D {
    Point2D position;
    double yaw = 0.0;  // radians, counter-clockwise positive
};

struct VehicleParams {
    double wheelbase = 2.8;           // wheelbase (m)
    double track_width = 1.6;         // track width (m)
    double length = 4.6;              // vehicle length (m)
    double width = 1.85;              // vehicle width (m)
    double rear_overhang = 0.9;       // rear overhang (m)
    double front_overhang = 0.9;      // front overhang (m)
    double min_turn_radius = 5.0;     // minimum turning radius (m)
    double max_steering_angle = 35.0 * kDeg2Rad;  // max steering angle (rad)
    double max_speed = 5.0;           // max forward speed (m/s)
    double max_reverse_speed = 2.0;   // max reverse speed (m/s)
};

struct SensorData {
    std::vector<double> ultrasonic;  // distance values in meters
    double timestamp = 0.0;
};

struct ParkingSlot {
    enum class Type { PARALLEL, PERPENDICULAR, DIAGONAR };
    Type type = Type::PARALLEL;
    Point2D corners[4];    // 4 corners counter-clockwise
    double width = 0.0;    // slot width (m)
    double depth = 0.0;    // slot depth (m)
    double angle = 0.0;    // slot orientation angle (rad)
    Point2D center;        // center point

    std::string typeToString() const {
        switch (type) {
            case Type::PARALLEL: return "Parallel";
            case Type::PERPENDICULAR: return "Perpendicular";
            case Type::DIAGONAR: return "Diagonal";
            default: return "Unknown";
        }
    }
};

struct PathPoint {
    Pose2D pose;
    double curvature = 0.0;   // curvature (1/m), 0 = straight
    double speed = 0.0;       // target speed (m/s)
};

struct ControlCommand {
    double steering_angle = 0.0;  // target steering angle (rad), positive = left
    double speed = 0.0;          // target speed (m/s), positive = forward, negative = reverse
    double curvature = 0.0;      // current path curvature (1/m)
};

enum class ParkingState {
    IDLE,
    SEARCHING,
    PLANNING,
    EXECUTING,
    COMPLETED,
    FAILED
};

std::string parkingStateToString(ParkingState state);

// Geometry utility functions
double normalizeAngle(double angle);  // normalize to [-pi, pi]
Point2D rotatePoint(const Point2D& p, double angle);  // rotate point around origin
Point2D transformPoint(const Point2D& p, const Pose2D& frame);  // transform point to frame

// Collision detection using Separating Axis Theorem (SAT)
bool checkCollision(const Pose2D& vehicle_pose, const VehicleParams& vehicle,
                    const std::vector<Point2D>& obstacles);

}  // namespace auto_parking
