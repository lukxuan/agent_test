#include "auto_parking/parking_system.h"
#include "auto_parking/types.h"
#include "auto_parking/path_planner.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace auto_parking;

// Helper: build a ParkingSlot
ParkingSlot makeSlot(Point2D center, double width, double depth, double angle,
                     ParkingSlot::Type type) {
    ParkingSlot slot;
    slot.center = center;
    slot.width = width;
    slot.depth = depth;
    slot.angle = angle;
    slot.type = type;

    double half_w = width / 2.0;
    double half_d = depth / 2.0;
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);

    std::vector<Point2D> local = {
        {-half_d, -half_w}, {half_d, -half_w},
        {half_d,  half_w},  {-half_d,  half_w}
    };

    for (int i = 0; i < 4; ++i) {
        slot.corners[i].x = center.x + local[i].x * cos_a - local[i].y * sin_a;
        slot.corners[i].y = center.y + local[i].x * sin_a + local[i].y * cos_a;
    }
    return slot;
}

void printSeparator() {
    std::cout << std::string(60, '=') << std::endl;
}

void runParkingDemo(ParkingSystem& system, ParkingSlot& slot,
                    Pose2D start_pose, const std::string& scenario_name) {
    printSeparator();
    std::cout << "Scenario: " << scenario_name << std::endl;
    std::cout << "  Slot type: " << slot.typeToString() << std::endl;
    std::cout << "  Slot center: (" << slot.center.x << ", " << slot.center.y << ")" << std::endl;
    std::cout << "  Slot size: " << slot.width << " x " << slot.depth << " m" << std::endl;
    std::cout << "  Slot angle: " << (slot.angle * kRad2Deg) << " deg" << std::endl;
    std::cout << "  Start pose: (" << start_pose.position.x << ", "
              << start_pose.position.y << ", " << (start_pose.yaw * kRad2Deg) << " deg)"
              << std::endl;
    printSeparator();

    system.reset();
    system.startParking(slot);

    Pose2D pose = start_pose;
    double speed = 0.0;
    double dt = 0.02;  // 50 Hz control loop
    int step = 0;
    const int max_steps = 10000;
    int log_interval = 200;  // Log every 4 seconds

    std::cout << std::fixed << std::setprecision(3);

    while (step < max_steps) {
        auto cmd = system.update(pose, speed);
        auto state = system.getState();

        if (step % log_interval == 0) {
            std::cout << "  [Step " << std::setw(5) << step << "] "
                      << "State: " << std::setw(10) << parkingStateToString(state)
                      << " | Pose: (" << std::setw(7) << pose.position.x << ", "
                      << std::setw(7) << pose.position.y << ", "
                      << std::setw(7) << (pose.yaw * kRad2Deg) << " deg)"
                      << " | Cmd: steer=" << std::setw(6) << (cmd.steering_angle * kRad2Deg)
                      << " deg, speed=" << std::setw(5) << cmd.speed << " m/s"
                      << std::endl;
        }

        if (state == ParkingState::COMPLETED || state == ParkingState::FAILED) {
            std::cout << "  [Step " << std::setw(5) << step << "] "
                      << "State: " << std::setw(10) << parkingStateToString(state)
                      << " | Final Pose: (" << pose.position.x << ", "
                      << pose.position.y << ", " << (pose.yaw * kRad2Deg) << " deg)"
                      << std::endl;
            break;
        }

        pose = system.simulateStep(pose, cmd, dt);
        speed = cmd.speed;
        step++;
    }

    // Report results
    const auto& path = system.getPath();
    double dist_to_center = pose.position.distanceTo(slot.center);
    double heading_error = std::abs(normalizeAngle(pose.yaw - slot.angle));

    std::cout << "\n  --- Results ---" << std::endl;
    std::cout << "  Total steps: " << step << " (sim time: " << (step * dt) << " s)" << std::endl;
    std::cout << "  Path points: " << path.size() << std::endl;
    std::cout << "  Distance to slot center: " << dist_to_center << " m" << std::endl;
    std::cout << "  Heading error: " << (heading_error * kRad2Deg) << " deg" << std::endl;
    std::cout << "  Final state: " << parkingStateToString(system.getState()) << std::endl;
    std::cout << std::endl;
}

int main() {
    std::cout << "\n";
    printSeparator();
    std::cout << "   Auto Parking System - Demo" << std::endl;
    printSeparator();
    std::cout << std::endl;

    VehicleParams params;
    ParkingSystem system(params);

    // --- Scenario 1: Parallel Parking ---
    auto parallel_slot = makeSlot({10.0, 3.0}, 5.5, 2.5, 0.0,
                                  ParkingSlot::Type::PARALLEL);
    Pose2D parallel_start{{0.0, 0.5}, 0.0};  // Vehicle slightly offset from slot
    runParkingDemo(system, parallel_slot, parallel_start, "Parallel Parking");

    // --- Scenario 2: Perpendicular Parking ---
    auto perpendicular_slot = makeSlot({10.0, 0.0}, 2.5, 5.5, kPi / 2.0,
                                       ParkingSlot::Type::PERPENDICULAR);
    Pose2D perpendicular_start{{0.0, -2.0}, 0.0};
    runParkingDemo(system, perpendicular_slot, perpendicular_start, "Perpendicular Parking");

    // --- Scenario 3: Diagonal Parking (45 degrees) ---
    auto diagonal_slot = makeSlot({10.0, 3.0}, 3.0, 5.0, 45.0 * kDeg2Rad,
                                  ParkingSlot::Type::DIAGONAR);
    Pose2D diagonal_start{{0.0, 0.5}, 0.0};
    runParkingDemo(system, diagonal_slot, diagonal_start, "Diagonal Parking (45 deg)");

    printSeparator();
    std::cout << "   All demos completed." << std::endl;
    printSeparator();

    return 0;
}
