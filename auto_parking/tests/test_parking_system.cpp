#include <gtest/gtest.h>
#include "auto_parking/parking_system.h"
#include "auto_parking/path_planner.h"
#include "auto_parking/vehicle_model.h"
#include "auto_parking/types.h"
#include <cmath>

using namespace auto_parking;

// Helper: build a ParkingSlot from center, width, depth, angle, type
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

// ---------------------------------------------------------------------------
// 1. Construction
// ---------------------------------------------------------------------------
TEST(ParkingSystemTest, Construction) {
    VehicleParams params;
    ParkingSystem system(params);
    EXPECT_EQ(system.getState(), ParkingState::IDLE);
}

// ---------------------------------------------------------------------------
// 2. Start and get path
// ---------------------------------------------------------------------------
TEST(ParkingSystemTest, StartAndGetPath) {
    VehicleParams params;
    ParkingSystem system(params);

    auto slot = makeSlot({10.0, 3.0}, 5.5, 2.5, 0.0, ParkingSlot::Type::PARALLEL);
    system.startParking(slot);

    // State should have changed from IDLE
    EXPECT_NE(system.getState(), ParkingState::IDLE);

    // Drive a few update cycles to trigger planning
    Pose2D pose{{0.0, 0.0}, 0.0};
    for (int i = 0; i < 5; ++i) {
        system.update(pose, 0.0);
    }

    // Path should have been generated
    EXPECT_FALSE(system.getPath().empty());
}

// ---------------------------------------------------------------------------
// 3. Full parking integration (parallel)
// ---------------------------------------------------------------------------
TEST(ParkingSystemTest, FullParkingIntegration) {
    VehicleParams params;
    ParkingSystem system(params);

    auto slot = makeSlot({10.0, 3.0}, 5.5, 2.5, 0.0, ParkingSlot::Type::PARALLEL);
    system.startParking(slot);

    Pose2D pose{{0.0, 0.0}, 0.0};
    double speed = 0.0;
    double dt = 0.02;
    const int max_iterations = 10000;

    for (int i = 0; i < max_iterations; ++i) {
        auto cmd = system.update(pose, speed);
        auto state = system.getState();

        if (state == ParkingState::COMPLETED || state == ParkingState::FAILED) {
            break;
        }

        // Simulate one step
        pose = system.simulateStep(pose, cmd, dt);
        speed = cmd.speed;
    }

    auto final_state = system.getState();
    EXPECT_TRUE(final_state == ParkingState::COMPLETED ||
                final_state == ParkingState::FAILED);
    EXPECT_FALSE(system.getPath().empty());
}

// ---------------------------------------------------------------------------
// 4. Get vehicle model
// ---------------------------------------------------------------------------
TEST(ParkingSystemTest, GetVehicleModel) {
    VehicleParams params;
    params.wheelbase = 3.0;
    ParkingSystem system(params);

    const auto& model = system.getVehicleModel();
    EXPECT_DOUBLE_EQ(model.getParams().wheelbase, 3.0);
}

// ---------------------------------------------------------------------------
// 5. Simulate step
// ---------------------------------------------------------------------------
TEST(ParkingSystemTest, SimulateStep) {
    VehicleParams params;
    ParkingSystem system(params);

    Pose2D start{{5.0, 0.0}, 0.0};
    ControlCommand cmd{0.0, 1.0, 0.0};  // forward straight
    auto result = system.simulateStep(start, cmd, 1.0);

    EXPECT_NE(result.position.x, start.position.x);
    EXPECT_NEAR(result.position.x, 6.0, 0.05);
    EXPECT_NEAR(result.position.y, 0.0, 0.05);
}

// ---------------------------------------------------------------------------
// 6. Reset
// ---------------------------------------------------------------------------
TEST(ParkingSystemTest, Reset) {
    VehicleParams params;
    ParkingSystem system(params);

    auto slot = makeSlot({10.0, 3.0}, 5.5, 2.5, 0.0, ParkingSlot::Type::PARALLEL);
    system.startParking(slot);
    EXPECT_NE(system.getState(), ParkingState::IDLE);

    system.reset();
    EXPECT_EQ(system.getState(), ParkingState::IDLE);
}

// ---------------------------------------------------------------------------
// 7. Multiple parking scenarios sequentially
// ---------------------------------------------------------------------------
TEST(ParkingSystemTest, MultipleParkingScenarios) {
    VehicleParams params;
    ParkingSystem system(params);

    // First: parallel parking
    auto slot1 = makeSlot({10.0, 3.0}, 5.5, 2.5, 0.0, ParkingSlot::Type::PARALLEL);
    system.startParking(slot1);

    Pose2D pose{{0.0, 0.0}, 0.0};
    double speed = 0.0;

    for (int i = 0; i < 5000; ++i) {
        auto cmd = system.update(pose, speed);
        auto state = system.getState();
        if (state == ParkingState::COMPLETED || state == ParkingState::FAILED) break;
        pose = system.simulateStep(pose, cmd, 0.02);
        speed = cmd.speed;
    }

    // Reset for next scenario
    system.reset();
    EXPECT_EQ(system.getState(), ParkingState::IDLE);

    // Second: perpendicular parking
    auto slot2 = makeSlot({10.0, 0.0}, 2.5, 5.5, kPi / 2.0,
                          ParkingSlot::Type::PERPENDICULAR);
    system.startParking(slot2);

    pose = {{0.0, 0.0}, 0.0};
    speed = 0.0;

    for (int i = 0; i < 5000; ++i) {
        auto cmd = system.update(pose, speed);
        auto state = system.getState();
        if (state == ParkingState::COMPLETED || state == ParkingState::FAILED) break;
        pose = system.simulateStep(pose, cmd, 0.02);
        speed = cmd.speed;
    }

    EXPECT_FALSE(system.getPath().empty());
}
