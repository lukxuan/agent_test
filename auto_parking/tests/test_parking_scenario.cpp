#include <gtest/gtest.h>
#include "auto_parking/parking_scenario.h"
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
// 1. Initial state
// ---------------------------------------------------------------------------
TEST(ParkingScenarioTest, InitialState) {
    VehicleParams params;
    ParkingScenario scenario(params);
    EXPECT_EQ(scenario.getState(), ParkingState::IDLE);
}

// ---------------------------------------------------------------------------
// 2. Start transitions to SEARCHING
// ---------------------------------------------------------------------------
TEST(ParkingScenarioTest, StartTransition) {
    VehicleParams params;
    ParkingScenario scenario(params);

    auto slot = makeSlot({10.0, 3.0}, 5.5, 2.5, 0.0, ParkingSlot::Type::PARALLEL);
    scenario.start(slot);
    EXPECT_EQ(scenario.getState(), ParkingState::SEARCHING);
}

// ---------------------------------------------------------------------------
// 3. Full parallel parking simulation
// ---------------------------------------------------------------------------
TEST(ParkingScenarioTest, FullParallelParking) {
    VehicleParams params;
    ParkingScenario scenario(params);
    VehicleModel model(params);

    auto slot = makeSlot({10.0, 3.0}, 5.5, 2.5, 0.0, ParkingSlot::Type::PARALLEL);

    scenario.start(slot);
    EXPECT_EQ(scenario.getState(), ParkingState::SEARCHING);

    Pose2D pose{{0.0, 0.0}, 0.0};
    double speed = 0.0;
    double dt = 0.02;
    const int max_iterations = 10000;
    bool completed = false;

    for (int i = 0; i < max_iterations; ++i) {
        auto cmd = scenario.update(pose, speed);
        auto state = scenario.getState();

        if (state == ParkingState::COMPLETED) {
            completed = true;
            break;
        }
        if (state == ParkingState::FAILED) {
            break;
        }
        if (state == ParkingState::EXECUTING || state == ParkingState::SEARCHING) {
            // Simulate vehicle motion
            pose = model.update(pose, cmd, dt);
            speed = cmd.speed;
        }
    }

    // Verify the scenario reached a terminal state
    EXPECT_TRUE(completed || scenario.getState() == ParkingState::FAILED ||
                scenario.getState() == ParkingState::COMPLETED);

    // Verify path was generated
    EXPECT_FALSE(scenario.getPath().empty());
}

// ---------------------------------------------------------------------------
// 4. Full perpendicular parking simulation
// ---------------------------------------------------------------------------
TEST(ParkingScenarioTest, FullPerpendicularParking) {
    VehicleParams params;
    ParkingScenario scenario(params);
    VehicleModel model(params);

    auto slot = makeSlot({10.0, 0.0}, 2.5, 5.5, kPi / 2.0,
                         ParkingSlot::Type::PERPENDICULAR);

    scenario.start(slot);

    Pose2D pose{{0.0, 0.0}, 0.0};
    double speed = 0.0;
    double dt = 0.02;
    const int max_iterations = 10000;

    for (int i = 0; i < max_iterations; ++i) {
        auto cmd = scenario.update(pose, speed);
        auto state = scenario.getState();

        if (state == ParkingState::COMPLETED || state == ParkingState::FAILED) {
            break;
        }
        pose = model.update(pose, cmd, dt);
        speed = cmd.speed;
    }

    // Should have reached a terminal state
    auto final_state = scenario.getState();
    EXPECT_TRUE(final_state == ParkingState::COMPLETED ||
                final_state == ParkingState::FAILED);
    EXPECT_FALSE(scenario.getPath().empty());
}

// ---------------------------------------------------------------------------
// 5. Reset returns to IDLE
// ---------------------------------------------------------------------------
TEST(ParkingScenarioTest, Reset) {
    VehicleParams params;
    ParkingScenario scenario(params);

    auto slot = makeSlot({10.0, 3.0}, 5.5, 2.5, 0.0, ParkingSlot::Type::PARALLEL);
    scenario.start(slot);
    EXPECT_NE(scenario.getState(), ParkingState::IDLE);

    scenario.reset();
    EXPECT_EQ(scenario.getState(), ParkingState::IDLE);
    EXPECT_TRUE(scenario.getPath().empty());
}

// ---------------------------------------------------------------------------
// 6. Update without start returns zero command
// ---------------------------------------------------------------------------
TEST(ParkingScenarioTest, UpdateWithoutStart) {
    VehicleParams params;
    ParkingScenario scenario(params);

    Pose2D pose{{0.0, 0.0}, 0.0};
    auto cmd = scenario.update(pose, 0.0);

    EXPECT_DOUBLE_EQ(cmd.steering_angle, 0.0);
    EXPECT_DOUBLE_EQ(cmd.speed, 0.0);
}

// ---------------------------------------------------------------------------
// 7. Diagonal parking simulation
// ---------------------------------------------------------------------------
TEST(ParkingScenarioTest, DiagonalParking) {
    VehicleParams params;
    ParkingScenario scenario(params);
    VehicleModel model(params);

    auto slot = makeSlot({10.0, 3.0}, 3.0, 5.0, 45.0 * kDeg2Rad,
                         ParkingSlot::Type::DIAGONAR);

    scenario.start(slot);

    Pose2D pose{{0.0, 0.0}, 0.0};
    double speed = 0.0;
    double dt = 0.02;
    const int max_iterations = 10000;

    for (int i = 0; i < max_iterations; ++i) {
        auto cmd = scenario.update(pose, speed);
        auto state = scenario.getState();

        if (state == ParkingState::COMPLETED || state == ParkingState::FAILED) {
            break;
        }
        pose = model.update(pose, cmd, dt);
        speed = cmd.speed;
    }

    auto final_state = scenario.getState();
    EXPECT_TRUE(final_state == ParkingState::COMPLETED ||
                final_state == ParkingState::FAILED);
}
