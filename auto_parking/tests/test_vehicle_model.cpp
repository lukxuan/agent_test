#include <gtest/gtest.h>
#include "auto_parking/vehicle_model.h"
#include "auto_parking/types.h"
#include <cmath>
#include <limits>

using namespace auto_parking;

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------
class VehicleModelTest : public ::testing::Test {
protected:
    VehicleParams default_params;
    VehicleModel model{default_params};
};

// ---------------------------------------------------------------------------
// 1. Construction
// ---------------------------------------------------------------------------
TEST_F(VehicleModelTest, VehicleModelConstruction) {
    VehicleModel m(default_params);
    const auto& p = m.getParams();

    EXPECT_DOUBLE_EQ(p.wheelbase, 2.8);
    EXPECT_DOUBLE_EQ(p.track_width, 1.6);
    EXPECT_DOUBLE_EQ(p.length, 4.6);
    EXPECT_DOUBLE_EQ(p.width, 1.85);
    EXPECT_DOUBLE_EQ(p.rear_overhang, 0.9);
    EXPECT_DOUBLE_EQ(p.front_overhang, 0.9);
    EXPECT_DOUBLE_EQ(p.min_turn_radius, 5.0);
    EXPECT_DOUBLE_EQ(p.max_steering_angle, 35.0 * kDeg2Rad);
    EXPECT_DOUBLE_EQ(p.max_speed, 5.0);
    EXPECT_DOUBLE_EQ(p.max_reverse_speed, 2.0);
}

// ---------------------------------------------------------------------------
// 2. Update straight line
// ---------------------------------------------------------------------------
TEST_F(VehicleModelTest, UpdateStraightLine) {
    Pose2D start{{0.0, 0.0}, 0.0};
    ControlCommand cmd{0.0, 1.0, 0.0};
    double dt = 1.0;

    Pose2D result = model.update(start, cmd, dt);

    EXPECT_NEAR(result.position.x, 1.0, 0.05);
    EXPECT_NEAR(result.position.y, 0.0, 0.05);
    EXPECT_NEAR(result.yaw, 0.0, 0.05);
}

// ---------------------------------------------------------------------------
// 3. Update with steering (left turn)
// ---------------------------------------------------------------------------
TEST_F(VehicleModelTest, UpdateWithSteering) {
    Pose2D start{{0.0, 0.0}, 0.0};
    // Positive steering = left turn
    ControlCommand cmd{0.1, 1.0, 0.0};
    double dt = 1.0;

    Pose2D result = model.update(start, cmd, dt);

    // Vehicle should have turned left: yaw should increase
    double dyaw = (cmd.speed * std::tan(cmd.steering_angle) / default_params.wheelbase) * dt;
    EXPECT_NEAR(result.yaw, dyaw, 0.05);
    EXPECT_GT(result.position.x, 0.0);  // moved forward
    EXPECT_GT(result.position.y, 0.0);  // moved left (positive y)
}

// ---------------------------------------------------------------------------
// 4. Turning radius
// ---------------------------------------------------------------------------
TEST_F(VehicleModelTest, TurningRadius) {
    // At max steering angle, R = L / tan(delta)
    double max_steer = default_params.max_steering_angle;
    double expected_R = default_params.wheelbase / std::tan(max_steer);
    EXPECT_NEAR(model.turningRadius(max_steer), expected_R, 1e-3);

    // At a known moderate angle
    double R10 = model.turningRadius(10.0 * kDeg2Rad);
    double expected_R10 = default_params.wheelbase / std::tan(10.0 * kDeg2Rad);
    EXPECT_NEAR(R10, expected_R10, 1e-3);

    // At zero steering, R should be infinity
    EXPECT_EQ(model.turningRadius(0.0), std::numeric_limits<double>::infinity());

    // Negative steering angle should give negative radius
    double R_neg = model.turningRadius(-0.1);
    EXPECT_LT(R_neg, 0.0);
}

// ---------------------------------------------------------------------------
// 5. Clamp steering
// ---------------------------------------------------------------------------
TEST_F(VehicleModelTest, ClampSteering) {
    double max_steer = default_params.max_steering_angle;

    // Above max
    EXPECT_DOUBLE_EQ(model.clampSteering(max_steer + 0.1), max_steer);

    // Below -max
    EXPECT_DOUBLE_EQ(model.clampSteering(-max_steer - 0.1), -max_steer);

    // Within range
    EXPECT_DOUBLE_EQ(model.clampSteering(0.0), 0.0);
    EXPECT_DOUBLE_EQ(model.clampSteering(0.05), 0.05);
    EXPECT_DOUBLE_EQ(model.clampSteering(max_steer), max_steer);
    EXPECT_DOUBLE_EQ(model.clampSteering(-max_steer), -max_steer);
}

// ---------------------------------------------------------------------------
// 6. Clamp speed
// ---------------------------------------------------------------------------
TEST_F(VehicleModelTest, ClampSpeed) {
    // Positive above max
    EXPECT_DOUBLE_EQ(model.clampSpeed(default_params.max_speed + 1.0), default_params.max_speed);

    // Negative below -max_reverse
    EXPECT_DOUBLE_EQ(model.clampSpeed(-default_params.max_reverse_speed - 1.0),
                     -default_params.max_reverse_speed);

    // Zero
    EXPECT_DOUBLE_EQ(model.clampSpeed(0.0), 0.0);

    // Within range positive
    EXPECT_DOUBLE_EQ(model.clampSpeed(1.0), 1.0);
    EXPECT_DOUBLE_EQ(model.clampSpeed(default_params.max_speed), default_params.max_speed);

    // Within range negative
    EXPECT_DOUBLE_EQ(model.clampSpeed(-1.0), -1.0);
    EXPECT_DOUBLE_EQ(model.clampSpeed(-default_params.max_reverse_speed),
                     -default_params.max_reverse_speed);
}

// ---------------------------------------------------------------------------
// 7. Get footprint (heading = 0)
// ---------------------------------------------------------------------------
TEST_F(VehicleModelTest, GetFootprint) {
    Pose2D pose{{0.0, 0.0}, 0.0};
    auto corners = model.getFootprint(pose);

    ASSERT_EQ(corners.size(), 4u);

    double rear = -default_params.rear_overhang;        // -0.9
    double front = default_params.wheelbase + default_params.front_overhang;  // 3.7
    double half_w = default_params.width / 2.0;         // 0.925

    // Corner order: rear-left, front-left, front-right, rear-right
    EXPECT_NEAR(corners[0].x, rear, 1e-6);
    EXPECT_NEAR(corners[0].y, half_w, 1e-6);

    EXPECT_NEAR(corners[1].x, front, 1e-6);
    EXPECT_NEAR(corners[1].y, half_w, 1e-6);

    EXPECT_NEAR(corners[2].x, front, 1e-6);
    EXPECT_NEAR(corners[2].y, -half_w, 1e-6);

    EXPECT_NEAR(corners[3].x, rear, 1e-6);
    EXPECT_NEAR(corners[3].y, -half_w, 1e-6);

    // Verify rectangle dimensions: opposite sides should be equal
    double left_len = corners[0].distanceTo(corners[1]);
    double right_len = corners[2].distanceTo(corners[3]);
    EXPECT_NEAR(left_len, right_len, 1e-6);
    EXPECT_NEAR(left_len, front - rear, 1e-6);  // length of vehicle

    double top_len = corners[1].distanceTo(corners[2]);
    double bottom_len = corners[3].distanceTo(corners[0]);
    EXPECT_NEAR(top_len, bottom_len, 1e-6);
    EXPECT_NEAR(top_len, default_params.width, 1e-6);
}

// ---------------------------------------------------------------------------
// 8. Get footprint with rotation (heading = 90 degrees)
// ---------------------------------------------------------------------------
TEST_F(VehicleModelTest, GetFootprintWithRotation) {
    double yaw90 = 90.0 * kDeg2Rad;
    Pose2D pose{{0.0, 0.0}, yaw90};
    auto corners = model.getFootprint(pose);

    ASSERT_EQ(corners.size(), 4u);

    double rear = -default_params.rear_overhang;
    double front = default_params.wheelbase + default_params.front_overhang;
    double half_w = default_params.width / 2.0;

    double cos90 = std::cos(yaw90);
    double sin90 = std::sin(yaw90);

    // After 90-degree rotation:
    //   local (x, y) -> world (x*cos - y*sin, x*sin + y*cos)
    // rear-left local (-0.9, +0.925) -> world (0* - 0.925*1, 0*1 + 0.925*0) = (-0.925, -0.9)
    EXPECT_NEAR(corners[0].x, -half_w * sin90 - rear * (-sin90), 1e-6);  // indirect check
    EXPECT_NEAR(corners[0].x, rear * cos90 - half_w * sin90, 1e-6);
    EXPECT_NEAR(corners[0].y, rear * sin90 + half_w * cos90, 1e-6);

    // More directly: rear-left at 90 degrees
    // x = 0 + (-0.9)*cos90 - (0.925)*sin90 = 0 - 0 - 0.925 = -0.925
    // y = 0 + (-0.9)*sin90 + (0.925)*cos90 = -0.9 + 0 = -0.9
    EXPECT_NEAR(corners[0].x, -0.925, 1e-6);
    EXPECT_NEAR(corners[0].y, -0.9, 1e-6);

    // front-right local (3.7, -0.925) -> world (3.7*0 - (-0.925)*1, 3.7*1 + (-0.925)*0) = (0.925, 3.7)
    EXPECT_NEAR(corners[2].x, 0.925, 1e-6);
    EXPECT_NEAR(corners[2].y, 3.7, 1e-6);

    // Verify the rotated rectangle still has the same side lengths
    double left_len = corners[0].distanceTo(corners[1]);
    double right_len = corners[2].distanceTo(corners[3]);
    EXPECT_NEAR(left_len, front - rear, 1e-6);
    EXPECT_NEAR(right_len, front - rear, 1e-6);
}

// ---------------------------------------------------------------------------
// 9. Update reverse
// ---------------------------------------------------------------------------
TEST_F(VehicleModelTest, UpdateReverse) {
    Pose2D start{{0.0, 0.0}, 0.0};
    // Negative speed = reverse
    ControlCommand cmd{0.0, -1.0, 0.0};
    double dt = 1.0;

    Pose2D result = model.update(start, cmd, dt);

    EXPECT_NEAR(result.position.x, -1.0, 0.05);
    EXPECT_NEAR(result.position.y, 0.0, 0.05);
    EXPECT_NEAR(result.yaw, 0.0, 0.05);
}

// ---------------------------------------------------------------------------
// 10. Multiple steps convergence (circular path)
// ---------------------------------------------------------------------------
TEST_F(VehicleModelTest, MultipleStepsConvergence) {
    // With constant steering and speed, the vehicle should follow a circle.
    // We verify that after many small steps, the vehicle's distance from the
    // expected turning circle center remains approximately constant.
    Pose2D pose{{0.0, 0.0}, 0.0};
    double steering = 0.1;  // radians
    double speed = 1.0;
    double dt = 0.01;
    int num_steps = 5000;

    ControlCommand cmd{steering, speed, 0.0};

    // Expected turning radius and center (for left turn, center is to the left)
    double R = default_params.wheelbase / std::tan(steering);
    // Center of turning circle relative to start:
    //   At heading 0, left is +y direction
    double cx = 0.0;
    double cy = R;

    for (int i = 0; i < num_steps; ++i) {
        pose = model.update(pose, cmd, dt);
    }

    // After many steps, the vehicle should remain on a circle of radius R
    double dist_from_center = std::sqrt(
        (pose.position.x - cx) * (pose.position.x - cx) +
        (pose.position.y - cy) * (pose.position.y - cy));

    // Allow some tolerance due to Euler integration drift
    EXPECT_NEAR(dist_from_center, R, 0.5);

    // The total angle traversed should be approximately:
    double total_angle = (speed * std::tan(steering) / default_params.wheelbase) * dt * num_steps;
    double expected_yaw = normalizeAngle(total_angle);
    EXPECT_NEAR(normalizeAngle(pose.yaw), expected_yaw, 0.3);
}
