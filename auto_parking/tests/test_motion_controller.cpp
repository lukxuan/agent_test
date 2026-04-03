#include <gtest/gtest.h>
#include "auto_parking/motion_controller.h"
#include "auto_parking/types.h"
#include <cmath>
#include <vector>

using namespace auto_parking;

// ---------------------------------------------------------------------------
// Helper: build a straight path along the x-axis
// ---------------------------------------------------------------------------
std::vector<PathPoint> makeStraightPath(double start_x, double end_x, double y,
                                        double yaw, double speed) {
    std::vector<PathPoint> path;
    double step = 0.1;
    int n = static_cast<int>(std::abs(end_x - start_x) / step);
    if (n < 1) n = 1;
    double dx = (end_x - start_x) / n;

    Pose2D pose{{start_x, y}, yaw};
    for (int i = 0; i <= n; ++i) {
        PathPoint pt;
        pt.pose = pose;
        pt.curvature = 0.0;
        pt.speed = speed;
        path.push_back(pt);
        if (i < n) {
            pose.position.x += dx;
            pose.position.y += dx * std::tan(yaw);
        }
    }
    return path;
}

// ---------------------------------------------------------------------------
// Helper: build a left-turning arc path (counterclockwise)
// ---------------------------------------------------------------------------
std::vector<PathPoint> makeLeftArcPath(double cx, double cy, double radius,
                                       double start_angle, double end_angle,
                                       double speed) {
    std::vector<PathPoint> path;
    double angle_step = 0.05;
    int n = static_cast<int>(std::abs(end_angle - start_angle) / angle_step);
    if (n < 1) n = 1;
    double da = (end_angle - start_angle) / n;

    for (int i = 0; i <= n; ++i) {
        double a = start_angle + da * i;
        PathPoint pt;
        pt.pose.position.x = cx + radius * std::cos(a);
        pt.pose.position.y = cy + radius * std::sin(a);
        // Heading is tangent to the circle (perpendicular to radius, CCW)
        pt.pose.yaw = a + kPi / 2.0;
        pt.curvature = 1.0 / radius;
        pt.speed = speed;
        path.push_back(pt);
    }
    return path;
}

// ---------------------------------------------------------------------------
// Helper: build a right-turning arc path (clockwise)
// ---------------------------------------------------------------------------
std::vector<PathPoint> makeRightArcPath(double cx, double cy, double radius,
                                        double start_angle, double end_angle,
                                        double speed) {
    std::vector<PathPoint> path;
    double angle_step = 0.05;
    int n = static_cast<int>(std::abs(end_angle - start_angle) / angle_step);
    if (n < 1) n = 1;
    double da = (end_angle - start_angle) / n;

    for (int i = 0; i <= n; ++i) {
        double a = start_angle - da * i;  // clockwise: decreasing angle
        PathPoint pt;
        pt.pose.position.x = cx + radius * std::cos(a);
        pt.pose.position.y = cy + radius * std::sin(a);
        // Heading is tangent to the circle (perpendicular to radius, CW)
        pt.pose.yaw = a - kPi / 2.0;
        pt.curvature = -1.0 / radius;
        pt.speed = speed;
        path.push_back(pt);
    }
    return path;
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------
class MotionControllerTest : public ::testing::Test {
protected:
    PIDParams default_pid;
    MotionController controller{2.0, 0.8, default_pid};
};

// ---------------------------------------------------------------------------
// 1. Construction
// ---------------------------------------------------------------------------
TEST_F(MotionControllerTest, Construction) {
    MotionController ctrl;
    EXPECT_FALSE(ctrl.isFinished());
}

// ---------------------------------------------------------------------------
// 2. Reset
// ---------------------------------------------------------------------------
TEST_F(MotionControllerTest, Reset) {
    // Create a very short path and compute once near the end
    std::vector<PathPoint> path;
    Pose2D p1{{0.0, 0.0}, 0.0};
    Pose2D p2{{0.01, 0.0}, 0.0};
    path.push_back({p1, 0.0, 0.0});
    path.push_back({p2, 0.0, 0.0});

    MotionController ctrl;
    ctrl.compute(p2, path, 0.0);

    // After reset, controller should no longer be finished
    ctrl.reset();
    EXPECT_FALSE(ctrl.isFinished());
}

// ---------------------------------------------------------------------------
// 3. Pure pursuit straight line
// ---------------------------------------------------------------------------
TEST_F(MotionControllerTest, PurePursuitStraight) {
    auto path = makeStraightPath(0.0, 10.0, 0.0, 0.0, 1.0);
    ASSERT_FALSE(path.empty());

    Pose2D current{{0.0, 0.0}, 0.0};
    MotionController ctrl;
    auto cmd = ctrl.compute(current, path, 0.0);

    // On a straight path, steering should be near zero
    EXPECT_NEAR(cmd.steering_angle, 0.0, 0.05)
        << "Steering should be ~0 on straight path, got " << cmd.steering_angle;
}

// ---------------------------------------------------------------------------
// 4. Pure pursuit left turn
// ---------------------------------------------------------------------------
TEST_F(MotionControllerTest, PurePursuitLeftTurn) {
    // Build a left-turning arc path: center at (0, R), start at (R, 0)
    double R = 5.0;
    auto path = makeLeftArcPath(0.0, R, R, -kPi / 2.0, kPi / 2.0, 1.0);
    ASSERT_FALSE(path.empty());

    Pose2D current = path[0].pose;
    MotionController ctrl;
    auto cmd = ctrl.compute(current, path, 0.0);

    // For a left-turning path, steering should be positive
    EXPECT_GT(cmd.steering_angle, 0.0)
        << "Steering should be positive for left turn, got " << cmd.steering_angle;
}

// ---------------------------------------------------------------------------
// 5. Pure pursuit right turn
// ---------------------------------------------------------------------------
TEST_F(MotionControllerTest, PurePursuitRightTurn) {
    // Build a right-turning arc path: center at (0, -R), start at (R, 0)
    double R = 5.0;
    auto path = makeRightArcPath(0.0, -R, R, kPi / 2.0, -kPi / 2.0, 1.0);
    ASSERT_FALSE(path.empty());

    Pose2D current = path[0].pose;
    MotionController ctrl;
    auto cmd = ctrl.compute(current, path, 0.0);

    // For a right-turning path, steering should be negative
    EXPECT_LT(cmd.steering_angle, 0.0)
        << "Steering should be negative for right turn, got " << cmd.steering_angle;
}

// ---------------------------------------------------------------------------
// 6. PID control -- speed trending toward target
// ---------------------------------------------------------------------------
TEST_F(MotionControllerTest, PIDControl) {
    auto path = makeStraightPath(0.0, 10.0, 0.0, 0.0, 1.0);
    ASSERT_FALSE(path.empty());

    Pose2D current{{0.0, 0.0}, 0.0};
    MotionController ctrl;

    // Start with zero speed; PID should produce positive output to speed up
    double current_speed = 0.0;
    auto cmd = ctrl.compute(current, path, current_speed);

    // Target speed is 1.0, current is 0.0, so PID should produce positive speed
    EXPECT_GT(cmd.speed, 0.0)
        << "PID should command positive speed to reach target of 1.0 from 0.0";

    // Simulate a few more steps -- speed should continue trending toward target
    for (int i = 0; i < 50; ++i) {
        cmd = ctrl.compute(current, path, current_speed);
        current_speed += cmd.speed * 0.01;  // crude integration
        current_speed = std::clamp(current_speed, -5.0, 5.0);
    }

    // After several iterations, speed should be closer to target
    EXPECT_GT(current_speed, 0.0)
        << "Speed should be positive after PID iterations";
}

// ---------------------------------------------------------------------------
// 7. Finished detection
// ---------------------------------------------------------------------------
TEST_F(MotionControllerTest, FinishedDetection) {
    // Create a short path with only 2 points close together
    std::vector<PathPoint> path;
    Pose2D p1{{0.0, 0.0}, 0.0};
    Pose2D p2{{0.05, 0.0}, 0.0};  // Only 5 cm apart
    path.push_back({p1, 0.0, 0.0});
    path.push_back({p2, 0.0, 0.0});

    MotionController ctrl;

    // Start near the end point
    Pose2D near_end{{0.04, 0.0}, 0.0};
    auto cmd = ctrl.compute(near_end, path, 0.0);

    // Distance from near_end to p2 is 0.01 m, which is < 0.1 m threshold
    // and closest_idx should be >= path.size() - 2
    EXPECT_TRUE(ctrl.isFinished())
        << "Controller should report finished when very close to path end";
}

// ---------------------------------------------------------------------------
// 8. Empty path
// ---------------------------------------------------------------------------
TEST_F(MotionControllerTest, EmptyPath) {
    std::vector<PathPoint> empty_path;
    MotionController ctrl;

    Pose2D current{{0.0, 0.0}, 0.0};
    auto cmd = ctrl.compute(current, empty_path, 0.0);

    EXPECT_DOUBLE_EQ(cmd.steering_angle, 0.0);
    EXPECT_DOUBLE_EQ(cmd.speed, 0.0);
    EXPECT_DOUBLE_EQ(cmd.curvature, 0.0);
    EXPECT_TRUE(ctrl.isFinished());
}

// ---------------------------------------------------------------------------
// 9. Lookahead adaptation (different speed paths)
// ---------------------------------------------------------------------------
TEST_F(MotionControllerTest, LookaheadAdaptation) {
    // Create two paths: one with low speed, one with high speed
    auto low_speed_path = makeStraightPath(0.0, 10.0, 0.0, 0.0, 0.5);
    auto high_speed_path = makeStraightPath(0.0, 10.0, 0.0, 0.0, 3.0);

    ASSERT_FALSE(low_speed_path.empty());
    ASSERT_FALSE(high_speed_path.empty());

    Pose2D current{{0.0, 0.0}, 0.0};

    MotionController ctrl_low;
    auto cmd_low = ctrl_low.compute(current, low_speed_path, 0.0);

    MotionController ctrl_high;
    auto cmd_high = ctrl_high.compute(current, high_speed_path, 0.0);

    // Both should produce valid commands without errors
    EXPECT_FALSE(std::isnan(cmd_low.steering_angle));
    EXPECT_FALSE(std::isnan(cmd_high.steering_angle));

    // On a straight path, both should produce near-zero steering
    EXPECT_NEAR(cmd_low.steering_angle, 0.0, 0.05);
    EXPECT_NEAR(cmd_high.steering_angle, 0.0, 0.05);

    // The controller should handle both speed regimes
    EXPECT_FALSE(std::isnan(cmd_low.speed));
    EXPECT_FALSE(std::isnan(cmd_high.speed));
}
