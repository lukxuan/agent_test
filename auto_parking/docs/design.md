# Auto Parking System - System Design Document

## 1. 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                   ParkingSystem (Facade)                  │
│                    startParking() / update()              │
└──────────┬──────────────┬──────────────┬─────────────────┘
           │              │              │
     ┌─────▼─────┐ ┌─────▼─────┐ ┌──────▼──────┐
     │ Perception│ │PathPlanner│ │   Motion    │
     │  Module   │ │  Module   │ │ Controller  │
     └─────┬─────┘ └─────┬─────┘ └──────┬──────┘
           │              │              │
     ┌─────▼─────┐ ┌─────▼─────┐ ┌──────▼──────┐
     │  Sensor   │ │  Vehicle  │ │   PID +     │
     │  Data     │ │  Model    │ │ PurePursuit │
     └───────────┘ └───────────┘ └─────────────┘
                        │
                ┌───────▼───────┐
                │  Bicycle      │
                │  Kinematic    │
                │  Model        │
                └───────────────┘
```

### 三层架构

| 层次 | 模块 | 职责 |
|------|------|------|
| 感知层 | Perception | 传感器数据处理 → 车位检测 |
| 决策规划层 | PathPlanner | 车位分析 → 无碰撞路径生成 |
| 控制层 | MotionController | 路径追踪 → 控制指令输出 |

## 2. 核心数据结构

### 2.1 坐标系定义

- 采用车辆后轴中心为原点的局部坐标系
- X 轴指向车辆前进方向，Y 轴指向车辆左侧
- yaw 角为车辆航向角，逆时针为正

### 2.2 数据类型

```cpp
// 二维点
struct Point2D { double x, y; };

// 二维位姿（位置 + 航向）
struct Pose2D { Point2D position; double yaw; };

// 车辆参数
struct VehicleParams {
    double wheelbase;          // 轴距 (m)
    double track_width;        // 轮距 (m)
    double length;             // 车长 (m)
    double width;              // 车宽 (m)
    double rear_overhang;      // 后悬 (m)
    double front_overhang;     // 前悬 (m)
    double min_turn_radius;    // 最小转弯半径 (m)
    double max_steering_angle; // 最大转向角 (rad)
    double max_speed;          // 最大速度 (m/s)
    double max_reverse_speed;  // 最大倒车速度 (m/s)
};

// 车位信息
struct ParkingSlot {
    enum class Type { PARALLEL, PERPENDICULAR, DIAGONAL };
    Type type;
    Point2D corners[4];        // 车位四角坐标（逆时针）
    double width;              // 车位宽度 (m)
    double depth;              // 车位深度/长度 (m)
    double angle;              // 车位朝向角 (rad)
    Point2D center;            // 车位中心点
};

// 传感器数据
struct SensorData {
    std::vector<double> ultrasonic;  // 超声波距离值序列 (m)
    double timestamp;
};

// 路径点
struct PathPoint {
    Pose2D pose;
    double curvature;          // 曲率 (1/m)
    double speed;              // 目标速度 (m/s)
};

// 控制指令
struct ControlCommand {
    double steering_angle;     // 目标转向角 (rad), 正值左转
    double speed;              // 目标速度 (m/s), 正值前进, 负值倒车
    double curvature;          // 当前路径曲率 (1/m)
};

// 泊车状态
enum class ParkingState {
    IDLE,        // 空闲
    SEARCHING,   // 搜索车位
    PLANNING,    // 路径规划中
    EXECUTING,   // 泊车执行中
    COMPLETED,   // 泊车完成
    FAILED       // 泊车失败
};
```

## 3. 车辆运动学模型

### 3.1 Bicycle Model（自行车模型）

```
         δ (前轮转角)
          /
    □────/──────→ x (前进方向)
    │   /
    │  / L (轴距)
    │ /
    ●──────→
   (后轴中心)
```

**运动学方程：**

```
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = v * tan(δ) / L
```

其中：
- `(x, y)` = 后轴中心位置
- `θ` = 航向角
- `v` = 后轴中心线速度
- `δ` = 前轮转向角
- `L` = 轴距

**离散化更新（欧拉法）：**

```cpp
x_new = x + v * cos(θ) * dt
y_new = y + v * sin(θ) * dt
θ_new = θ + v * tan(δ) / L * dt
```

### 3.2 约束条件

- 转向角范围：`|δ| ≤ δ_max`（通常 35° ≈ 0.611 rad）
- 最小转弯半径：`R_min = L / tan(δ_max)`
- 最大曲率：`κ_max = 1 / R_min = tan(δ_max) / L`

## 4. 路径规划算法

### 4.1 平行泊车 (Parallel Parking)

**策略：两段式圆弧 + 直线**

```
阶段1: 向右后方圆弧行驶（转向角 = -δ_max）
阶段2: 向左后方圆弧行驶（转向角 = +δ_max）
阶段3: 直线后退微调（可选）
```

**路径生成：**

1. 计算起始圆弧半径 `R = L / tan(δ_max)`
2. 第一段圆弧：从起始位姿沿半径 R 向右后方行驶角度 α
3. 第二段圆弧：切换转向方向，沿半径 R 向左后方行驶角度 α
4. 计算目标位姿是否与车位对齐，如需调整则添加直线段

**关键参数：**
- 两段圆弧的转角 `α` 由车位深度和车辆尺寸决定
- 圆弧之间的切换点需满足连续性约束

### 4.2 垂直泊车 (Perpendicular Parking)

**策略：前进调整位置 + 倒车圆弧入库**

```
阶段1: 前进至起始位姿（车尾对准车位入口）
阶段2: 直线后退一段距离
阶段3: 向右转向倒车圆弧进入车位（δ = -δ_max）
阶段4: 回正方向盘继续后退
阶段5: 向左转向微调对中（δ = +δ_max）
```

**路径生成：**

1. 计算车辆需要后退的距离，使后轴到达圆弧起始点
2. 以最大转向角倒车，沿圆弧进入车位
3. 回正后直线后退至目标位姿
4. 如有偏移，小角度调整对中

### 4.3 斜向泊车 (Diagonal Parking)

**策略：角度自适应弧线**

```
阶段1: 根据车位角度计算起始位姿
阶段2: 后退圆弧（曲率根据车位角度自适应）
阶段3: 回正后退入位
```

**自适应曲率计算：**

```
κ = tan(θ_target - θ_current) / L_adapt
```

其中 `L_adapt` 是根据车位角度和距离调整的等效轴距参数，使得圆弧终点自然对齐车位方向。

### 4.4 碰撞检测

**方法：分离轴定理（SAT）**

- 将车辆表示为 AABB（轴对齐包围盒）或 OBB（有向包围盒）
- 沿两个矩形的 4 条边的法线方向投影
- 若任一方向上投影不重叠，则无碰撞
- 若所有方向投影均重叠，则发生碰撞

**优化：**
- 仅检测路径点附近的障碍物
- 使用时间步长离散化检查

## 5. 运动控制器

### 5.1 横向控制 — Pure Pursuit（纯追踪）

**原理：** 在路径前方找一个目标点，计算到达该点所需的转向角。

```
           目标点 (x_g, y_g)
          /
         / L_d (前视距离)
        /
   ●───/──────→
 (后轴)
```

**算法步骤：**

1. 在参考路径上找到距离当前位姿 `L_d` 处的目标点
2. 计算目标点相对车辆的偏转角 `α`
3. 转向角计算：`δ = atan(2 * L * sin(α) / L_d)`

**前视距离自适应：**
```
L_d = k_v * |v| + L_base
```
- `k_v` = 速度系数（通常 0.5 ~ 1.5）
- `L_base` = 基础前视距离（通常 1.0 ~ 3.0m）

### 5.2 纵向控制 — PID 控制器

**速度控制：**

```
u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de(t)/dt
```

- `e(t)` = 目标速度 - 当前速度
- 推荐参数：`Kp=1.0, Ki=0.1, Kd=0.05`

### 5.3 复合控制

```
ControlCommand {
    steering_angle = PurePursuit(位姿, 路径)    // 横向
    speed          = PID(目标速度, 当前速度)     // 纵向
}
```

## 6. 状态机设计

```
         ┌──────────┐
    ┌───►│   IDLE   │◄──────────────────┐
    │    └────┬─────┘                   │
    │         │ startParking()          │ reset()
    │         ▼                         │
    │    ┌────────────┐                │
    │    │ SEARCHING  │                │
    │    └────┬───────┘                │
    │         │ 车位检测成功             │
    │         ▼                        │
    │    ┌────────────┐                │
    │    │  PLANNING  │                │
    │    └────┬───────┘                │
    │    ┌────┴──────────┐             │
    │    │规划成功       │规划失败      │
    │    ▼               └──────┐      │
    │ ┌────────────┐            │      │
    │ │ EXECUTING  │◄───────────┘      │
    │ └──┬────┬────┘                   │
    │    │    │                         │
    │    │    │ 到达目标位姿            │
    │    │    ▼                         │
    │    │ ┌───────────┐               │
    │    │ │ COMPLETED │───────────────┘
    │    │ └───────────┘
    │    │ 执行超时/碰撞
    │    ▼
    │ ┌────────┐
    └─│ FAILED │
      └────────┘
```

## 7. 模块接口

### 7.1 VehicleModel

```cpp
class VehicleModel {
public:
    explicit VehicleModel(VehicleParams params);
    Pose2D update(Pose2D current, ControlCommand cmd, double dt);
    VehicleParams getParams() const;
    double turningRadius(double steering_angle) const;
};
```

### 7.2 Perception

```cpp
class Perception {
public:
    std::optional<ParkingSlot> detectParkingSlot(const SensorData& data, VehicleParams params);
};
```

### 7.3 PathPlanner

```cpp
class PathPlanner {
public:
    virtual ~PathPlanner() = default;
    virtual std::vector<PathPoint> plan(Pose2D start, ParkingSlot target, VehicleParams params) = 0;
    virtual bool validatePath(const std::vector<PathPoint>& path, VehicleParams params) = 0;
};

class ParallelPlanner : public PathPlanner { ... };
class PerpendicularPlanner : public PathPlanner { ... };
class DiagonalPlanner : public PathPlanner { ... };
```

### 7.4 MotionController

```cpp
class MotionController {
public:
    MotionController(double lookahead_base, double lookahead_k, PIDParams pid);
    ControlCommand compute(Pose2D current, const std::vector<PathPoint>& path, double current_speed);
};
```

### 7.5 ParkingSystem

```cpp
class ParkingSystem {
public:
    ParkingSystem(VehicleParams params);
    void startParking(ParkingSlot slot);
    ControlCommand update(Pose2D current_pose, double current_speed, double dt);
    ParkingState getState() const;
    const std::vector<PathPoint>& getPath() const;
};
```
