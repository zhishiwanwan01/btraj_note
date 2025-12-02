# Corridor 替换指南 - 从立方体到椭球

本文档说明如何将 Btraj 项目中的立方体 corridor 生成算法替换为椭球 corridor 算法。

## 目录

1. [核心文件概览](#核心文件概览)
2. [接口对齐要求](#接口对齐要求)
3. [替换实现步骤](#替换实现步骤)
4. [关键约束](#关键约束)
5. [辅助代码参考](#辅助代码参考)

---

## 核心文件概览

### 1. 数据结构定义

**文件**: `catkin_ws/src/Btraj/include/data_type.h` (行19-112)

当前的 `Cube` 结构体定义了 corridor 的数据格式：

```cpp
struct Cube {
    Eigen::MatrixXd vertex;        // 8个顶点坐标 (8×3)
    Eigen::Vector3d center;        // 中心点
    bool valid;                    // 有效性标记
    double t;                      // 时间分配
    std::vector<std::pair<double, double>> box;  // 轴边界 [x, y, z]
};
```

### 2. Corridor 生成相关代码

**文件**: `catkin_ws/src/Btraj/src/b_traj_node.cpp`

| 函数名 | 行号 | 功能描述 |
|--------|------|----------|
| `corridorGeneration()` | 782-828 | 主要生成逻辑，遍历路径点生成 corridor |
| `generateCube()` | 709-750 | 生成单个立方体 |
| `inflateCube()` | 634-707 | 膨胀立方体至最大无碰撞尺寸 |
| `corridorSimplify()` | 764-779 | 简化 corridor（移除被包含的立方体） |
| `timeAllocation()` | 1080-1194 | 根据路径长度分配时间 |

**调用位置**: 行1003 (A* 分支)
```cpp
corridor = corridorGeneration(gridPath);
```

### 3. A* 路径查找

**文件**: `catkin_ws/src/Btraj/include/a_star.h` (行61)

```cpp
std::vector<Eigen::Vector3d> getPath();
```

**使用示例** (`b_traj_node.cpp:994`):
```cpp
vector<Vector3d> gridPath = path_finder->getPath();
```

### 4. 轨迹优化使用 corridor

**文件**: `catkin_ws/src/Btraj/include/trajectory_generator.h` (行25-39)

```cpp
int BezierPloyCoeffGeneration(
    const vector<Cube>& corridor,  // 飞行走廊输入
    const MatrixXd& MQM,
    const MatrixXd& pos,
    const MatrixXd& vel,
    const MatrixXd& acc,
    const double maxVel,
    const double maxAcc,
    const int traj_order,
    const double minimize_order,
    const double margin,
    const bool& isLimitVel,
    const bool& isLimitAcc,
    double& obj,
    MatrixXd& PolyCoeff
);
```

**corridor 数据访问位置** (`trajectory_generator.cpp`):
- 行64-66: 提取首尾时间和段数
- 行165-166: 获取 cube 和时间缩放
- 行174-178: 使用 `box` 字段设置控制点边界约束
- 行302-304: 使用 `t` 字段计算加速度约束系数
- 行405-406: 使用 `t` 字段设置连续性约束
- 行498: 使用 `t` 字段构建代价矩阵

---

## 接口对齐要求

### 输入接口（从 A* 获取）

```cpp
// A* 输出格式
vector<Vector3d> gridPath = path_finder->getPath();
```

**数据格式**:
- `vector<Vector3d>`: 路径点序列
- 每个 `Vector3d`: 三维坐标 (x, y, z)

### 输出接口（提供给轨迹优化）

你的椭球 corridor 算法需要生成 `vector<Cube>` 类型数据。

#### 必需字段

| 字段 | 类型 | 说明 | 使用位置 |
|------|------|------|----------|
| **`box`** | `vector<pair<double, double>>` | 轴对齐边界盒，大小为3 | `trajectory_generator.cpp:174-178` |
| **`t`** | `double` | 时间分配（秒） | `trajectory_generator.cpp:166, 302-304, 405-406, 498` |
| **`center`** | `Eigen::Vector3d` | 中心点坐标 | `b_traj_node.cpp:1093, 1151` |

**`box` 字段格式**:
```cpp
box[0] = {x_min, x_max};  // x 轴边界
box[1] = {y_min, y_max};  // y 轴边界
box[2] = {z_min, z_max};  // z 轴边界
```

#### 可选字段（用于可视化）

| 字段 | 类型 | 说明 | 使用位置 |
|------|------|------|----------|
| **`vertex`** | `Eigen::MatrixXd` (8×3) | 8个顶点坐标（立方体） | `b_traj_node.cpp:1507-1523` (RViz可视化) |
| **`valid`** | `bool` | 有效性标记 | `corridorSimplify()` |

---

## 替换实现步骤

### 方案 1: 保持接口兼容（推荐）

如果你的椭球 corridor 可以用轴对齐包围盒（AABB）表示：

#### Step 1: 实现椭球到 AABB 转换函数

在 `b_traj_node.cpp` 中添加：

```cpp
// 椭球生成函数（示例）
struct Ellipsoid {
    Eigen::Vector3d center;
    Eigen::Vector3d axes;  // 半轴长度 (a, b, c)
    Eigen::Matrix3d rotation;  // 旋转矩阵（如果需要）
};

Ellipsoid generateEllipsoid(Vector3d pt, /* 其他参数 */) {
    Ellipsoid ellipsoid;
    // TODO: 实现你的椭球生成算法
    // 输入: 路径点 pt, 障碍物地图 collision_map
    // 输出: 椭球参数
    return ellipsoid;
}

// 椭球转 AABB（轴对齐包围盒）
Cube ellipsoidToAABB(const Ellipsoid& ellipsoid) {
    Cube cube;

    // 计算椭球的轴对齐包围盒
    // 对于轴对齐椭球：AABB = center ± axes
    // 对于旋转椭球：需要计算旋转后的边界

    double x_min = ellipsoid.center(0) - ellipsoid.axes(0);
    double x_max = ellipsoid.center(0) + ellipsoid.axes(0);
    double y_min = ellipsoid.center(1) - ellipsoid.axes(1);
    double y_max = ellipsoid.center(1) + ellipsoid.axes(1);
    double z_min = ellipsoid.center(2) - ellipsoid.axes(2);
    double z_max = ellipsoid.center(2) + ellipsoid.axes(2);

    // 设置 box 字段（必需）
    cube.box.resize(3);
    cube.box[0] = {x_min, x_max};
    cube.box[1] = {y_min, y_max};
    cube.box[2] = {z_min, z_max};

    // 设置 center 字段（必需）
    cube.center = ellipsoid.center;

    // 设置 vertex 字段（可选，用于可视化）
    cube.vertex = MatrixXd::Zero(8, 3);
    cube.vertex.row(0) = Vector3d(x_max, y_min, z_max);  // P1
    cube.vertex.row(1) = Vector3d(x_max, y_max, z_max);  // P2
    cube.vertex.row(2) = Vector3d(x_min, y_max, z_max);  // P3
    cube.vertex.row(3) = Vector3d(x_min, y_min, z_max);  // P4
    cube.vertex.row(4) = Vector3d(x_max, y_min, z_min);  // P5
    cube.vertex.row(5) = Vector3d(x_max, y_max, z_min);  // P6
    cube.vertex.row(6) = Vector3d(x_min, y_max, z_min);  // P7
    cube.vertex.row(7) = Vector3d(x_min, y_min, z_min);  // P8

    // 设置其他字段
    cube.valid = true;
    cube.t = 0.0;  // 稍后由 timeAllocation() 填充

    return cube;
}
```

#### Step 2: 实现椭球 corridor 生成函数

```cpp
vector<Cube> ellipsoidCorridorGeneration(vector<Vector3d> path_coord) {
    vector<Cube> cubeList;

    for (int i = 0; i < (int)path_coord.size(); i++) {
        Vector3d pt = path_coord[i];

        // 1. 基于路径点生成椭球
        Ellipsoid ellipsoid = generateEllipsoid(pt);

        // 2. 将椭球转换为 AABB
        Cube cube = ellipsoidToAABB(ellipsoid);

        // 3. 可选：简化逻辑（避免重复）
        // if (isContains(cube, lstcube)) continue;

        cubeList.push_back(cube);
    }

    return cubeList;
}
```

#### Step 3: 替换调用位置

在 `b_traj_node.cpp:1003` 修改：

```cpp
// 原代码:
// corridor = corridorGeneration(gridPath);

// 新代码:
corridor = ellipsoidCorridorGeneration(gridPath);
```

#### Step 4: 添加函数声明

在 `b_traj_node.cpp` 文件顶部的函数声明区域添加：

```cpp
// 在原有的 corridorGeneration() 声明后添加
vector<Cube> ellipsoidCorridorGeneration(vector<Vector3d> path_coord);
Cube ellipsoidToAABB(const Ellipsoid& ellipsoid);
```

---

## 关键约束

### 1. box 字段格式约束

**严格要求**: `box[i].first` 必须是最小值，`box[i].second` 必须是最大值

```cpp
// 正确示例
cube.box[0] = {-1.0, 1.0};  // x: [-1.0, 1.0]

// 错误示例（会导致轨迹优化失败）
cube.box[0] = {1.0, -1.0};  // 顺序错误！
```

**使用位置**: `trajectory_generator.cpp:174-178`
```cpp
lo_bound = (cube_.box[i].first + margin) / scale_k;
up_bound = (cube_.box[i].second - margin) / scale_k;
```

### 2. 时间分配处理

- **自动填充**: 调用 `timeAllocation(corridor)` 会自动计算并填充 `t` 字段
- **初始值**: 生成 corridor 时可将 `t` 设为 0.0
- **调用位置**: `b_traj_node.cpp:1008`

### 3. margin 处理

- **无需预留**: 轨迹优化器会自动应用 margin
- **参数来源**: `_cube_margin` (从 launch 文件读取)
- **应用位置**: `trajectory_generator.cpp:174` (`cube_.box[i].first + margin`)

### 4. 坐标系统

- **世界坐标系**: 使用 `collision_map` 的全局坐标系
- **地图 API**:
  - `LocationToGridIndex(Vector3d pos)`: 世界坐标 → 网格索引
  - `GridIndexToLocation(Vector3i idx)`: 网格索引 → 世界坐标
- **参考实现**: `b_traj_node.cpp:726-727`

### 5. 段数要求

- **最小段数**: 至少1段（起点到终点）
- **访问方式**:
  - `corridor.front()`: 第一段
  - `corridor.back()`: 最后一段
  - `corridor.size()`: 段数
- **使用位置**: `trajectory_generator.cpp:64-66`

---

## 辅助代码参考

### 1. 碰撞检测地图访问

```cpp
// 全局变量（在 b_traj_node.cpp 中）
sdf_tools::CollisionMapGrid* collision_map;

// 使用示例
Vector3d world_pos(1.0, 2.0, 3.0);
Vector3i grid_index = collision_map->LocationToGridIndex(world_pos);
double occupancy = collision_map->Get(grid_index.x(), grid_index.y(), grid_index.z()).first.occupancy;

// 检查是否有障碍物
if (occupancy > 0.5) {
    // 该位置被占据
}
```

### 2. 立方体膨胀参考

参考当前的 `inflateCube()` 函数 (`b_traj_node.cpp:634-707`)：

```cpp
// 核心逻辑：迭代膨胀直到碰撞
while (iter < max_inflate_iter) {
    // 1. 计算新的顶点索引
    // 2. 检查是否有碰撞
    // 3. 如果无碰撞，继续膨胀
    // 4. 如果有碰撞，返回上一次的结果
}
```

### 3. 时间分配参考

时间分配基于路径段长度（`timeAllocation()` 函数）：

```cpp
// 简化逻辑
for (int k = 0; k < corridor.size(); k++) {
    Vector3d p1 = points[k];
    Vector3d p2 = points[k + 1];
    double dist = (p2 - p1).norm();

    // 根据距离和速度限制计算时间
    double time = dist / max_velocity;
    corridor[k].t = time;
}
```

### 4. 可视化函数

参考 `visCorridor()` 函数 (`b_traj_node.cpp:1477-1530`)：

```cpp
// 发布 RViz Marker 显示 corridor
void visCorridor(vector<Cube> corridor) {
    // 为每个 corridor 段创建 CUBE marker
    for (int i = 0; i < corridor.size(); i++) {
        // 使用 vertex 或 box 字段计算位置和尺寸
        mk.pose.position.x = (corridor[i].vertex(0,0) + corridor[i].vertex(3,0)) / 2.0;
        mk.scale.x = corridor[i].vertex(0,0) - corridor[i].vertex(3,0);
        // ...
    }
}
```

---

## 完整替换检查清单

- [ ] 实现椭球生成算法 `generateEllipsoid()`
- [ ] 实现椭球到 AABB 转换 `ellipsoidToAABB()`
- [ ] 实现椭球 corridor 生成 `ellipsoidCorridorGeneration()`
- [ ] 确保 `box` 字段格式正确（min, max 顺序）
- [ ] 确保 `center` 字段正确设置
- [ ] 替换调用位置 (`b_traj_node.cpp:1003`)
- [ ] 测试时间分配是否正常工作
- [ ] 测试轨迹优化是否成功
- [ ] （可选）实现椭球可视化
- [ ] （可选）实现 corridor 简化逻辑

---

## 注意事项

1. **保留原函数**: 建议保留原 `corridorGeneration()` 函数作为备份
2. **添加开关**: 可在 launch 文件中添加参数切换立方体/椭球模式
3. **性能考虑**: 椭球生成可能比立方体更耗时，注意优化
4. **边界情况**: 处理路径点在地图边界外的情况
5. **调试输出**: 添加 ROS_INFO/ROS_WARN 输出中间结果

---

## 相关文档

- [CLAUDE.md](../CLAUDE.md) - 项目整体说明
- [trajectory_generator.cpp](../catkin_ws/src/Btraj/src/trajectory_generator.cpp) - 轨迹优化实现
- [data_type.h](../catkin_ws/src/Btraj/include/data_type.h) - 数据结构定义
- [b_traj_node.cpp](../catkin_ws/src/Btraj/src/b_traj_node.cpp) - 主规划节点

---

**最后更新**: 2025-11-27
