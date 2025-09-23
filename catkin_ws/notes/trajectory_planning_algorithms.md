# 轨迹规划算法详解

本文档详细解释BTraj系统中使用的轨迹规划算法，包括Fast Marching Method、A*搜索、Bezier曲线优化等。

## 目录
1. [算法概览](#overview)
2. [Fast Marching Method详解](#fast-marching)
3. [A*搜索算法](#a-star)
4. [安全走廊生成](#safety-corridor)
5. [Bezier轨迹优化](#bezier-optimization)
6. [时间分配策略](#time-allocation)

---

## 1. 算法概览 {#overview}

### 1.1 总体架构

BTraj采用两阶段轨迹规划方法：

```
输入: 起点、终点、地图
         ↓
阶段1: 路径搜索 (Front-end)
    ├── Fast Marching Method (FM*)
    └── A* 搜索算法
         ↓
    粗糙路径点序列
         ↓
阶段2: 轨迹优化 (Back-end)
    ├── 安全走廊生成
    ├── 时间分配
    └── Bezier曲线优化
         ↓
输出: 平滑可执行轨迹
```

### 1.2 算法选择机制

通过参数`_is_use_fm`控制：
- `true`: 使用Fast Marching Method
- `false`: 使用A*算法

**选择建议**:
- FM*: 适合复杂环境，生成更平滑路径
- A*: 计算速度快，适合简单环境

---

## 2. Fast Marching Method详解 {#fast-marching}

### 2.1 物理原理

Fast Marching模拟波在介质中的传播：
- **波源**: 目标点
- **介质**: 速度场 (越接近障碍物速度越慢)
- **波前**: 等时间到达线
- **路径**: 沿梯度方向的最快到达路径

### 2.2 速度场设计

```cpp
double velMapping(double d, double max_v) {
    if (d <= 0.25)
        vel = 2.0 * d * d;                    // 二次减速区
    else if (d > 0.25 && d <= 0.75)
        vel = 1.5 * d - 0.25;                // 线性过渡区
    else if (d > 0.75 && d <= 1.0)
        vel = -2.0 * (d - 1.0) * (d - 1.0) + 1;  // 二次加速区
    else
        vel = 1.0;                           // 自由空间

    return vel * max_v;
}
```

**速度场特性**:
```
速度
 ↑
 │     ╭─────────── max_v
 │    ╱
 │   ╱
 │  ╱
 │ ╱
 │╱
 └────────────────────→ 距离障碍物
 0   0.25  0.75  1.0
```

### 2.3 算法实现步骤

#### 步骤1: 构建速度场
```cpp
// 为每个网格点计算到最近障碍物距离
auto EDT = collision_map_local->ExtractDistanceField(oob_value);

// 根据距离计算允许速度
for (每个网格点 i,j,k) {
    double d = sqrt(EDT.distance_square) * _resolution;
    double flow_vel = velMapping(d, max_vel);
    grid_fmm[idx].setOccupancy(flow_vel);
}
```

#### 步骤2: 初始化求解器
```cpp
// 设置起点和终点
Coord3D goal_point = {startIdx[0], startIdx[1], startIdx[2]};
Coord3D init_point = {endIdx[0], endIdx[1], endIdx[2]};

// 创建FMM*求解器
Solver<FMGrid3D>* fm_solver = new FMMStar<FMGrid3D>("FMM*_Dist", TIME);
fm_solver->setEnvironment(&grid_fmm);
fm_solver->setInitialAndGoalPoints(startIndices, goalIdx);
```

#### 步骤3: 求解到达时间
```cpp
if (fm_solver->compute(max_vel) == -1) {
    // 无解处理
    return;
}
```

#### 步骤4: 路径提取
```cpp
// 使用梯度下降法提取路径
GradientDescent<FMGrid3D> grad3D;
if (grad3D.gradient_descent(grid_fmm, goalIdx, path3D, path_vels, time) == -1) {
    // 路径提取失败
    return;
}
```

### 2.4 优势与局限

**优势**:
- 考虑速度约束，生成时间最优路径
- 自然避开障碍物密集区域
- 路径相对平滑

**局限**:
- 计算复杂度较高 O(N log N)
- 需要构建完整的距离场
- 内存消耗较大

---

## 3. A*搜索算法 {#a-star}

### 3.1 基本原理

A*是一种启发式图搜索算法，结合了：
- **Dijkstra算法**: 保证最优性
- **贪婪搜索**: 利用启发式信息提高效率

### 3.2 评估函数

```cpp
f(n) = g(n) + h(n)
```

其中：
- `g(n)`: 从起点到节点n的实际代价
- `h(n)`: 从节点n到目标的启发式估计 (欧几里得距离)
- `f(n)`: 节点n的总评估值

### 3.3 算法实现

#### 3.3.1 初始化
```cpp
// 创建路径搜索器
gridPathFinder* path_finder = new gridPathFinder();
path_finder->linkLocalMap(collision_map_local, _local_origin);
```

#### 3.3.2 搜索过程
```cpp
path_finder->AstarSearch(_start_pt, _end_pt);
vector<Vector3d> gridPath = path_finder->getPath();
vector<GridNodePtr> searchedNodes = path_finder->getVisitedNodes();
```

#### 3.3.3 邻居扩展
A*搜索考虑26个邻居方向 (3D空间):
- 6个面邻居: 代价 = 1.0
- 12个边邻居: 代价 = √2 ≈ 1.414
- 8个顶点邻居: 代价 = √3 ≈ 1.732

### 3.4 优势与局限

**优势**:
- 计算速度快 O(b^d)
- 内存占用少
- 实现简单

**局限**:
- 路径可能不够平滑
- 不考虑动力学约束
- 在网格上搜索，分辨率受限

---

## 4. 安全走廊生成 {#safety-corridor}

### 4.1 基本概念

安全走廊是一系列无碰撞的立方体，约束轨迹优化在安全区域内。

```
障碍物 ████████████████████
      █                  █
      █  [立方体1]      █
      █      ↓           █
      █  [立方体2]      █
      █      ↓           █
      █  [立方体3]      █
      █                  █
障碍物 ████████████████████
```

### 4.2 立方体膨胀算法

#### 4.2.1 生成初始立方体
```cpp
Cube generateCube(Vector3d pt) {
    // 将路径点对齐到网格
    Vector3i pc_index = collision_map->LocationToGridIndex(pt);
    Vector3d pc_coord = collision_map->GridIndexToLocation(pc_index);

    // 创建单位立方体
    cube.center = pc_coord;
    // 设置8个顶点坐标...
}
```

#### 4.2.2 六方向膨胀
立方体按以下顺序膨胀：
1. Y- (左侧面)
2. Y+ (右侧面)
3. X+ (前侧面)
4. X- (后侧面)
5. Z+ (上侧面)
6. Z- (下侧面)

```cpp
// 以Y-方向膨胀为例
for (id_y = vertex_idx(0, 1); id_y >= y_lo; id_y--) {
    bool collide = false;
    // 检查该层所有体素
    for (id_x = ...; id_z = ...) {
        if (collision_map->Get(id_x, id_y, id_z).occupancy > 0.5) {
            collide = true;
            break;
        }
    }
    if (collide) break;
}
```

#### 4.2.3 碰撞检测
- 遍历立方体表面的所有体素
- 检查占用概率是否超过阈值 (0.5)
- 一旦检测到碰撞立即停止膨胀

#### 4.2.4 迭代膨胀
```cpp
int iter = 0;
while (iter < _max_inflate_iter) {
    // 六个方向膨胀
    // 检查是否还能继续膨胀
    if (vertex_idx_lst == vertex_idx) break;  // 无变化则停止
    vertex_idx_lst = vertex_idx;
    iter++;
}
```

### 4.3 走廊简化

#### 4.3.1 包含关系检查
```cpp
bool isContains(Cube cube1, Cube cube2) {
    // 检查cube1是否完全包含cube2
    return (cube1边界 >= cube2边界);
}
```

#### 4.3.2 冗余移除
```cpp
void corridorSimplify(vector<Cube>& cubicList) {
    for (int j = cubicList.size()-1; j >= 0; j--) {
        for (int k = j-1; k >= 0; k--) {
            if (isContains(cubicList[j], cubicList[k])) {
                cubicList[k].valid = false;  // 标记为无效
            }
        }
    }
    // 移除无效立方体
}
```

---

## 5. Bezier轨迹优化 {#bezier-optimization}

### 5.1 Bezier曲线表示

#### 5.1.1 数学形式
n阶Bezier曲线定义为：
```
B(t) = Σᵢ₌₀ⁿ Pᵢ Bᵢⁿ(t),  t ∈ [0,1]
```

其中Bernstein基函数：
```
Bᵢⁿ(t) = C(n,i) tⁱ (1-t)ⁿ⁻ⁱ
```

#### 5.1.2 控制点意义
- `P₀`: 起始点 (轨迹通过)
- `Pₙ`: 终止点 (轨迹通过)
- `P₁...Pₙ₋₁`: 中间控制点 (影响形状)

#### 5.1.3 导数性质
```cpp
// 位置
pos(t) = Σᵢ Pᵢ Bᵢⁿ(t)

// 速度 (一阶导数)
vel(t) = n Σᵢ (Pᵢ₊₁ - Pᵢ) Bᵢⁿ⁻¹(t)

// 加速度 (二阶导数)
acc(t) = n(n-1) Σᵢ (Pᵢ₊₂ - 2Pᵢ₊₁ + Pᵢ) Bᵢⁿ⁻²(t)
```

### 5.2 优化问题建模

#### 5.2.1 目标函数
最小化轨迹的平滑性代价：
```
minimize ∫₀ᵀ ||d^k r(t)/dt^k||² dt
```

通常选择 k=3 (最小化jerk) 或 k=4 (最小化snap)。

#### 5.2.2 约束条件

**等式约束** (边界条件):
```cpp
// 起始约束
pos(0) = start_pos
vel(0) = start_vel
acc(0) = start_acc

// 终止约束
pos(T) = end_pos
vel(T) = 0  // 期望终止速度为0
acc(T) = 0  // 期望终止加速度为0
```

**不等式约束** (安全和动力学):
```cpp
// 安全约束: 轨迹在走廊内
pos(t) ∈ 安全走廊, ∀t

// 速度约束
||vel(t)|| ≤ v_max, ∀t

// 加速度约束
||acc(t)|| ≤ a_max, ∀t
```

### 5.3 二次规划求解

#### 5.3.1 标准形式
```
minimize   x^T Q x + c^T x
subject to Ax ≤ b      (不等式约束)
           Aeq x = beq (等式约束)
```

其中：
- `x`: 控制点向量 [P₀ₓ, P₀ᵧ, P₀ᵤ, P₁ₓ, ...]
- `Q`: 平滑性代价矩阵
- `A`: 约束矩阵

#### 5.3.2 MOSEK求解器
```cpp
_trajectoryGenerator.BezierPloyCoeffGeneration(
    corridor,          // 安全走廊
    _MQM,             // 代价矩阵
    pos, vel, acc,    // 边界条件
    _MAX_Vel, _MAX_Acc, // 动力学限制
    _traj_order,      // Bezier阶数
    _minimize_order,  // 优化阶数
    _cube_margin,     // 安全裕度
    _is_limit_vel,    // 是否限制速度
    _is_limit_acc,    // 是否限制加速度
    obj,              // 输出: 目标函数值
    _bezier_coeff     // 输出: Bezier系数
);
```

---

## 6. 时间分配策略 {#time-allocation}

### 6.1 基本原理

合理的时间分配确保：
- 轨迹满足动力学约束
- 执行时间尽可能短
- 运动平滑舒适

### 6.2 时间分配算法

#### 6.2.1 输入处理
```cpp
void timeAllocation(vector<Cube>& corridor) {
    vector<Vector3d> points;
    points.push_back(_start_pt);

    // 使用走廊中心点作为路径点
    for (int i = 1; i < corridor.size(); i++)
        points.push_back(corridor[i].center);

    points.push_back(_end_pt);
}
```

#### 6.2.2 逐段时间计算
对每个路径段 `i → i+1`：

```cpp
Vector3d p0 = points[i];      // 起始点
Vector3d p1 = points[i+1];    // 终止点
Vector3d d = p1 - p0;         // 位移向量
double D = d.norm();          // 距离

Vector3d v0 = (i == 0) ? _start_vel : Vector3d::Zero();
double V0 = v0.dot(d/D);      // 沿路径方向的初始速度
```

#### 6.2.3 运动模型
考虑三种运动阶段：

**阶段1: 加速** (时间 t₁)
```cpp
double acct = (_Vel - V0) / _Acc;           // 加速时间
double accd = V0*acct + 0.5*_Acc*acct*acct; // 加速距离
```

**阶段2: 匀速** (时间 t₂)
```cpp
double const_vel_dist = D - accd - dccd;    // 匀速距离
double t2 = const_vel_dist / _Vel;          // 匀速时间
```

**阶段3: 减速** (时间 t₃)
```cpp
double dcct = _Vel / _Acc;                  // 减速时间
double dccd = 0.5 * _Acc * dcct * dcct;     // 减速距离
```

#### 6.2.4 特殊情况处理

**情况1**: 距离太短，无法达到最大速度
```cpp
if (D < accd + dccd) {
    // 重新计算加速和减速时间
    double t2 = (-aV0 + sqrt(aV0*aV0 + _Acc*D - aV0*aV0/2)) / _Acc;
    dtxyz = t1 + t2 + t3;
}
```

**情况2**: 初始速度过大，需要先减速
```cpp
if (D < aV0*aV0 / (2*_Acc)) {
    double t1 = (V0 < 0) ? 2.0*aV0/_Acc : 0.0;
    dtxyz = t1 + t2;
}
```

### 6.3 时间优化

#### 6.3.1 Fast Marching时间修正
如果使用FM*算法，已有初始时间估计：
```cpp
if (dtxyz < tmp_time[i] * 0.5) {
    tmp_time[i] = dtxyz;  // 使用动力学时间替代FM时间
}
```

#### 6.3.2 最终时间分配
```cpp
for (int i = 0; i < corridor.size(); i++) {
    corridor[i].t = tmp_time[i];  // 分配给每个走廊段
}
```

---

## 总结

BTraj系统集成了多种先进的轨迹规划技术：

1. **Fast Marching/A***: 前端路径搜索，生成初始路径
2. **安全走廊**: 膨胀算法生成无碰撞约束区域
3. **Bezier优化**: 后端轨迹优化，生成平滑轨迹
4. **时间分配**: 确保轨迹满足动力学约束

这种分层设计既保证了安全性，又实现了轨迹的平滑性和可执行性，是现代无人机轨迹规划的典型范例。

理解这些算法原理后，可以：
- 调整算法参数以适应不同场景
- 改进算法性能和鲁棒性
- 扩展系统功能 (如多机协同、动态障碍物等)