# BTraj轨迹规划系统 - 背景知识指南

本文档为C++、ROS和运动规划背景较弱的开发者提供详细的背景知识说明，帮助理解`b_traj_node.cpp`代码。

## 目录
1. [C++基础概念](#cpp-basics)
2. [ROS机器人操作系统](#ros-basics)
3. [无人机轨迹规划基础](#trajectory-planning)
4. [数学基础](#math-foundations)
5. [算法原理](#algorithms)
6. [代码架构解析](#code-architecture)

---

## 1. C++基础概念 {#cpp-basics}

### 1.1 命名空间 (Namespace)
```cpp
using namespace std;      // 标准库命名空间
using namespace Eigen;    // Eigen数学库命名空间
```

**作用**: 避免函数名和类名冲突，简化代码书写。
- `std::vector` 可以简写为 `vector`
- `Eigen::Vector3d` 可以简写为 `Vector3d`

### 1.2 指针和动态内存分配
```cpp
CollisionMapGrid* collision_map = new CollisionMapGrid();
```

**概念说明**:
- `*` 表示指针类型，存储对象的内存地址
- `new` 在堆内存中创建对象，返回地址
- 使用指针可以在运行时决定对象大小和生命周期
- **注意**: 使用`new`分配的内存需要用`delete`释放

### 1.3 引用 (Reference)
```cpp
void rcvWaypointsCallback(const nav_msgs::Path& wp)
```

**概念说明**:
- `&` 表示引用，是对象的别名
- `const` 表示只读，不能修改原对象
- 引用避免了对象拷贝，提高性能

### 1.4 静态变量 (Static Variable)
```cpp
static tf::TransformBroadcaster br;
```

**作用**:
- 变量只初始化一次，在函数多次调用间保持状态
- 避免重复创建昂贵的对象

---

## 2. ROS机器人操作系统 {#ros-basics}

### 2.1 ROS概念架构

ROS (Robot Operating System) 是一个分布式机器人软件框架。

#### 核心概念:
- **节点 (Node)**: 独立的可执行程序，执行特定功能
- **话题 (Topic)**: 节点间异步通信的数据通道
- **消息 (Message)**: 在话题上传输的数据结构
- **服务 (Service)**: 节点间同步通信的请求-响应机制

#### 通信模式:
```
发布者节点 --[话题]--> 订阅者节点
    |                        |
传感器数据                  处理算法
```

### 2.2 关键ROS组件

#### 2.2.1 节点句柄 (NodeHandle)
```cpp
ros::NodeHandle nh("~");
```
- 管理节点的ROS通信接口
- `"~"` 表示私有命名空间，用于参数管理

#### 2.2.2 发布者和订阅者
```cpp
// 订阅者 - 接收数据
ros::Subscriber _map_sub = nh.subscribe("map", 1, rcvPointCloudCallBack);

// 发布者 - 发送数据
ros::Publisher _traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 10);
```

**参数说明**:
- `"map"`: 话题名称
- `1`: 队列大小
- `rcvPointCloudCallBack`: 回调函数
- `10`: 发布队列大小

#### 2.2.3 参数服务器
```cpp
nh.param("planning/max_vel", _MAX_Vel, 1.0);
```
- 从launch文件或命令行读取配置参数
- 第三个参数是默认值

### 2.3 重要ROS消息类型

#### 2.3.1 几何消息
```cpp
geometry_msgs::PoseStamped    // 带时间戳的位姿
nav_msgs::Odometry           // 里程计（位置+速度）
nav_msgs::Path              // 路径点序列
```

#### 2.3.2 传感器消息
```cpp
sensor_msgs::PointCloud2     // 3D点云数据
```

#### 2.3.3 可视化消息
```cpp
visualization_msgs::Marker       // 单个3D标记
visualization_msgs::MarkerArray  // 3D标记数组
```

### 2.4 TF坐标变换系统

TF管理机器人系统中多个坐标系的关系：

```
world (世界坐标系)
  └── quadrotor (机器人坐标系)
      └── camera (相机坐标系)
```

**作用**:
- 统一不同传感器的数据到同一坐标系
- 支持RViz中的3D可视化

---

## 3. 无人机轨迹规划基础 {#trajectory-planning}

### 3.1 轨迹规划问题定义

#### 输入:
- **起始状态**: 位置、速度、加速度 `[x₀, y₀, z₀, vₓ₀, vᵧ₀, vᵢ₀, aₓ₀, aᵧ₀, aᵢ₀]`
- **目标状态**: 期望位置 `[xf, yf, zf]`
- **环境地图**: 3D障碍物分布
- **动力学约束**: 最大速度、加速度限制

#### 输出:
- **可行轨迹**: 从起点到终点的连续路径
- **时间参数化**: 每个时刻的期望状态

### 3.2 两阶段规划方法

本系统采用分层规划策略：

#### 前端 (Front-end): 路径搜索
- **Fast Marching*** 或 **A*** 算法
- 在离散化地图上找到无碰撞路径
- 输出: 路径点序列

#### 后端 (Back-end): 轨迹优化
- 使用**Bezier曲线**表示轨迹
- 在**安全走廊**内优化平滑性
- 输出: 连续可微轨迹

### 3.3 关键概念

#### 3.3.1 安全走廊 (Safety Corridor)
```
障碍物 ████████████████
      █                █
      █  ←──走廊──→   █
      █                █
障碍物 ████████████████
```

- 由一系列无碰撞立方体组成
- 约束轨迹优化在安全区域内
- 平衡安全性和轨迹质量

#### 3.3.2 动力学约束
- **速度约束**: `|v| ≤ vₘₐₓ` (电机转速限制)
- **加速度约束**: `|a| ≤ aₘₐₓ` (推力限制)
- **连续性约束**: 轨迹各阶导数连续

#### 3.3.3 平滑性优化
最小化轨迹的"急动度" (jerk = 加速度的导数)：
```
minimize ∫ ||d³r/dt³||² dt
```
使轨迹更适合实际飞行。

---

## 4. 数学基础 {#math-foundations}

### 4.1 Eigen线性代数库

#### 4.1.1 基本数据类型
```cpp
Vector3d pt;           // 3D向量 [x, y, z]
MatrixXd coeff;        // 动态大小矩阵
VectorXd time;         // 动态大小向量
```

#### 4.1.2 常用操作
```cpp
Vector3d a(1, 2, 3);
Vector3d b(4, 5, 6);

double dot = a.dot(b);      // 点积: 1*4 + 2*5 + 3*6 = 32
double norm = a.norm();     // 模长: sqrt(1² + 2² + 3²) = 3.74
Vector3d c = a + b;         // 向量加法: [5, 7, 9]
```

### 4.2 Bezier曲线数学

#### 4.2.1 Bernstein基函数
n阶Bezier曲线定义为：
```
B(t) = Σᵢ₌₀ⁿ Pᵢ * Bᵢⁿ(t)
```

其中 `Bᵢⁿ(t) = C(n,i) * tⁱ * (1-t)ⁿ⁻ⁱ` 是Bernstein基函数。

#### 4.2.2 控制点和形状
- **控制点** `Pᵢ`: 影响曲线形状的参数
- **权重** `Bᵢⁿ(t)`: 控制各控制点的影响程度
- **性质**:
  - 曲线通过首末控制点
  - 曲线在控制点构成的凸包内
  - 易于计算导数 (速度、加速度)

### 4.3 数值优化

#### 4.3.1 二次规划 (QP)
轨迹优化问题表述为：
```
minimize   x^T Q x + c^T x
subject to Ax ≤ b         (不等式约束)
           Aeq x = beq    (等式约束)
```

其中:
- `x`: 优化变量 (Bezier控制点)
- `Q`: 成本矩阵 (平滑性)
- `A`: 约束矩阵 (安全走廊、动力学限制)

#### 4.3.2 MOSEK求解器
- 商业二次规划求解器
- 高效处理大规模优化问题
- 需要学术许可证

---

## 5. 算法原理 {#algorithms}

### 5.1 Fast Marching Method

#### 5.1.1 基本思想
- 模拟波的传播过程
- 在速度场中计算到达时间
- 生成最时间最优路径

#### 5.1.2 速度场设计
```cpp
double velMapping(double d, double max_v) {
    // d: 到最近障碍物的距离
    // 距离越近，允许速度越小
    if (d <= 0.25)      return 2.0 * d * d;
    else if (d <= 0.75) return 1.5 * d - 0.25;
    else               return max_v;
}
```

#### 5.1.3 路径提取
使用梯度下降法沿时间梯度提取最优路径。

### 5.2 A*搜索算法

#### 5.2.1 评估函数
```
f(n) = g(n) + h(n)
```
- `g(n)`: 从起点到当前点的实际代价
- `h(n)`: 从当前点到目标的启发式估计
- `f(n)`: 总评估值

#### 5.2.2 搜索策略
1. 优先扩展`f(n)`值最小的节点
2. 保证找到最优解 (如果启发式函数可行)
3. 在网格地图上搜索

### 5.3 立方体膨胀算法

#### 5.3.1 目的
为路径上每个点生成最大无碰撞立方体，形成安全走廊。

#### 5.3.2 膨胀策略
```
初始立方体 → 六个方向膨胀 → 碰撞检测 → 更新边界
     ↓
重复直到无法膨胀或达到最大迭代次数
```

#### 5.3.3 碰撞检测
遍历立方体表面的所有体素，检查是否为障碍物。

---

## 6. 代码架构解析 {#code-architecture}

### 6.1 系统状态管理

#### 6.1.1 状态标志
```cpp
bool _has_odom   = false;    // 是否接收到里程计
bool _has_map    = false;    // 是否接收到地图
bool _has_target = false;    // 是否接收到目标点
bool _has_traj   = false;    // 是否存在有效轨迹
bool _is_emerg   = false;    // 是否紧急状态
```

#### 6.1.2 状态转换
```
初始状态 → 接收传感器数据 → 接收目标点 → 轨迹规划 → 轨迹执行
   ↓              ↓              ↓           ↓          ↓
_is_init     _has_odom     _has_target   _has_traj   正常飞行
           _has_map                                      ↓
                                                   碰撞检测
                                                       ↓
                                                   重新规划
```

### 6.2 数据流

#### 6.2.1 输入数据流
```
点云传感器 → rcvPointCloudCallBack → 地图更新 → 碰撞检测
里程计    → rcvOdometryCallbck    → 状态更新 → TF发布
用户输入  → rcvWaypointsCallback  → 目标设置 → 轨迹规划
```

#### 6.2.2 输出数据流
```
轨迹规划 → getBezierTraj → 轨迹消息 → 控制器
可视化   → vis*函数      → RViz标记 → 图形界面
```

### 6.3 内存管理

#### 6.3.1 动态分配对象
```cpp
CollisionMapGrid* collision_map = new CollisionMapGrid();
```
- 全局地图：程序运行期间持续存在
- 局部地图：根据机器人位置动态更新

#### 6.3.2 临时对象
```cpp
vector<Vector3d> path_coord;    // 自动管理内存
MatrixXd _bezier_coeff;         // Eigen自动管理
```

### 6.4 时间管理

#### 6.4.1 轨迹时间参数化
```cpp
VectorXd _seg_time;     // 各段轨迹的持续时间
ros::Time _start_time;  // 轨迹开始执行时间
```

#### 6.4.2 碰撞预测
```cpp
double _check_horizon = 10.0;  // 前瞻10秒
double _stop_horizon  = 5.0;   // 5秒内碰撞触发紧急停止
```

---

## 总结

BTraj系统是一个典型的现代机器人系统，集成了：

1. **系统架构**: ROS分布式框架
2. **数学工具**: Eigen线性代数库
3. **算法理论**: 图搜索 + 数值优化
4. **工程实践**: 内存管理 + 异常处理

理解这些基础知识后，阅读和修改`b_traj_node.cpp`代码将更加容易。建议按以下顺序学习：

1. 熟悉C++基础语法和指针概念
2. 了解ROS通信机制和消息类型
3. 学习Eigen库的基本用法
4. 理解轨迹规划的数学原理
5. 分析代码的数据流和控制流

如有具体问题，可参考代码中的详细注释或查阅相关文档。