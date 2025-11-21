# BTraj 参数速查手册

> 快速查找ROS参数服务器参数名、C++变量名、默认值及含义的完整对照表

**文件位置:**
- Launch配置: [`catkin_ws/src/Btraj/launch/simulation.launch`](../catkin_ws/src/Btraj/launch/simulation.launch)
- 参数读取: [`catkin_ws/src/Btraj/src/b_traj_node.cpp:1207-1237`](../catkin_ws/src/Btraj/src/b_traj_node.cpp#L1207)

---

## 1. 地图相关参数 (map/)

| ROS参数名 | C++变量名 | Launch值 | 代码默认值 | 单位 | 含义 |
|-----------|-----------|----------|------------|------|------|
| `map/margin` | `_cloud_margin` | 0.2 | 0.25 | m | 障碍物安全边距（点云膨胀半径） |
| `map/resolution` | `_resolution` | *(未设置)* | 0.2 | m | 栅格地图分辨率（每个栅格的实际尺寸） |
| `map/x_size` | `_x_size` | 50.0 | 50.0 | m | 全局地图X轴尺寸（东西方向） |
| `map/y_size` | `_y_size` | 50.0 | 50.0 | m | 全局地图Y轴尺寸（南北方向） |
| `map/z_size` | `_z_size` | 5.0 | 5.0 | m | 全局地图Z轴尺寸（高度方向） |
| `map/x_local_size` | `_x_local_size` | 16.0 | 20.0 | m | 局部规划窗口X尺寸（滑动窗口） |
| `map/y_local_size` | `_y_local_size` | 16.0 | 20.0 | m | 局部规划窗口Y尺寸（滑动窗口） |
| `map/z_local_size` | `_z_local_size` | 8.0 | 5.0 | m | 局部规划窗口Z尺寸（滑动窗口） |

**代码位置:**
```cpp
// b_traj_node.cpp:1207-1216
nh.param("map/margin", _cloud_margin, 0.25);
nh.param("map/resolution", _resolution, 0.2);
nh.param("map/x_size", _x_size, 50.0);
nh.param("map/y_size", _y_size, 50.0);
nh.param("map/z_size", _z_size, 5.0);
nh.param("map/x_local_size", _x_local_size, 20.0);
nh.param("map/y_local_size", _y_local_size, 20.0);
nh.param("map/z_local_size", _z_local_size, 5.0);
```

---

## 2. 规划相关参数 (planning/)

| ROS参数名 | C++变量名 | Launch值 | 代码默认值 | 单位 | 含义 |
|-----------|-----------|----------|------------|------|------|
| `planning/init_x` | `_init_x` | -20.0 | 0.0 | m | 无人机初始X坐标 |
| `planning/init_y` | `_init_y` | -20.0 | 0.0 | m | 无人机初始Y坐标 |
| `planning/init_z` | `_init_z` | 0.5 | 0.0 | m | 无人机初始高度 |
| `planning/max_vel` | `_MAX_Vel` | 2.0 | 1.0 | m/s | **最大速度限制** |
| `planning/max_acc` | `_MAX_Acc` | 2.0 | 1.0 | m/s² | **最大加速度限制** |
| `planning/max_inflate` | `_max_inflate_iter` | *(未设置)* | 100 | - | 飞行走廊膨胀最大迭代次数 |
| `planning/inflate_iter` | `_max_inflate_iter` | 200 | 100 | - | 飞行走廊膨胀最大迭代次数 |
| `planning/step_length` | `_step_length` | 1 | 2 | 栅格 | 走廊膨胀扫描步长（影响计算速度） |
| `planning/cube_margin` | `_cube_margin` | 0.0 | 0.2 | m | 飞行走廊额外安全边距 |
| `planning/check_horizon` | `_check_horizon` | 10.0 | 10.0 | m | 轨迹碰撞检测前瞻距离 |
| `planning/stop_horizon` | `_stop_horizon` | 3.0 | 5.0 | m | 紧急停止判断距离（检测前方此距离内障碍物） |
| `planning/is_limit_vel` | `_is_limit_vel` | true | false | - | 优化时是否启用速度约束 |
| `planning/is_limit_acc` | `_is_limit_acc` | false | false | - | 优化时是否启用加速度约束 |
| `planning/is_use_fm` | `_is_use_fm` | **true** | true | - | **算法选择: true=Fast Marching*, false=A*** |

**代码位置:**
```cpp
// b_traj_node.cpp:1218-1231
nh.param("planning/init_x", _init_x, 0.0);
nh.param("planning/init_y", _init_y, 0.0);
nh.param("planning/init_z", _init_z, 0.0);
nh.param("planning/max_vel", _MAX_Vel, 1.0);
nh.param("planning/max_acc", _MAX_Acc, 1.0);
nh.param("planning/max_inflate", _max_inflate_iter, 100);
nh.param("planning/step_length", _step_length, 2);
nh.param("planning/cube_margin", _cube_margin, 0.2);
nh.param("planning/check_horizon", _check_horizon, 10.0);
nh.param("planning/stop_horizon", _stop_horizon, 5.0);
nh.param("planning/is_limit_vel", _is_limit_vel, false);
nh.param("planning/is_limit_acc", _is_limit_acc, false);
nh.param("planning/is_use_fm", _is_use_fm, true);
```

---

## 3. 优化相关参数 (optimization/)

| ROS参数名 | C++变量名 | Launch值 | 代码默认值 | 单位 | 含义 |
|-----------|-----------|----------|------------|------|------|
| `optimization/poly_order` | `_traj_order` | 8 | 10 | - | 贝塞尔多项式阶数（影响轨迹平滑度，8阶为论文推荐值） |
| `optimization/min_order` | `_minimize_order` | 2.5 | 3.0 | - | 优化目标的导数阶数<br>• 1=最小化速度<br>• 2=最小化加速度<br>• 3=最小化急动度(jerk)<br>• 4=最小化snap<br>• 2.5=50%加速度+50%急动度 |

**代码位置:**
```cpp
// b_traj_node.cpp:1233-1234
nh.param("optimization/min_order", _minimize_order, 3.0);
nh.param("optimization/poly_order", _traj_order, 10);
```

**使用说明:**
- `min_order` 小数值含义：线性插值不同阶导数的优化目标
  - 例如 2.5 = 0.5×(最小化加速度) + 0.5×(最小化急动度)
  - 平衡轨迹平滑性与执行效率

---

## 4. 可视化参数 (vis/)

| ROS参数名 | C++变量名 | Launch值 | 代码默认值 | 单位 | 含义 |
|-----------|-----------|----------|------------|------|------|
| `vis/vis_traj_width` | `_vis_traj_width` | 0.15 | 0.15 | m | RViz中轨迹线的显示宽度 |
| `vis/is_proj_cube` | `_is_proj_cube` | false | true | - | 是否将飞行走廊投影到地面显示（方便观察平面路径） |

**代码位置:**
```cpp
// b_traj_node.cpp:1236-1237
nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
nh.param("vis/is_proj_cube", _is_proj_cube, true);
```

---

## 5. 其他节点参数

### 5.1 轨迹服务器 (b_traj_server)

| ROS参数名 | Launch值 | 含义 |
|-----------|----------|------|
| `optimization/poly_order_min` | 3 | 轨迹执行器支持的最小多项式阶数 |
| `optimization/poly_order_max` | 12 | 轨迹执行器支持的最大多项式阶数 |

**说明:** 当前项目规划器固定输出8阶，服务器实际处理8阶。设计目的是系统扩展性，支持未来不同规划器(3-12阶)。

### 5.2 传感器仿真 (random_forest_sensing)

| ROS参数名 | Launch值 | 单位 | 含义 |
|-----------|----------|------|------|
| `map/obs_num` | 520 | - | 障碍物（树木）数量 |
| `map/resolution` | 0.2 | m | 地图分辨率 |
| `ObstacleShape/lower_rad` | 0.3 | m | 树干最小半径 |
| `ObstacleShape/upper_rad` | 1.6 | m | 树干最大半径 |
| `ObstacleShape/lower_hei` | 1.0 | m | 树木最小高度 |
| `ObstacleShape/upper_hei` | 6.0 | m | 树木最大高度 |
| `sensing/radius` | 15.0 | m | 传感器检测半径（模拟激光雷达范围） |
| `sensing/rate` | 10.0 | Hz | 传感器更新频率 |

### 5.3 里程计生成器 (odom_generator)

| ROS参数名 | 继承自 | 含义 |
|-----------|--------|------|
| `init_x` | `$(arg init_x)` = -20.0 | 初始X坐标 |
| `init_y` | `$(arg init_y)` = -20.0 | 初始Y坐标 |
| `init_z` | `$(arg init_z)` = 0.5 | 初始Z坐标 |

---

## 6. 常用参数调优指南

### 6.1 算法选择

**切换路径规划算法:**
```xml
<param name="planning/is_use_fm" value="true"/>   <!-- Fast Marching* -->
<param name="planning/is_use_fm" value="false"/>  <!-- A* -->
```

| 算法 | 优点 | 缺点 | 适用场景 |
|------|------|------|----------|
| **Fast Marching*** | 路径更平滑，考虑速度场 | 计算稍慢 | 复杂障碍环境、需要平滑路径 |
| **A*** | 计算快速 | 路径较为离散 | 简单环境、快速响应 |

### 6.2 速度/加速度调优

**提高飞行速度:**
```xml
<param name="planning/max_vel" value="3.0"/>  <!-- 默认2.0 m/s -->
<param name="planning/max_acc" value="3.0"/>  <!-- 默认2.0 m/s² -->
```

**注意事项:**
- 速度过高可能导致碰撞检测不及时
- 需要同时提高加速度以保证动力学可行性
- 实际使用速度为 `max_vel * 0.6`（代码中硬编码系数）

### 6.3 安全距离调优

**增加安全距离:**
```xml
<param name="map/margin" value="0.3"/>         <!-- 障碍物膨胀半径，默认0.2m -->
<param name="planning/cube_margin" value="0.1"/> <!-- 走廊额外边距，默认0.0m -->
```

**效果:**
- `map/margin` 越大，障碍物周围安全区域越大
- `cube_margin` 越大，飞行走廊越保守（距离障碍物更远）

### 6.4 轨迹平滑度调优

**更平滑的轨迹:**
```xml
<param name="optimization/poly_order" value="10"/>  <!-- 提高阶数，默认8 -->
<param name="optimization/min_order" value="3.5"/>  <!-- 优化更高阶导数，默认2.5 -->
```

**权衡:**
- 阶数越高，轨迹越平滑，但计算量越大
- `min_order` 越高，轨迹越平滑但可能速度较慢

### 6.5 膨胀参数调优

**加快计算速度（降低精度）:**
```xml
<param name="planning/inflate_iter" value="100"/>  <!-- 减少迭代次数，默认200 -->
<param name="planning/step_length" value="2"/>     <!-- 增大步长，默认1 -->
```

**提高走廊质量（增加计算时间）:**
```xml
<param name="planning/inflate_iter" value="500"/>  <!-- 增加迭代次数 -->
<param name="planning/step_length" value="1"/>     <!-- 减小步长 -->
```

---

## 7. 全局变量声明位置

在 `b_traj_node.cpp` 中的全局变量声明（第42-51行）:

```cpp
// simulation param from launch file
double _vis_traj_width;
double _resolution, _inv_resolution;
double _cloud_margin, _cube_margin, _check_horizon, _stop_horizon;
double _x_size, _y_size, _z_size, _x_local_size, _y_local_size, _z_local_size;
double _MAX_Vel, _MAX_Acc;
bool _is_use_fm, _is_proj_cube, _is_limit_vel, _is_limit_acc;
int _step_length, _max_inflate_iter, _traj_order;
double _minimize_order;
```

---

## 8. 快速查找索引

### 按功能分类

**性能相关:**
- 速度/加速度: `planning/max_vel`, `planning/max_acc`
- 计算速度: `planning/inflate_iter`, `planning/step_length`
- 轨迹阶数: `optimization/poly_order`

**安全相关:**
- 障碍物边距: `map/margin`
- 走廊边距: `planning/cube_margin`
- 碰撞检测: `planning/check_horizon`, `planning/stop_horizon`

**算法选择:**
- 路径算法: `planning/is_use_fm`
- 优化约束: `planning/is_limit_vel`, `planning/is_limit_acc`

**地图相关:**
- 全局地图: `map/x_size`, `map/y_size`, `map/z_size`
- 局部窗口: `map/x_local_size`, `map/y_local_size`, `map/z_local_size`
- 分辨率: `map/resolution`

---

## 附录: 参数修改方法

### A. 修改launch文件（推荐）
直接编辑 `simulation.launch` 文件：
```xml
<param name="planning/max_vel" value="3.0"/>
```
无需重新编译，下次启动生效。

### B. 命令行覆盖
```bash
roslaunch bezier_planer simulation.launch planning/max_vel:=3.0
```

### C. 运行时修改（临时）
```bash
rosparam set /b_traj_node/planning/max_vel 3.0
```
仅当前会话有效，重启失效。

---

**文档版本:** v1.0
**最后更新:** 2025-11-21
**维护者:** BTraj 项目组
