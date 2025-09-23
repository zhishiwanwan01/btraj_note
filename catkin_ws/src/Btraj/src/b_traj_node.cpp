// ============================================
// 标准C++库头文件
// ============================================
#include <iostream>     // 标准输入输出流，用于打印调试信息
#include <fstream>      // 文件流操作，可能用于读写配置文件
#include <math.h>       // 数学函数库，提供sin、cos、sqrt等基础数学函数
#include <random>       // 随机数生成器，用于仿真中的随机化

// ============================================
// 第三方数学库
// ============================================
#include <eigen3/Eigen/Dense>  // Eigen线性代数库，用于矩阵、向量运算（UAV轨迹规划的核心数学工具）

// ============================================
// PCL点云处理库
// ============================================
#include <pcl_conversions/pcl_conversions.h>  // ROS与PCL之间的消息转换
#include <pcl/point_cloud.h>                  // 点云数据结构，用于表示3D环境中的障碍物
#include <pcl/point_types.h>                  // 点云中点的类型定义（如XYZ坐标点）
#include <pcl/search/kdtree.h>                // KD树搜索结构，用于高效的最近邻搜索

// ============================================
// ROS核心组件
// ============================================
#include <ros/ros.h>              // ROS核心功能：节点初始化、参数服务器、时间管理
#include <ros/console.h>          // ROS日志系统：ROS_INFO、ROS_WARN、ROS_ERROR等

// ============================================
// ROS标准消息类型
// ============================================
#include <sensor_msgs/PointCloud2.h>        // 点云数据消息，用于接收环境感知信息
#include <nav_msgs/Odometry.h>               // 里程计消息，包含位置、速度、加速度信息
#include <nav_msgs/Path.h>                   // 路径消息，用于接收目标点
#include <geometry_msgs/PoseStamped.h>       // 带时间戳的位姿消息
#include <visualization_msgs/MarkerArray.h>  // 可视化标记数组，用于RViz显示
#include <visualization_msgs/Marker.h>       // 单个可视化标记

// ============================================
// TF坐标变换系统
// ============================================
#include <tf/tf.h>                     // TF变换基础功能
#include <tf/transform_datatypes.h>    // 变换数据类型（四元数、欧拉角等）
#include <tf/transform_broadcaster.h>  // 坐标变换广播器，发布坐标系关系

// ============================================
// 项目自定义头文件
// ============================================
#include "trajectory_generator.h"  // 轨迹生成器：核心优化算法，使用Bezier曲线生成平滑轨迹
#include "bezier_base.h"           // Bernstein基函数：Bezier曲线的数学基础
#include "data_type.h"             // 自定义数据类型（如Cube立方体结构）
#include "utils.h"                 // 工具函数集合
#include "a_star.h"                // A*路径搜索算法实现
#include "backward.hpp"            // 异常处理和堆栈回溯工具

// ============================================
// 四旋翼专用消息类型
// ============================================
#include "quadrotor_msgs/PositionCommand.h"     // 位置控制指令
#include "quadrotor_msgs/PolynomialTrajectory.h" // 多项式轨迹表示（Bezier系数编码）

// ============================================
// 命名空间声明 - 简化代码书写
// ============================================
using namespace std;       // 标准库命名空间（vector、string等）
using namespace Eigen;     // Eigen库命名空间（Vector3d、MatrixXd等）
using namespace sdf_tools; // SDF（Signed Distance Field）工具库

// ============================================
// 异常处理配置
// ============================================
namespace backward {
    backward::SignalHandling sh;  // 自动堆栈回溯，程序崩溃时提供详细错误信息
}

// ============================================
// 全局配置参数（从ROS launch文件读取）
// ============================================

// 可视化参数
double _vis_traj_width;          // 轨迹可视化线条宽度（RViz显示）

// 地图分辨率参数
double _resolution;              // 栅格地图分辨率（米/格子）- 影响规划精度
double _inv_resolution;          // 分辨率倒数（格子/米）- 用于坐标转换优化

// 安全边界参数
double _cloud_margin;            // 点云膨胀边界（米）- 为UAV添加安全距离
double _cube_margin;             // 立方体走廊边界（米）- 轨迹优化时的安全裕度
double _check_horizon;           // 碰撞检测前瞻距离（秒）- 预测轨迹碰撞
double _stop_horizon;            // 紧急停止判断距离（秒）- 触发紧急避障

// 地图尺寸参数
double _x_size, _y_size, _z_size;                    // 全局地图尺寸（米）
double _x_local_size, _y_local_size, _z_local_size;  // 局部地图尺寸（米）- 提高计算效率

// 动力学约束参数
double _MAX_Vel;                 // 最大速度约束（米/秒）- 确保UAV动力学可行性
double _MAX_Acc;                 // 最大加速度约束（米/秒²）- 确保电机功率足够

// 算法选择开关
bool _is_use_fm;                 // 是否使用Fast Marching算法（true）还是A*算法（false）
bool _is_proj_cube;              // 是否投影立方体到2D平面显示
bool _is_limit_vel;              // 是否启用速度约束
bool _is_limit_acc;              // 是否启用加速度约束

// 算法内部参数
int _step_length;                // 立方体膨胀步长（栅格单位）
int _max_inflate_iter;           // 立方体膨胀最大迭代次数
int _traj_order;                 // Bezier曲线阶数（通常8-12阶，越高越光滑）
double _minimize_order;          // 优化目标阶数（snap最小化通常用4阶导数）

// ============================================
// 系统状态变量
// ============================================

// 传感器数据
nav_msgs::Odometry _odom;        // 当前里程计信息（位置、速度、加速度）

// 系统状态标志 - 用于节点间通信同步
bool _has_odom   = false;        // 是否接收到里程计数据
bool _has_map    = false;        // 是否接收到环境地图数据
bool _has_target = false;        // 是否接收到目标点
bool _has_traj   = false;        // 是否存在有效轨迹
bool _is_emerg   = false;        // 是否处于紧急状态（碰撞风险）
bool _is_init    = true;         // 是否为初始状态（未接收第一个目标）

// ============================================
// 轨迹规划状态变量
// ============================================

// 起点和终点状态
Vector3d _start_pt;              // 起始位置 [x,y,z]（米）
Vector3d _start_vel;             // 起始速度 [vx,vy,vz]（米/秒）
Vector3d _start_acc;             // 起始加速度 [ax,ay,az]（米/秒²）
Vector3d _end_pt;                // 目标位置 [x,y,z]（米）

// 初始化位置
double _init_x, _init_y, _init_z; // 系统初始位置（从参数服务器读取）

// ============================================
// 地图坐标系参数
// ============================================
Vector3d _map_origin;            // 地图原点坐标（世界坐标系）

// 地图边界限制（世界坐标系）
double _pt_max_x, _pt_min_x;     // X轴边界（米）
double _pt_max_y, _pt_min_y;     // Y轴边界（米）
double _pt_max_z, _pt_min_z;     // Z轴边界（米）

// 栅格地图索引边界
double _max_x_id, _max_y_id, _max_z_id;                        // 全局地图最大索引
int _max_local_x_id, _max_local_y_id, _max_local_z_id;         // 局部地图最大索引

// ============================================
// 轨迹管理参数
// ============================================
int _traj_id = 1;                // 轨迹ID计数器，用于跟踪轨迹版本

// ============================================
// 碰撞检测单元格定义
// ============================================
COLLISION_CELL _free_cell(0.0);  // 自由空间单元格（占据概率0.0）
COLLISION_CELL _obst_cell(1.0);  // 障碍物单元格（占据概率1.0）
// ============================================
// ROS发布者和订阅者
// ============================================

// 订阅者 - 接收外部数据
ros::Subscriber _map_sub;        // 订阅点云地图数据
ros::Subscriber _pts_sub;        // 订阅目标点数据
ros::Subscriber _odom_sub;       // 订阅里程计数据

// 发布者 - 发送结果数据
ros::Publisher _fm_path_vis_pub;     // 发布Fast Marching路径可视化
ros::Publisher _local_map_vis_pub;   // 发布局部地图可视化
ros::Publisher _inf_map_vis_pub;     // 发布膨胀后地图可视化
ros::Publisher _corridor_vis_pub;    // 发布轨迹走廊可视化
ros::Publisher _traj_vis_pub;        // 发布最终轨迹可视化
ros::Publisher _grid_path_vis_pub;   // 发布A*网格路径可视化
ros::Publisher _nodes_vis_pub;       // 发布A*搜索节点可视化
ros::Publisher _traj_pub;            // 发布轨迹指令（给控制器）
ros::Publisher _checkTraj_vis_pub;   // 发布碰撞检测轨迹可视化
ros::Publisher _stopTraj_vis_pub;    // 发布紧急停止轨迹可视化

// ============================================
// 轨迹表示参数 - 存储生成的轨迹
// ============================================
int _seg_num;                    // 轨迹段数（分段规划，每段为一个Bezier曲线）
VectorXd _seg_time;              // 各段轨迹持续时间（秒）
MatrixXd _bezier_coeff;          // Bezier系数矩阵（每行为一段轨迹的控制点）

// ============================================
// Bezier曲线数学基础矩阵 - 用于轨迹优化计算
// ============================================
// 这些矩阵由Bernstein类预计算，避免重复计算提高效率
MatrixXd _MQM;                   // 最小二乘成本矩阵（用于平滑性优化）
MatrixXd _FM;                    // 映射矩阵（用于约束条件）
VectorXd _C;                     // 位置系数（Bernstein基系数）
VectorXd _Cv;                    // 速度系数（1阶导数系数）
VectorXd _Ca;                    // 加速度系数（2阶导数系数）
VectorXd _Cj;                    // Jerk系数（3阶导数系数，加速度变化率）

// ============================================
// 核心功能对象
// ============================================

// ROS消息对象
quadrotor_msgs::PolynomialTrajectory _traj;  // 当前有效轨迹消息
ros::Time _start_time = ros::TIME_MAX;       // 轨迹开始执行时间

// 算法核心类
TrajectoryGenerator _trajectoryGenerator;    // 轨迹优化器（使用MOSEK求解器）

// 地图管理对象 - 使用指针以支持动态内存分配
CollisionMapGrid* collision_map       = new CollisionMapGrid();  // 全局碰撞检测地图
CollisionMapGrid* collision_map_local = new CollisionMapGrid();  // 局部碰撞检测地图（提高效率）
gridPathFinder* path_finder           = new gridPathFinder();    // A*路径搜索器

// ============================================
// ROS回调函数声明 - 处理外部数据输入
// ============================================
void rcvWaypointsCallback(const nav_msgs::Path& wp);               // 接收目标点，触发轨迹规划
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map);  // 接收点云地图，更新碰撞检测
void rcvOdometryCallbck(const nav_msgs::Odometry odom);            // 接收里程计，更新机器人状态

// ============================================
// 轨迹规划核心函数
// ============================================
void trajPlanning();                                              // 主轨迹规划函数：整合路径搜索+轨迹优化
bool checkExecTraj();                                             // 检查当前轨迹安全性，预测碰撞
bool checkCoordObs(Vector3d checkPt);                            // 检查单点是否与障碍物碰撞
vector<pcl::PointXYZ> pointInflate(pcl::PointXYZ pt);            // 点云膨胀：为UAV添加安全边界

// ============================================
// 可视化函数 - RViz显示用
// ============================================
void visPath(vector<Vector3d> path);                             // 显示原始路径（Fast Marching/A*结果）
void visCorridor(vector<Cube> corridor);                         // 显示走廊立方体（轨迹约束区域）
void visGridPath(vector<Vector3d> grid_path);                    // 显示A*网格路径
void visExpNode(vector<GridNodePtr> nodes);                      // 显示A*搜索过的节点
void visBezierTrajectory(MatrixXd polyCoeff, VectorXd time);     // 显示最终Bezier轨迹

// ============================================
// 走廊生成相关函数
// ============================================
// 走廊：无碰撞的立方体区域，用于约束轨迹优化
pair<Cube, bool> inflateCube(Cube cube, Cube lstcube);           // 立方体膨胀：扩大至最大无碰撞尺寸
Cube generateCube(Vector3d pt);                                  // 生成初始单位立方体
bool isContains(Cube cube1, Cube cube2);                         // 判断立方体包含关系
void corridorSimplify(vector<Cube>& cubicList);                  // 简化走廊：移除冗余立方体
vector<Cube> corridorGeneration(vector<Vector3d> path_coord, vector<double> time);  // 生成走廊（带时间）
vector<Cube> corridorGeneration(vector<Vector3d> path_coord);    // 生成走廊（不带时间）

// ============================================
// 时间分配相关函数
// ============================================
void sortPath(vector<Vector3d>& path_coord, vector<double>& time);     // 路径排序和清理
void timeAllocation(vector<Cube>& corridor, vector<double> time);      // 时间分配（带初始时间）
void timeAllocation(vector<Cube>& corridor);                           // 时间分配（自动计算）

// ============================================
// Bezier轨迹计算函数
// ============================================
VectorXd getStateFromBezier(const MatrixXd& polyCoeff, double t_now, int seg_now);  // 获取全状态（位置+速度+加速度+Jerk）
Vector3d getPosFromBezier(const MatrixXd& polyCoeff, double t_now, int seg_now);    // 获取位置状态
quadrotor_msgs::PolynomialTrajectory getBezierTraj();                              // 封装轨迹为ROS消息

/**
 * @brief 里程计数据回调函数
 *
 * 功能说明：
 * 1. 接收机器人当前位置、速度、加速度信息
 * 2. 更新轨迹规划的起始状态
 * 3. 发布TF坐标变换（用于RViz显示）
 *
 * 注意：这里将加速度信息存储在angular字段中，这是一个特殊的设计
 */
void rcvOdometryCallbck(const nav_msgs::Odometry odom)
{
    // 数据有效性检查：只处理来自uav框架的数据
    if (odom.header.frame_id != "uav")
        return;

    // 保存里程计数据并标记为已接收
    _odom = odom;
    _has_odom = true;

    // 提取当前位置信息（世界坐标系）
    _start_pt(0) = _odom.pose.pose.position.x;  // X坐标（米）
    _start_pt(1) = _odom.pose.pose.position.y;  // Y坐标（米）
    _start_pt(2) = _odom.pose.pose.position.z;  // Z坐标（米）

    // 提取当前速度信息（世界坐标系）
    _start_vel(0) = _odom.twist.twist.linear.x;  // X方向速度（米/秒）
    _start_vel(1) = _odom.twist.twist.linear.y;  // Y方向速度（米/秒）
    _start_vel(2) = _odom.twist.twist.linear.z;  // Z方向速度（米/秒）

    // 提取当前加速度信息（特殊：存储在angular字段）
    _start_acc(0) = _odom.twist.twist.angular.x;  // X方向加速度（米/秒²）
    _start_acc(1) = _odom.twist.twist.angular.y;  // Y方向加速度（米/秒²）
    _start_acc(2) = _odom.twist.twist.angular.z;  // Z方向加速度（米/秒²）

    // NaN值检查：防止无效数据干扰规划
    if (std::isnan(_odom.pose.pose.position.x) || std::isnan(_odom.pose.pose.position.y) || std::isnan(_odom.pose.pose.position.z))
        return;

    // 发布TF坐标变换：从"world"到"quadrotor"框架
    // 用于RViz中显示机器人当前位置
    static tf::TransformBroadcaster br;  // 静态变量，避免重复初始化
    tf::Transform transform;

    // 设置位置信息
    transform.setOrigin(tf::Vector3(_odom.pose.pose.position.x,
                                    _odom.pose.pose.position.y,
                                    _odom.pose.pose.position.z));

    // 设置旋转信息（这里使用单位四元数，即无旋转）
    transform.setRotation(tf::Quaternion(0, 0, 0, 1.0));

    // 广播坐标变换
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "quadrotor"));
}

/**
 * @brief 目标点接收回调函数
 *
 * 功能说明：
 * 1. 接收来自用户（通过RViz 3D Nav Goal工具）的目标点
 * 2. 触发轨迹重新规划
 * 3. 设置紧急状态，确保当前轨迹被重新规划
 */
void rcvWaypointsCallback(const nav_msgs::Path& wp)
{
    // 有效性检查：Z坐标不能为负值（地面以下）
    if (wp.poses[0].pose.position.z < 0.0)
        return;

    // 标记系统已经不再是初始状态
    _is_init = false;

    // 提取目标位置（只使用第一个点，单点导航）
    _end_pt << wp.poses[0].pose.position.x,
               wp.poses[0].pose.position.y,
               wp.poses[0].pose.position.z;

    // 更新系统状态标志
    _has_target = true;   // 标记已有目标点
    _is_emerg   = true;   // 设置紧急状态，强制重新规划轨迹

    ROS_INFO("[Fast Marching Node] receive the way-points");

    // 立即触发轨迹规划
    trajPlanning();
}

Vector3d _local_origin;
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if((int)cloud.points.size() == 0)
        return;

    delete collision_map_local;

    ros::Time time_1 = ros::Time::now();
    collision_map->RestMap();
    
    double local_c_x = (int)((_start_pt(0) - _x_local_size/2.0)  * _inv_resolution + 0.5) * _resolution;
    double local_c_y = (int)((_start_pt(1) - _y_local_size/2.0)  * _inv_resolution + 0.5) * _resolution;
    double local_c_z = (int)((_start_pt(2) - _z_local_size/2.0)  * _inv_resolution + 0.5) * _resolution;

    _local_origin << local_c_x, local_c_y, local_c_z;

    Translation3d origin_local_translation( _local_origin(0), _local_origin(1), _local_origin(2));
    Quaterniond origin_local_rotation(1.0, 0.0, 0.0, 0.0);

    Affine3d origin_local_transform = origin_local_translation * origin_local_rotation;
    
    double _buffer_size = 2 * _MAX_Vel;
    double _x_buffer_size = _x_local_size + _buffer_size;
    double _y_buffer_size = _y_local_size + _buffer_size;
    double _z_buffer_size = _z_local_size + _buffer_size;

    collision_map_local = new CollisionMapGrid(origin_local_transform, "world", _resolution, _x_buffer_size, _y_buffer_size, _z_buffer_size, _free_cell);

    vector<pcl::PointXYZ> inflatePts(20);
    pcl::PointCloud<pcl::PointXYZ> cloud_inflation;
    pcl::PointCloud<pcl::PointXYZ> cloud_local;

    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {   
        auto mk = cloud.points[idx];
        pcl::PointXYZ pt(mk.x, mk.y, mk.z);

        if( fabs(pt.x - _start_pt(0)) > _x_local_size / 2.0 || fabs(pt.y - _start_pt(1)) > _y_local_size / 2.0 || fabs(pt.z - _start_pt(2)) > _z_local_size / 2.0 )
            continue; 
        
        // 将有效点加入局部点云
        cloud_local.push_back(pt);

        // 对每个障碍物点进行膨胀处理
        inflatePts = pointInflate(pt);
        for(int i = 0; i < (int)inflatePts.size(); i++)
        {
            pcl::PointXYZ inf_pt = inflatePts[i];
            Vector3d addPt(inf_pt.x, inf_pt.y, inf_pt.z);

            // 同时更新局部和全局碰撞地图
            collision_map_local->Set3d(addPt, _obst_cell);  // 标记为障碍物
            collision_map->Set3d(addPt, _obst_cell);        // 全局地图同步更新

            // 保存膨胀后的点用于可视化
            cloud_inflation.push_back(inf_pt);
        }
    }

    // 标记地图数据已准备就绪
    _has_map = true;

    // 设置膨胀点云的PCL属性
    cloud_inflation.width = cloud_inflation.points.size();  // 点云宽度（点的数量）
    cloud_inflation.height = 1;                            // 点云高度（无序点云为1）
    cloud_inflation.is_dense = true;                       // 所有点都是有效的（无NaN）
    cloud_inflation.header.frame_id = "world";              // 坐标系标识

    // 设置局部点云的PCL属性
    cloud_local.width = cloud_local.points.size();
    cloud_local.height = 1;
    cloud_local.is_dense = true;
    cloud_local.header.frame_id = "world";

    // 创建ROS点云消息用于可视化发布
    sensor_msgs::PointCloud2 inflateMap, localMap;

    // 将PCL点云转换为ROS消息格式
    pcl::toROSMsg(cloud_inflation, inflateMap);  // 膨胀后的障碍物点云
    pcl::toROSMsg(cloud_local, localMap);        // 原始局部点云

    // 发布可视化消息供RViz显示
    _inf_map_vis_pub.publish(inflateMap);        // 显示膨胀障碍物
    _local_map_vis_pub.publish(localMap);        // 显示原始点云

    ros::Time time_3 = ros::Time::now();
    // 可选：打印地图处理耗时信息
    //ROS_WARN("Time in receiving the map is %f", (time_3 - time_1).toSec());

    // 地图更新后立即检查当前轨迹的安全性
    if( checkExecTraj() == true )
        trajPlanning();  // 如果检测到潜在碰撞，立即重新规划轨迹 
}

/**
 * @brief 点云膨胀函数：为障碍物点添加安全边界
 *
 * 功能说明：
 * 1. 将单个障碍物点扩展为一个立方体区域
 * 2. 立方体大小由 _cloud_margin 参数决定
 * 3. Z方向膨胀范围减半（考虑UAV主要在水平面运动）
 *
 * @param pt 输入的障碍物点
 * @return 膨胀后的点集合
 */
vector<pcl::PointXYZ> pointInflate( pcl::PointXYZ pt)
{
    // 计算膨胀范围（网格单位）
    int num   = int(_cloud_margin * _inv_resolution);  // XY方向膨胀范围
    int num_z = max(1, num / 2);                       // Z方向膨胀范围（减半）

    vector<pcl::PointXYZ> infPts(20);  // 预分配空间避免频繁重分配
    pcl::PointXYZ pt_inf;

    // 三重循环生成立方体内的所有膨胀点
    for(int x = -num ; x <= num; x ++ )
        for(int y = -num ; y <= num; y ++ )
            for(int z = -num_z ; z <= num_z; z ++ )
            {
                // 计算膨胀点的世界坐标
                pt_inf.x = pt.x + x * _resolution;  // X方向偏移
                pt_inf.y = pt.y + y * _resolution;  // Y方向偏移
                pt_inf.z = pt.z + z * _resolution;  // Z方向偏移

                infPts.push_back( pt_inf );  // 添加到膨胀点集合
            }

    return infPts;
}

/**
 * @brief 轨迹安全检查函数：预测轨迹是否与障碍物碰撞
 *
 * 功能说明：
 * 1. 检查未来一段时间内的轨迹安全性
 * 2. 分两个时间窗口：_check_horizon(检测)和_stop_horizon(紧急停止)
 * 3. 在RViz中可视化检查结果
 *
 * @return true: 检测到碰撞风险，需要重新规划
 *         false: 轨迹安全，可以继续执行
 */
bool checkExecTraj()
{
    // 前置条件：必须存在有效轨迹
    if( _has_traj == false )
        return false;

    Vector3d traj_pt;  // 轨迹上的采样点

    // 创建可视化标记：用于在RViz中显示检查结果
    visualization_msgs::Marker _check_traj_vis, _stop_traj_vis;

    geometry_msgs::Point pt;  // 用于存储轨迹点坐标

    // 设置可视化消息的公共属性
    _stop_traj_vis.header.stamp    = _check_traj_vis.header.stamp    = ros::Time::now();
    _stop_traj_vis.header.frame_id = _check_traj_vis.header.frame_id = "world";

    // 设置命名空间（用于在RViz中组织显示）
    _check_traj_vis.ns = "trajectory/check_trajectory";  // 检查轨迹显示
    _stop_traj_vis.ns  = "trajectory/stop_trajectory";   // 紧急停止轨迹显示

    // 设置标记属性
    _stop_traj_vis.id     = _check_traj_vis.id = 0;      // 标记ID
    _stop_traj_vis.type   = _check_traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;  // 球列表类型
    _stop_traj_vis.action = _check_traj_vis.action = visualization_msgs::Marker::ADD;        // 添加操作

    // 设置球的尺寸（紧急停止轨迹的球更大，更显眼）
    _stop_traj_vis.scale.x = 2.0 * _vis_traj_width;   // 索急停止区域：较大的红球
    _stop_traj_vis.scale.y = 2.0 * _vis_traj_width;
    _stop_traj_vis.scale.z = 2.0 * _vis_traj_width;

    _check_traj_vis.scale.x = 1.5 * _vis_traj_width;  // 检查区域：稍小的蓝球
    _check_traj_vis.scale.y = 1.5 * _vis_traj_width;
    _check_traj_vis.scale.z = 1.5 * _vis_traj_width;

    // 设置旋转（使用单位四元数，即无旋转）
    _check_traj_vis.pose.orientation.x = 0.0;
    _check_traj_vis.pose.orientation.y = 0.0;
    _check_traj_vis.pose.orientation.z = 0.0;
    _check_traj_vis.pose.orientation.w = 1.0;

    _stop_traj_vis.pose = _check_traj_vis.pose;  // 复制旋转设置

    // 设置颜色：绿色表示紧急停止区域
    _stop_traj_vis.color.r = 0.0;  // 红色分量
    _stop_traj_vis.color.g = 1.0;  // 绿色分量（主色）
    _stop_traj_vis.color.b = 0.0;  // 蓝色分量
    _stop_traj_vis.color.a = 1.0;  // 透明度（不透明）

    // 设置颜色：蓝色表示检查区域
    _check_traj_vis.color.r = 0.0;  // 红色分量
    _check_traj_vis.color.g = 0.0;  // 绿色分量
    _check_traj_vis.color.b = 1.0;  // 蓝色分量（主色）
    _check_traj_vis.color.a = 1.0;  // 透明度（不透明）

    // ============================================
    // 计算当前轨迹执行状态
    // ============================================

    // 计算轨迹开始执行以来的时间
    double t_s = max(0.0, (_odom.header.stamp - _start_time).toSec());

    // 找到当前正在执行的轨迹段
    int idx;
    for (idx = 0; idx < _seg_num; ++idx)
    {
        // 如果当前段已经执行完成，移动到下一段
        if( t_s > _seg_time(idx) && idx + 1 < _seg_num)
            t_s -= _seg_time(idx);  // 减去已执行段的时间
        else
            break;  // 找到当前正在执行的段
    }

    // ============================================
    // 检查未来轨迹段的碰撞情况
    // ============================================

    double duration = 0.0;  // 累计检查时间
    double t_ss;            // 当前段的起始时间

    // 从当前段开始，检查后续所有轨迹段
    for(int i = idx; i < _seg_num; i++ )
    {
        // 计算当前段的起始检查时间
        t_ss = (i == idx) ? t_s : 0.0;  // 当前段从 t_s 开始，其他段从 0 开始

        // 在当前轨迹段上采样检查（每0.01秒采样一次）
        for(double t = t_ss; t < _seg_time(i); t += 0.01)
        {
            // 计算相对于当前时刻的预测时间
            double t_d = duration + t - t_ss;

            // 如果超出检查时间窗口，停止检查
            if( t_d > _check_horizon ) break;

            // 从 Bezier 轨迹获取当前时刻的位置
            traj_pt = getPosFromBezier( _bezier_coeff, t/_seg_time(i), i );

            // 将归一化坐标转换为世界坐标（乘以段时间）
            pt.x = traj_pt(0) = _seg_time(i) * traj_pt(0);  // X 坐标
            pt.y = traj_pt(1) = _seg_time(i) * traj_pt(1);  // Y 坐标
            pt.z = traj_pt(2) = _seg_time(i) * traj_pt(2);  // Z 坐标

            // 将检查点添加到可视化数组中
            _check_traj_vis.points.push_back(pt);

            // 如果在紧急停止时间窗口内，也添加到紧急可视化
            if( t_d <= _stop_horizon )
                _stop_traj_vis.points.push_back(pt);

            // 检查当前点是否与障碍物碰撞
            if( checkCoordObs(traj_pt))
            {
                ROS_WARN("predicted collision time is %f ahead", t_d);

                // 如果碰撞发生在紧急停止时间窗口内，设置紧急标志
                if( t_d <= _stop_horizon )
                {
                    ROS_ERROR("emergency occurs in time is %f ahead", t_d);
                    _is_emerg = true;  // 设置紧急状态标志
                }

                // 发布可视化消息显示碰撞点
                _checkTraj_vis_pub.publish(_check_traj_vis);
                _stopTraj_vis_pub.publish(_stop_traj_vis);

                return true;  // 返回 true 表示需要重新规划
            }
        }

        // 更新累计检查时间
        duration += _seg_time(i) - t_ss;
    }

    // 如果没有检测到碰撞，仍发布可视化消息显示安全轨迹
    _checkTraj_vis_pub.publish(_check_traj_vis);
    _stopTraj_vis_pub.publish(_stop_traj_vis);

    return false;  // 返回 false 表示轨迹安全，无需重新规划
}

/**
 * @brief 单点碰撞检测函数
 *
 * 功能说明：
 * 检查给定的3D点是否位于障碍物区域内
 * 通过查询全局碰撞地图的占据概率来判断
 *
 * @param checkPt 需要检查的3D点坐标 [x, y, z]
 * @return true: 点位于障碍物区域
 *         false: 点位于自由空间
 */
bool checkCoordObs(Vector3d checkPt)
{
    // 查询全局碰撞地图中该点的占据概率
    // 占据概率 > 0.0 表示该位置有障碍物
    if(collision_map->Get(checkPt(0), checkPt(1), checkPt(2)).first.occupancy > 0.0 )
        return true;   // 检测到碰撞

    return false;  // 无碰撞，安全
}

/**
 * @brief 立方体膨胀函数：将立方体扩大到最大可能的无碰撞尺寸
 *
 * 算法原理：
 * 1. 采用迭代的六方向膨胀策略：Y-、Y+、X+、X-、Z+、Z-
 * 2. 每个方向膨胀时，检查整个表面是否与障碍物碰撞
 * 3. 一旦检测到碰撞就停止该方向的膨胀
 * 4. 重复直到所有方向都无法继续膨胀
 *
 * @param cube 初始立方体（通常是单位网格立方体）
 * @param lstcube 上一个立方体（用于检查包含关系）
 * @return pair<Cube, bool> 第一个元素是膨胀后的立方体，第二个元素表示是否成功
 */
pair<Cube, bool> inflateCube(Cube cube, Cube lstcube)
{
    Cube cubeMax = cube;  // 复制初始立方体作为膨胀结果

    // ============================================
    // 初始化：将立方体顶点转换为网格索引
    // ============================================
    // 膨胀顺序：左、右、前、后、下、上 （Y-、Y+、X+、X-、Z+、Z-）
    MatrixXi vertex_idx(8, 3);  // 存储 8 个顶点的网格索引 [i, j, k]

    // 遵历立方体的 8 个顶点
    for (int i = 0; i < 8; i++)
    {
        // 将顶点坐标限制在地图边界内
        double coord_x = max(min(cube.vertex(i, 0), _pt_max_x), _pt_min_x);
        double coord_y = max(min(cube.vertex(i, 1), _pt_max_y), _pt_min_y);
        double coord_z = max(min(cube.vertex(i, 2), _pt_max_z), _pt_min_z);
        Vector3d coord(coord_x, coord_y, coord_z);

        // 将世界坐标转换为网格索引
        Vector3i pt_idx = collision_map->LocationToGridIndex(coord);

        // 检查初始立方体的顶点是否在障碍物中
        if( collision_map->Get( (int64_t)pt_idx(0), (int64_t)pt_idx(1), (int64_t)pt_idx(2) ).first.occupancy > 0.5 )
        {
            ROS_ERROR("[Planning Node] path has node in obstacles !");
            return make_pair(cubeMax, false);  // 初始立方体本身在障碍物中，无法膨胀
        }

        vertex_idx.row(i) = pt_idx;  // 保存网格索引
    }

    // ============================================
    // 迭代膨胀算法主循环
    // ============================================

    int id_x, id_y, id_z;  // 网格遍历索引

    /*
     * 立方体顶点编号和坐标系定义：
     *
     *            P4(P3)────────────P3(P2)
     *            /|                    /|
     *           / |                   / |              ^ Z
     *          /  |                  /  |              |
     *      P1(P0)───────────P2(P1)  |              |
     *         |   |                 |   |              |
     *         |   P8(P7)────────|---P7(P6)          /--------> Y
     *         |  /                  |  /              /
     *         | /                   | /              /
     *         |/                    |/              /
     *      P5(P4)───────────P6(P5)          X
     *
     * 注意：括号内为实际编码索引(0-7)，括号外为理论编号(1-8)
     */

    bool collide;  // 碰撞检测标志

    MatrixXi vertex_idx_lst = vertex_idx;  // 保存上一次迭代的顶点索引

    int iter = 0;  // 迭代计数器

    // 迭代膨胀，直到无法继续扩大或达到最大迭代次数
    while(iter < _max_inflate_iter)
    {
        collide = false;  // 重置碰撞标志

        // ============================================
        // Y- 方向膨胀：左侧面 (P1-P4-P8-P5) 面扫描
        // ============================================ 
        // 计算Y方向的膨胀边界
        int y_lo = max(0, vertex_idx(0, 1) - _step_length);      // Y方向下边界
        int y_up = min(_max_y_id, vertex_idx(1, 1) + _step_length);  // Y方向上边界

        // 在Y-方向逐层扫描，检查是否可以继续膨胀
        for(id_y = vertex_idx(0, 1); id_y >= y_lo; id_y-- )
        {
            if( collide == true)  // 如果已经检测到碰撞，停止扫描
                break;

            // 扫描当前Y层的所有X坐标
            for(id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x-- )
            {
                if( collide == true)  // 如果已经检测到碰撞，停止扫描
                    break;

                // 扫描当前XY平面的所有Z坐标
                for(id_z = vertex_idx(0, 2); id_z >= vertex_idx(4, 2); id_z-- )
                {
                    // 查询当前体素的占据状态
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;
                    if(occupy > 0.5)  // 占据概率 > 0.5 表示有障碍物
                    {
                        collide = true;  // 检测到碰撞
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(0, 1) = min(id_y+2, vertex_idx(0, 1));
            vertex_idx(3, 1) = min(id_y+2, vertex_idx(3, 1));
            vertex_idx(7, 1) = min(id_y+2, vertex_idx(7, 1));
            vertex_idx(4, 1) = min(id_y+2, vertex_idx(4, 1));
        }
        else
            vertex_idx(0, 1) = vertex_idx(3, 1) = vertex_idx(7, 1) = vertex_idx(4, 1) = id_y + 1;
        
        // Y+ now is the right side : (p2 -- p3 -- p7 -- p6) face
        // ############################################################################################################
        collide = false;
        for(id_y = vertex_idx(1, 1); id_y <= y_up; id_y++ )
        {   
            if( collide == true) 
                break;
            
            for(id_x = vertex_idx(1, 0); id_x >= vertex_idx(2, 0); id_x-- )
            {
                if( collide == true) 
                    break;

                for(id_z = vertex_idx(1, 2); id_z >= vertex_idx(5, 2); id_z-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(1, 1) = max(id_y-2, vertex_idx(1, 1));
            vertex_idx(2, 1) = max(id_y-2, vertex_idx(2, 1));
            vertex_idx(6, 1) = max(id_y-2, vertex_idx(6, 1));
            vertex_idx(5, 1) = max(id_y-2, vertex_idx(5, 1));
        }
        else
            vertex_idx(1, 1) = vertex_idx(2, 1) = vertex_idx(6, 1) = vertex_idx(5, 1) = id_y - 1;

        // X + now is the front side : (p1 -- p2 -- p6 -- p5) face
        // ############################################################################################################
        int x_lo = max(0, vertex_idx(3, 0) - _step_length);
        int x_up = min(_max_x_id, vertex_idx(0, 0) + _step_length);

        collide = false;
        for(id_x = vertex_idx(0, 0); id_x <= x_up; id_x++ )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_z = vertex_idx(0, 2); id_z >= vertex_idx(4, 2); id_z-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(0, 0) = max(id_x-2, vertex_idx(0, 0)); 
            vertex_idx(1, 0) = max(id_x-2, vertex_idx(1, 0)); 
            vertex_idx(5, 0) = max(id_x-2, vertex_idx(5, 0)); 
            vertex_idx(4, 0) = max(id_x-2, vertex_idx(4, 0)); 
        }
        else
            vertex_idx(0, 0) = vertex_idx(1, 0) = vertex_idx(5, 0) = vertex_idx(4, 0) = id_x - 1;    

        // X- now is the back side : (p4 -- p3 -- p7 -- p8) face
        // ############################################################################################################
        collide = false;
        for(id_x = vertex_idx(3, 0); id_x >= x_lo; id_x-- )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(3, 1); id_y <= vertex_idx(2, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_z = vertex_idx(3, 2); id_z >= vertex_idx(7, 2); id_z-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(3, 0) = min(id_x+2, vertex_idx(3, 0)); 
            vertex_idx(2, 0) = min(id_x+2, vertex_idx(2, 0)); 
            vertex_idx(6, 0) = min(id_x+2, vertex_idx(6, 0)); 
            vertex_idx(7, 0) = min(id_x+2, vertex_idx(7, 0)); 
        }
        else
            vertex_idx(3, 0) = vertex_idx(2, 0) = vertex_idx(6, 0) = vertex_idx(7, 0) = id_x + 1;

        // Z+ now is the above side : (p1 -- p2 -- p3 -- p4) face
        // ############################################################################################################
        collide = false;
        int z_lo = max(0, vertex_idx(4, 2) - _step_length);
        int z_up = min(_max_z_id, vertex_idx(0, 2) + _step_length);
        for(id_z = vertex_idx(0, 2); id_z <= z_up; id_z++ )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(0, 2) = max(id_z-2, vertex_idx(0, 2));
            vertex_idx(1, 2) = max(id_z-2, vertex_idx(1, 2));
            vertex_idx(2, 2) = max(id_z-2, vertex_idx(2, 2));
            vertex_idx(3, 2) = max(id_z-2, vertex_idx(3, 2));
        }
        vertex_idx(0, 2) = vertex_idx(1, 2) = vertex_idx(2, 2) = vertex_idx(3, 2) = id_z - 1;

        // now is the below side : (p5 -- p6 -- p7 -- p8) face
        // ############################################################################################################
        collide = false;
        for(id_z = vertex_idx(4, 2); id_z >= z_lo; id_z-- )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(4, 1); id_y <= vertex_idx(5, 1); id_y++ )
            {
                if( collide == true) 
                    break;

                for(id_x = vertex_idx(4, 0); id_x >= vertex_idx(7, 0); id_x-- )
                {
                    double occupy = collision_map->Get( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z).first.occupancy;    
                    if(occupy > 0.5) // the voxel is occupied
                    {   
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            vertex_idx(4, 2) = min(id_z+2, vertex_idx(4, 2));
            vertex_idx(5, 2) = min(id_z+2, vertex_idx(5, 2));
            vertex_idx(6, 2) = min(id_z+2, vertex_idx(6, 2));
            vertex_idx(7, 2) = min(id_z+2, vertex_idx(7, 2));
        }
        else
            vertex_idx(4, 2) = vertex_idx(5, 2) = vertex_idx(6, 2) = vertex_idx(7, 2) = id_z + 1;

        if(vertex_idx_lst == vertex_idx)
            break;

        vertex_idx_lst = vertex_idx;

        MatrixXd vertex_coord(8, 3);
        for(int i = 0; i < 8; i++)
        {   
            int index_x = max(min(vertex_idx(i, 0), _max_x_id - 1), 0);
            int index_y = max(min(vertex_idx(i, 1), _max_y_id - 1), 0);
            int index_z = max(min(vertex_idx(i, 2), _max_z_id - 1), 0);

            Vector3i index(index_x, index_y, index_z);
            Vector3d pos = collision_map->GridIndexToLocation(index);
            vertex_coord.row(i) = pos;
        }

        cubeMax.setVertex(vertex_coord, _resolution);
        if( isContains(lstcube, cubeMax))        
            return make_pair(lstcube, false);

        iter ++;
    }

    return make_pair(cubeMax, true);
}

Cube generateCube( Vector3d pt) 
{   
/*
           P4------------P3 
           /|           /|              ^
          / |          / |              | z
        P1--|---------P2 |              |
         |  P8--------|--p7             |
         | /          | /               /--------> y
         |/           |/               /  
        P5------------P6              / x
*/       
    Cube cube;
    
    pt(0) = max(min(pt(0), _pt_max_x), _pt_min_x);
    pt(1) = max(min(pt(1), _pt_max_y), _pt_min_y);
    pt(2) = max(min(pt(2), _pt_max_z), _pt_min_z);

    Vector3i pc_index = collision_map->LocationToGridIndex(pt);    
    Vector3d pc_coord = collision_map->GridIndexToLocation(pc_index);

    cube.center = pc_coord;
    double x_u = pc_coord(0);
    double x_l = pc_coord(0);
    
    double y_u = pc_coord(1);
    double y_l = pc_coord(1);
    
    double z_u = pc_coord(2);
    double z_l = pc_coord(2);

    cube.vertex.row(0) = Vector3d(x_u, y_l, z_u);  
    cube.vertex.row(1) = Vector3d(x_u, y_u, z_u);  
    cube.vertex.row(2) = Vector3d(x_l, y_u, z_u);  
    cube.vertex.row(3) = Vector3d(x_l, y_l, z_u);  

    cube.vertex.row(4) = Vector3d(x_u, y_l, z_l);  
    cube.vertex.row(5) = Vector3d(x_u, y_u, z_l);  
    cube.vertex.row(6) = Vector3d(x_l, y_u, z_l);  
    cube.vertex.row(7) = Vector3d(x_l, y_l, z_l);  

    return cube;
}

bool isContains(Cube cube1, Cube cube2)
{   
    if( cube1.vertex(0, 0) >= cube2.vertex(0, 0) && cube1.vertex(0, 1) <= cube2.vertex(0, 1) && cube1.vertex(0, 2) >= cube2.vertex(0, 2) &&
        cube1.vertex(6, 0) <= cube2.vertex(6, 0) && cube1.vertex(6, 1) >= cube2.vertex(6, 1) && cube1.vertex(6, 2) <= cube2.vertex(6, 2)  )
        return true;
    else
        return false; 
}

void corridorSimplify(vector<Cube> & cubicList)
{
    vector<Cube> cubicSimplifyList;
    for(int j = (int)cubicList.size() - 1; j >= 0; j--)
    {   
        for(int k = j - 1; k >= 0; k--)
        {   
            if(cubicList[k].valid == false)
                continue;
            else if(isContains(cubicList[j], cubicList[k]))
                cubicList[k].valid = false;   
        }
    }

    for(auto cube:cubicList)
        if(cube.valid == true)
            cubicSimplifyList.push_back(cube);

    cubicList = cubicSimplifyList;
}

vector<Cube> corridorGeneration(vector<Vector3d> path_coord, vector<double> time)
{   
    vector<Cube> cubeList;
    Vector3d pt;

    Cube lstcube;

    for (int i = 0; i < (int)path_coord.size(); i += 1)
    {
        pt = path_coord[i];

        Cube cube = generateCube(pt);
        auto result = inflateCube(cube, lstcube);

        if(result.second == false)
            continue;

        cube = result.first;
        
        lstcube = cube;
        cube.t = time[i];
        cubeList.push_back(cube);
    }
    return cubeList;
}

vector<Cube> corridorGeneration(vector<Vector3d> path_coord)
{   
    vector<Cube> cubeList;
    Vector3d pt;

    Cube lstcube;

    for (int i = 0; i < (int)path_coord.size(); i += 1)
    {
        pt = path_coord[i];

        Cube cube = generateCube(pt);
        auto result = inflateCube(cube, lstcube);

        if(result.second == false)
            continue;

        cube = result.first;
        
        lstcube = cube;
        cubeList.push_back(cube);
    }
    return cubeList;
}

double velMapping(double d, double max_v)
{   
    double vel;

    if( d <= 0.25)
        vel = 2.0 * d * d;
    else if(d > 0.25 && d <= 0.75)
        vel = 1.5 * d - 0.25;
    else if(d > 0.75 && d <= 1.0)
        vel = - 2.0 * (d - 1.0) * (d - 1.0) + 1;  
    else
        vel = 1.0;

    return vel * max_v;
}

/**
 * @brief 轨迹规划主函数：集成路径搜索和轨迹优化的完整规划流程
 *
 * 功能概述：
 * 1. 检查前置条件（目标点、地图、里程计）
 * 2. 根据参数选择路径搜索算法（Fast Marching或A*）
 * 3. 生成安全走廊约束轨迹优化
 * 4. 时间分配确保动力学可行性
 * 5. Bezier曲线优化生成平滑轨迹
 * 6. 发布轨迹命令和可视化
 *
 * 算法流程：
 * 前端路径搜索 → 安全走廊生成 → 时间分配 → 后端轨迹优化
 */
void trajPlanning()
{
    // ============================================
    // 前置条件检查：确保所有必要数据就绪
    // ============================================
    if( _has_target == false || _has_map == false || _has_odom == false)
        return;  // 缺少必要数据，无法进行规划

    vector<Cube> corridor;  // 安全走廊：约束轨迹优化的区域

    // ============================================
    // 阶段1：前端路径搜索
    // ============================================
    if(_is_use_fm)  // 选择Fast Marching Method
    {
        // ----------------------------------------
        // Fast Marching算法实现
        // ----------------------------------------

        ros::Time time_1 = ros::Time::now();

        // 计算欧几里得距离场（EDT）：每个点到最近障碍物的距离
        float oob_value = INFINITY;  // 越界值设为无穷大
        auto EDT = collision_map_local->ExtractDistanceField(oob_value);

        ros::Time time_2 = ros::Time::now();
        ROS_WARN("time in generate EDT is %f", (time_2 - time_1).toSec());

        // 初始化Fast Marching所需变量
        unsigned int idx;               // 线性索引
        double max_vel = _MAX_Vel * 0.75;  // 最大允许速度（保守取75%）
        vector<unsigned int> obs;       // 障碍物索引列表
        Vector3d pt;                    // 当前处理的3D点
        vector<int64_t> pt_idx;         // 点索引
        double flow_vel;                // 当前点的流速（速度场值）

        // 获取地图尺寸（网格数量）
        unsigned int size_x = (unsigned int)(_max_x_id);
        unsigned int size_y = (unsigned int)(_max_y_id);
        unsigned int size_z = (unsigned int)(_max_z_id);

        // 创建Fast Marching网格
        Coord3D dimsize {size_x, size_y, size_z};  // 网格维度
        FMGrid3D grid_fmm(dimsize);                // Fast Marching 3D网格

        // 构建速度场：根据距离障碍物的远近设置允许速度
        for(unsigned int k = 0; k < size_z; k++)  // Z方向遍历
        {
            for(unsigned int j = 0; j < size_y; j++)  // Y方向遍历
            {
                for(unsigned int i = 0; i < size_x; i++)  // X方向遍历
                {
                    // 计算线性索引（将3D索引转换为1D索引）
                    idx = k * size_y * size_x + j * size_x + i;

                    // 计算网格中心点的世界坐标
                    pt << (i + 0.5) * _resolution + _map_origin(0),  // X坐标
                          (j + 0.5) * _resolution + _map_origin(1),  // Y坐标
                          (k + 0.5) * _resolution + _map_origin(2);  // Z坐标

                    // 将世界坐标转换为局部地图的网格索引
                    Vector3i index = collision_map_local->LocationToGridIndex(pt);

                    // 检查点是否在局部地图范围内
                    if(collision_map_local->Inside(index))
                    {
                        // 从距离场获取到最近障碍物的距离
                        double d = sqrt(EDT.GetImmutable(index).first.distance_square) * _resolution;
                        // 根据距离映射速度：距离越近速度越小
                        flow_vel = velMapping(d, max_vel);
                    }
                    else
                        flow_vel = max_vel;  // 超出局部地图范围，使用最大速度

                    // 地图边界处设置速度为0（避免越界）
                    if( k == 0 || k == (size_z - 1) ||
                        j == 0 || j == (size_y - 1) ||
                        i == 0 || i == (size_x - 1) )
                        flow_vel = 0.0;

                    // 设置网格点的占据状态（这里用速度值表示）
                    grid_fmm[idx].setOccupancy(flow_vel);

                    // 如果是障碍物点（速度为0），添加到障碍物列表
                    if (grid_fmm[idx].isOccupied())
                        obs.push_back(idx);
                }
            }
        }
        
        grid_fmm.setOccupiedCells(std::move(obs));
        grid_fmm.setLeafSize(_resolution);

        Vector3d startIdx3d = (_start_pt - _map_origin) * _inv_resolution; 
        Vector3d endIdx3d   = (_end_pt   - _map_origin) * _inv_resolution;

        Coord3D goal_point = {(unsigned int)startIdx3d[0], (unsigned int)startIdx3d[1], (unsigned int)startIdx3d[2]};
        Coord3D init_point = {(unsigned int)endIdx3d[0],   (unsigned int)endIdx3d[1],   (unsigned int)endIdx3d[2]}; 

        unsigned int startIdx;
        vector<unsigned int> startIndices;
        grid_fmm.coord2idx(init_point, startIdx);
        
        startIndices.push_back(startIdx);
        
        unsigned int goalIdx;
        grid_fmm.coord2idx(goal_point, goalIdx);
        grid_fmm[goalIdx].setOccupancy(max_vel);     

        Solver<FMGrid3D>* fm_solver = new FMMStar<FMGrid3D>("FMM*_Dist", TIME); // LSM, FMM
    
        fm_solver->setEnvironment(&grid_fmm);
        fm_solver->setInitialAndGoalPoints(startIndices, goalIdx);

        ros::Time time_bef_fm = ros::Time::now();
        if(fm_solver->compute(max_vel) == -1)
        {
            ROS_WARN("[Fast Marching Node] No path can be found");
            _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
            _traj_pub.publish(_traj);
            _has_traj = false;

            return;
        }
        ros::Time time_aft_fm = ros::Time::now();
        ROS_WARN("[Fast Marching Node] Time in Fast Marching computing is %f", (time_aft_fm - time_bef_fm).toSec() );

        Path3D path3D;
        vector<double> path_vels, time;
        GradientDescent< FMGrid3D > grad3D;
        grid_fmm.coord2idx(goal_point, goalIdx);

        if(grad3D.gradient_descent(grid_fmm, goalIdx, path3D, path_vels, time) == -1)
        {
            ROS_WARN("[Fast Marching Node] FMM failed, valid path not exists");
            if(_has_traj && _is_emerg)
            {
                _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
                _traj_pub.publish(_traj);
                _has_traj = false;
            } 
            return;
        }

        vector<Vector3d> path_coord;
        path_coord.push_back(_start_pt);

        double coord_x, coord_y, coord_z;
        for( int i = 0; i < (int)path3D.size(); i++)
        {
            coord_x = max(min( (path3D[i][0]+0.5) * _resolution + _map_origin(0), _x_size), -_x_size);
            coord_y = max(min( (path3D[i][1]+0.5) * _resolution + _map_origin(1), _y_size), -_y_size);
            coord_z = max(min( (path3D[i][2]+0.5) * _resolution, _z_size), 0.0);

            Vector3d pt(coord_x, coord_y, coord_z);
            path_coord.push_back(pt);
        }
        visPath(path_coord);

        ros::Time time_bef_corridor = ros::Time::now();    
        sortPath(path_coord, time);
        corridor = corridorGeneration(path_coord, time);
        ros::Time time_aft_corridor = ros::Time::now();
        ROS_WARN("Time consume in corridor generation is %f", (time_aft_corridor - time_bef_corridor).toSec());

        timeAllocation(corridor, time);
        visCorridor(corridor);

        delete fm_solver;
    }
    else
    {   
        path_finder->linkLocalMap(collision_map_local, _local_origin);
        path_finder->AstarSearch(_start_pt, _end_pt);
        vector<Vector3d> gridPath = path_finder->getPath();
        vector<GridNodePtr> searchedNodes = path_finder->getVisitedNodes();
        path_finder->resetLocalMap();
        
        visGridPath(gridPath);
        visExpNode(searchedNodes);

        ros::Time time_bef_corridor = ros::Time::now();    
        corridor = corridorGeneration(gridPath);
        ros::Time time_aft_corridor = ros::Time::now();
        ROS_WARN("Time consume in corridor generation is %f", (time_aft_corridor - time_bef_corridor).toSec());

        timeAllocation(corridor);
        visCorridor(corridor);
    }

    MatrixXd pos = MatrixXd::Zero(2,3);
    MatrixXd vel = MatrixXd::Zero(2,3);
    MatrixXd acc = MatrixXd::Zero(2,3);

    pos.row(0) = _start_pt;
    pos.row(1) = _end_pt;    
    vel.row(0) = _start_vel;
    acc.row(0) = _start_acc;
    
    double obj;
    ros::Time time_bef_opt = ros::Time::now();

    if(_trajectoryGenerator.BezierPloyCoeffGeneration
        ( corridor, _MQM, pos, vel, acc, _MAX_Vel, _MAX_Acc, _traj_order, _minimize_order, 
         _cube_margin, _is_limit_vel, _is_limit_acc, obj, _bezier_coeff ) == -1 )
    {
        ROS_WARN("Cannot find a feasible and optimal solution, somthing wrong with the mosek solver");
          
        if(_has_traj && _is_emerg)
        {
            _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
            _traj_pub.publish(_traj);
            _has_traj = false;
        } 
    }
    else
    {   
        _seg_num = corridor.size();
        _seg_time.resize(_seg_num);

        for(int i = 0; i < _seg_num; i++)
            _seg_time(i) = corridor[i].t;

        _is_emerg = false;
        _has_traj = true;

        _traj = getBezierTraj();
        _traj_pub.publish(_traj);
        _traj_id ++;
        visBezierTrajectory(_bezier_coeff, _seg_time);
    }

    ros::Time time_aft_opt = ros::Time::now();

    ROS_WARN("The objective of the program is %f", obj);
    ROS_WARN("The time consumation of the program is %f", (time_aft_opt - time_bef_opt).toSec());
}

void sortPath(vector<Vector3d> & path_coord, vector<double> & time)
{   
    vector<Vector3d> path_tmp;
    vector<double> time_tmp;

    for (int i = 0; i < (int)path_coord.size(); i += 1)
    {
        if( i )
            if( std::isinf(time[i]) || time[i] == 0.0 || time[i] == time[i-1] )
                continue;

        if( (path_coord[i] - _end_pt).norm() < 0.2)
            break;

        path_tmp.push_back(path_coord[i]);
        time_tmp.push_back(time[i]);
    }
    path_coord = path_tmp;
    time       = time_tmp;
}   

void timeAllocation(vector<Cube> & corridor, vector<double> time)
{   
    vector<double> tmp_time;

    for(int i  = 0; i < (int)corridor.size() - 1; i++)
    {   
        double duration  = (corridor[i].t - corridor[i+1].t);
        tmp_time.push_back(duration);
    }    
    double lst_time = corridor.back().t;
    tmp_time.push_back(lst_time);

    vector<Vector3d> points;
    points.push_back (_start_pt);
    for(int i = 1; i < (int)corridor.size(); i++)
        points.push_back(corridor[i].center);

    points.push_back (_end_pt);

    double _Vel = _MAX_Vel * 0.6;
    double _Acc = _MAX_Acc * 0.6;

    Eigen::Vector3d initv = _start_vel;
    for(int i = 0; i < (int)points.size() - 1; i++)
    {
        double dtxyz;

        Eigen::Vector3d p0   = points[i];    
        Eigen::Vector3d p1   = points[i + 1];
        Eigen::Vector3d d    = p1 - p0;            
        Eigen::Vector3d v0(0.0, 0.0, 0.0);        
        
        if( i == 0) v0 = initv;

        double D    = d.norm();                   
        double V0   = v0.dot(d / D);              
        double aV0  = fabs(V0);                   

        double acct = (_Vel - V0) / _Acc * ((_Vel > V0)?1:-1); 
        double accd = V0 * acct + (_Acc * acct * acct / 2) * ((_Vel > V0)?1:-1);
        double dcct = _Vel / _Acc;                                              
        double dccd = _Acc * dcct * dcct / 2;                                   

        if (D < aV0 * aV0 / (2 * _Acc))
        {                 
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = aV0 / _Acc;
            dtxyz     = t1 + t2;                 
        }
        else if (D < accd + dccd)
        {
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = (-aV0 + sqrt(aV0 * aV0 + _Acc * D - aV0 * aV0 / 2)) / _Acc;
            double t3 = (aV0 + _Acc * t2) / _Acc;
            dtxyz     = t1 + t2 + t3;    
        }
        else
        {
            double t1 = acct;                              
            double t2 = (D - accd - dccd) / _Vel;
            double t3 = dcct;
            dtxyz     = t1 + t2 + t3;
        }

        if(dtxyz < tmp_time[i] * 0.5)
            tmp_time[i] = dtxyz; // if FM given time in this segment is rediculous long, use the new value
    }

    for(int i = 0; i < (int)corridor.size(); i++)
        corridor[i].t = tmp_time[i];
}

void timeAllocation(vector<Cube> & corridor)
{   
    vector<Vector3d> points;
    points.push_back (_start_pt);

    for(int i = 1; i < (int)corridor.size(); i++)
        points.push_back(corridor[i].center);

    points.push_back (_end_pt);

    double _Vel = _MAX_Vel * 0.6;
    double _Acc = _MAX_Acc * 0.6;

    for (int k = 0; k < (int)points.size() - 1; k++)
    {
          double dtxyz;
          Vector3d p0   = points[k];        
          Vector3d p1   = points[k + 1];    
          Vector3d d    = p1 - p0;          
          Vector3d v0(0.0, 0.0, 0.0);       
          
          if( k == 0) v0 = _start_vel;

          double D    = d.norm();                  
          double V0   = v0.dot(d / D);             
          double aV0  = fabs(V0);                  

          double acct = (_Vel - V0) / _Acc * ((_Vel > V0)?1:-1);
          double accd = V0 * acct + (_Acc * acct * acct / 2) * ((_Vel > V0)?1:-1);
          double dcct = _Vel / _Acc;                                              
          double dccd = _Acc * dcct * dcct / 2;                                   

          if (D < aV0 * aV0 / (2 * _Acc))
          {               
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = aV0 / _Acc;
            dtxyz     = t1 + t2;                 
          }
          else if (D < accd + dccd)
          {
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = (-aV0 + sqrt(aV0 * aV0 + _Acc * D - aV0 * aV0 / 2)) / _Acc;
            double t3 = (aV0 + _Acc * t2) / _Acc;
            dtxyz     = t1 + t2 + t3;    
          }
          else
          {
            double t1 = acct;                              
            double t2 = (D - accd - dccd) / _Vel;
            double t3 = dcct;
            dtxyz     = t1 + t2 + t3;                                                                  
          }
          corridor[k].t = dtxyz;
      }
}

/**
 * @brief BTraj轨迹规划节点主函数
 *
 * 程序入口点，负责：
 * 1. ROS节点初始化和参数配置
 * 2. 订阅者和发布者设置
 * 3. 算法核心组件初始化
 * 4. 主循环运行
 */
int main(int argc, char** argv)
{
    // ============================================
    // ROS节点初始化
    // ============================================
    ros::init(argc, argv, "b_traj_node");  // 初始化ROS节点，命名为"b_traj_node"
    ros::NodeHandle nh("~");               // 创建私有命名空间的节点句柄

    // ============================================
    // 订阅者设置 - 接收外部数据
    // ============================================
    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );  // 订阅点云地图
    _odom_sub = nh.subscribe( "odometry",  1, rcvOdometryCallbck);      // 订阅里程计数据
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );   // 订阅目标点

    // ============================================
    // 发布者设置 - 发送结果数据
    // ============================================

    // 可视化相关发布者
    _inf_map_vis_pub   = nh.advertise<sensor_msgs::PointCloud2>("vis_map_inflate", 1);      // 膨胀地图可视化
    _local_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("vis_map_local", 1);        // 局部地图可视化
    _traj_vis_pub      = nh.advertise<visualization_msgs::Marker>("trajectory_vis", 1);      // 轨迹可视化
    _corridor_vis_pub  = nh.advertise<visualization_msgs::MarkerArray>("corridor_vis", 1);   // 走廊可视化
    _fm_path_vis_pub   = nh.advertise<visualization_msgs::MarkerArray>("path_vis", 1);       // Fast Marching路径可视化
    _grid_path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("grid_path_vis", 1);  // A*网格路径可视化
    _nodes_vis_pub     = nh.advertise<visualization_msgs::Marker>("expanded_nodes_vis", 1);  // A*搜索节点可视化
    _checkTraj_vis_pub = nh.advertise<visualization_msgs::Marker>("check_trajectory", 1);    // 轨迹检查可视化
    _stopTraj_vis_pub  = nh.advertise<visualization_msgs::Marker>("stop_trajectory", 1);     // 紧急停止轨迹可视化

    // 控制指令发布者
    _traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 10);  // 轨迹指令发布

    // ============================================
    // 参数配置 - 从launch文件或参数服务器读取
    // ============================================

    // 地图相关参数
    nh.param("map/margin",     _cloud_margin, 0.25);  // 点云膨胀边界（米）
    nh.param("map/resolution", _resolution, 0.2);     // 地图分辨率（米/格）

    // 全局地图尺寸
    nh.param("map/x_size",       _x_size, 50.0);      // X方向地图尺寸（米）
    nh.param("map/y_size",       _y_size, 50.0);      // Y方向地图尺寸（米）
    nh.param("map/z_size",       _z_size, 5.0 );      // Z方向地图尺寸（米）

    // 局部地图尺寸（提高计算效率）
    nh.param("map/x_local_size", _x_local_size, 20.0); // X方向局部地图尺寸（米）
    nh.param("map/y_local_size", _y_local_size, 20.0); // Y方向局部地图尺寸（米）
    nh.param("map/z_local_size", _z_local_size, 5.0 ); // Z方向局部地图尺寸（米）

    // 初始位置参数
    nh.param("planning/init_x",       _init_x,  0.0);  // 初始X坐标（米）
    nh.param("planning/init_y",       _init_y,  0.0);  // 初始Y坐标（米）
    nh.param("planning/init_z",       _init_z,  0.0);  // 初始Z坐标（米）

    // 动力学约束参数
    nh.param("planning/max_vel",       _MAX_Vel,  1.0); // 最大速度（米/秒）
    nh.param("planning/max_acc",       _MAX_Acc,  1.0); // 最大加速度（米/秒²）

    // 算法内部参数
    nh.param("planning/max_inflate",   _max_inflate_iter, 100); // 立方体膨胀最大迭代次数
    nh.param("planning/step_length",   _step_length,     2);    // 膨胀步长（网格单位）
    nh.param("planning/cube_margin",   _cube_margin,   0.2);    // 立方体安全边界（米）

    // 安全检查参数
    nh.param("planning/check_horizon", _check_horizon,10.0);    // 碰撞检测前瞻时间（秒）
    nh.param("planning/stop_horizon",  _stop_horizon,  5.0);    // 紧急停止判断时间（秒）

    // 优化约束开关
    nh.param("planning/is_limit_vel",  _is_limit_vel,  false);  // 是否启用速度约束
    nh.param("planning/is_limit_acc",  _is_limit_acc,  false);  // 是否启用加速度约束

    // 算法选择参数
    nh.param("planning/is_use_fm",     _is_use_fm,  true);     // 是否使用Fast Marching（否则使用A*）

    // 轨迹优化参数
    nh.param("optimization/min_order",  _minimize_order, 3.0);  // 优化目标阶数（snap最小化）
    nh.param("optimization/poly_order", _traj_order,    10);    // Bezier曲线阶数

    // 可视化参数
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);     // 轨迹显示线宽
    nh.param("vis/is_proj_cube",   _is_proj_cube, true);       // 是否投影立方体到2D

    // ============================================
    // Bernstein基函数初始化
    // ============================================
    Bernstein _bernstein;  // 创建Bernstein基函数对象
    if(_bernstein.setParam(3, 12, _minimize_order) == -1)
        ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set ");

    // 获取预计算的Bernstein基函数矩阵（提高运行效率）
    _MQM = _bernstein.getMQM()[_traj_order];   // 最小二乘成本矩阵
    _FM  = _bernstein.getFM()[_traj_order];    // 映射矩阵
    _C   = _bernstein.getC()[_traj_order];     // 位置系数向量
    _Cv  = _bernstein.getC_v()[_traj_order];   // 速度系数向量
    _Ca  = _bernstein.getC_a()[_traj_order];   // 加速度系数向量
    _Cj  = _bernstein.getC_j()[_traj_order];   // Jerk系数向量

    // ============================================
    // 地图坐标系和边界计算
    // ============================================

    // 设置地图原点（中心对齐）
    _map_origin << -_x_size/2.0, -_y_size/2.0, 0.0;

    // 计算地图边界坐标
    _pt_max_x = + _x_size / 2.0;  // X方向最大值
    _pt_min_x = - _x_size / 2.0;  // X方向最小值
    _pt_max_y = + _y_size / 2.0;  // Y方向最大值
    _pt_min_y = - _y_size / 2.0;  // Y方向最小值
    _pt_max_z = + _z_size;        // Z方向最大值
    _pt_min_z = 0.0;              // Z方向最小值（地面）

    // 计算网格参数
    _inv_resolution = 1.0 / _resolution;  // 分辨率倒数（优化计算）

    // 全局地图网格数量
    _max_x_id = (int)(_x_size * _inv_resolution);        // X方向网格数
    _max_y_id = (int)(_y_size * _inv_resolution);        // Y方向网格数
    _max_z_id = (int)(_z_size * _inv_resolution);        // Z方向网格数

    // 局部地图网格数量
    _max_local_x_id = (int)(_x_local_size * _inv_resolution);  // 局部X方向网格数
    _max_local_y_id = (int)(_y_local_size * _inv_resolution);  // 局部Y方向网格数
    _max_local_z_id = (int)(_z_local_size * _inv_resolution);  // 局部Z方向网格数

    // ============================================
    // 算法核心组件初始化
    // ============================================

    // 设置地图尺寸向量
    Vector3i GLSIZE(_max_x_id, _max_y_id, _max_z_id);                        // 全局地图尺寸
    Vector3i LOSIZE(_max_local_x_id, _max_local_y_id, _max_local_z_id);      // 局部地图尺寸

    // 初始化A*路径搜索器
    path_finder = new gridPathFinder(GLSIZE, LOSIZE);
    path_finder->initGridNodeMap(_resolution, _map_origin);

    // 初始化全局碰撞检测地图
    Translation3d origin_translation( _map_origin(0), _map_origin(1), 0.0);  // 平移变换
    Quaterniond origin_rotation(1.0, 0.0, 0.0, 0.0);                       // 旋转变换（单位四元数）
    Affine3d origin_transform = origin_translation * origin_rotation;  // 组合变换

    // 创建全局碰撞检测地图
    collision_map = new CollisionMapGrid(origin_transform, "world", _resolution,
                                        _x_size, _y_size, _z_size, _free_cell);

    // ============================================
    // 主循环运行
    // ============================================
    ros::Rate rate(100);        // 设置循环频率为100Hz
    bool status = ros::ok();    // 检查ROS节点状态

    ROS_INFO("[BTraj] Trajectory planning node started successfully!");
    ROS_INFO("[BTraj] Waiting for map, odometry, and waypoint data...");

    // 主事件循环：处理ROS回调函数
    while(status)
    {
        ros::spinOnce();     // 处理所有挂起的ROS回调函数
        status = ros::ok();  // 检查节点是否仍在运行
        rate.sleep();        // 睡眠以维持100Hz频率
    }

    ROS_INFO("[BTraj] Trajectory planning node shutting down...");
    return 0;  // 程序正常退出
}

/**
 * @brief 将Bezier系数矩阵转换为ROS轨迹消息
 *
 * 功能说明：
 * 1. 封装Bezier系数为四旋翼可执行的多项式轨迹消息
 * 2. 设置时间戳和轨迹ID
 * 3. 按照[x,y,z]顺序组织系数数据
 *
 * @return 封装好的ROS轨迹消息
 */
quadrotor_msgs::PolynomialTrajectory getBezierTraj()
{
    quadrotor_msgs::PolynomialTrajectory traj;  // 创建轨迹消息

    // 设置轨迹基本属性
    traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;  // 设置为添加新轨迹
    traj.num_segment = _seg_num;                                    // 轨迹段数

    // 计算系数数组大小
    int order = _traj_order;                      // Bezier曲线阶数
    int poly_num1d = order + 1;                   // 单个轴的系数数量（n阶曲线有n+1个系数）
    int polyTotalNum = _seg_num * (order + 1);    // 单轴系数总数

      traj.coef_x.resize(polyTotalNum);
      traj.coef_y.resize(polyTotalNum);
      traj.coef_z.resize(polyTotalNum);

      int idx = 0;
      for(int i = 0; i < _seg_num; i++ )
      {    
          for(int j =0; j < poly_num1d; j++)
          { 
              traj.coef_x[idx] = _bezier_coeff(i,                  j);
              traj.coef_y[idx] = _bezier_coeff(i,     poly_num1d + j);
              traj.coef_z[idx] = _bezier_coeff(i, 2 * poly_num1d + j);
              idx++;
          }
      }

      traj.header.frame_id = "/bernstein";
      traj.header.stamp = _odom.header.stamp; 
      _start_time = traj.header.stamp;

      traj.time.resize(_seg_num);
      traj.order.resize(_seg_num);

      traj.mag_coeff = 1.0;
      for (int idx = 0; idx < _seg_num; ++idx){
          traj.time[idx] = _seg_time(idx);
          traj.order[idx] = _traj_order;
      }
      
      traj.start_yaw = 0.0;
      traj.final_yaw = 0.0;

      traj.trajectory_id = _traj_id;
      traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

      return traj;
}

Vector3d getPosFromBezier(const MatrixXd & polyCoeff, double t_now, int seg_now )
{
    Vector3d ret = VectorXd::Zero(3);
    VectorXd ctrl_now = polyCoeff.row(seg_now);
    int ctrl_num1D = polyCoeff.cols() / 3;

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < ctrl_num1D; j++)
            ret(i) += _C(j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (_traj_order - j) ); 

    return ret;  
}

VectorXd getStateFromBezier(const MatrixXd & polyCoeff, double t_now, int seg_now )
{
    VectorXd ret = VectorXd::Zero(12);

    VectorXd ctrl_now = polyCoeff.row(seg_now);
    int ctrl_num1D = polyCoeff.cols() / 3;

    for(int i = 0; i < 3; i++)
    {   
        for(int j = 0; j < ctrl_num1D; j++){
            ret[i] += _C(j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (_traj_order - j) ); 
          
            if(j < ctrl_num1D - 1 )
                ret[i+3] += _Cv(j) * _traj_order 
                      * ( ctrl_now(i * ctrl_num1D + j + 1) - ctrl_now(i * ctrl_num1D + j))
                      * pow(t_now, j) * pow((1 - t_now), (_traj_order - j - 1) ); 
          
            if(j < ctrl_num1D - 2 )
                ret[i+6] += _Ca(j) * _traj_order * (_traj_order - 1) 
                      * ( ctrl_now(i * ctrl_num1D + j + 2) - 2 * ctrl_now(i * ctrl_num1D + j + 1) + ctrl_now(i * ctrl_num1D + j))
                      * pow(t_now, j) * pow((1 - t_now), (_traj_order - j - 2) );                         

            if(j < ctrl_num1D - 3 )
                ret[i+9] += _Cj(j) * _traj_order * (_traj_order - 1) * (_traj_order - 2) 
                      * ( ctrl_now(i * ctrl_num1D + j + 3) - 3 * ctrl_now(i * ctrl_num1D + j + 2) + 3 * ctrl_now(i * ctrl_num1D + j + 1) - ctrl_now(i * ctrl_num1D + j))
                      * pow(t_now, j) * pow((1 - t_now), (_traj_order - j - 3) );                         
        }
    }

    return ret;  
}

visualization_msgs::MarkerArray path_vis; 
void visPath(vector<Vector3d> path)
{
    for(auto & mk: path_vis.markers) 
        mk.action = visualization_msgs::Marker::DELETE;

    _fm_path_vis_pub.publish(path_vis);
    path_vis.markers.clear();

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.ns = "b_traj/fast_marching_path";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.a = 0.6;
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;

    int idx = 0;
    for(int i = 0; i < int(path.size()); i++)
    {
        mk.id = idx;

        mk.pose.position.x = path[i](0); 
        mk.pose.position.y = path[i](1); 
        mk.pose.position.z = path[i](2);  

        mk.scale.x = _resolution;
        mk.scale.y = _resolution;
        mk.scale.z = _resolution;

        idx ++;
        path_vis.markers.push_back(mk);
    }

    _fm_path_vis_pub.publish(path_vis);
}

visualization_msgs::MarkerArray cube_vis;
void visCorridor(vector<Cube> corridor)
{   
    for(auto & mk: cube_vis.markers) 
        mk.action = visualization_msgs::Marker::DELETE;
    
    _corridor_vis_pub.publish(cube_vis);

    cube_vis.markers.clear();

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.ns = "corridor";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.a = 0.4;
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;

    int idx = 0;
    for(int i = 0; i < int(corridor.size()); i++)
    {   
        mk.id = idx;

        mk.pose.position.x = (corridor[i].vertex(0, 0) + corridor[i].vertex(3, 0) ) / 2.0; 
        mk.pose.position.y = (corridor[i].vertex(0, 1) + corridor[i].vertex(1, 1) ) / 2.0; 

        if(_is_proj_cube)
            mk.pose.position.z = 0.0; 
        else
            mk.pose.position.z = (corridor[i].vertex(0, 2) + corridor[i].vertex(4, 2) ) / 2.0; 

        mk.scale.x = (corridor[i].vertex(0, 0) - corridor[i].vertex(3, 0) );
        mk.scale.y = (corridor[i].vertex(1, 1) - corridor[i].vertex(0, 1) );

        if(_is_proj_cube)
            mk.scale.z = 0.05; 
        else
            mk.scale.z = (corridor[i].vertex(0, 2) - corridor[i].vertex(4, 2) );

        idx ++;
        cube_vis.markers.push_back(mk);
    }

    _corridor_vis_pub.publish(cube_vis);
}

void visBezierTrajectory(MatrixXd polyCoeff, VectorXd time)
{   
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "world";

    traj_vis.ns = "trajectory/trajectory";
    traj_vis.id = 0;
    traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    
    traj_vis.action = visualization_msgs::Marker::DELETE;
    _checkTraj_vis_pub.publish(traj_vis);
    _stopTraj_vis_pub.publish(traj_vis);

    traj_vis.action = visualization_msgs::Marker::ADD;
    traj_vis.scale.x = _vis_traj_width;
    traj_vis.scale.y = _vis_traj_width;
    traj_vis.scale.z = _vis_traj_width;
    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;
    traj_vis.color.r = 1.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 0.0;
    traj_vis.color.a = 0.6;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();
    
    traj_vis.points.clear();

    Vector3d state;
    geometry_msgs::Point pt;

    int segment_num  = polyCoeff.rows();
    for(int i = 0; i < segment_num; i++ ){
        for (double t = 0.0; t < 1.0; t += 0.05 / time(i), count += 1){
            state = getPosFromBezier( polyCoeff, t, i );
            cur(0) = pt.x = time(i) * state(0);
            cur(1) = pt.y = time(i) * state(1);
            cur(2) = pt.z = time(i) * state(2);
            traj_vis.points.push_back(pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
    }

    ROS_INFO("[GENERATOR] The length of the trajectory; %.3lfm.", traj_len);
    _traj_vis_pub.publish(traj_vis);
}

visualization_msgs::MarkerArray grid_vis; 
void visGridPath( vector<Vector3d> grid_path )
{   
    for(auto & mk: grid_vis.markers) 
        mk.action = visualization_msgs::Marker::DELETE;

    _grid_path_vis_pub.publish(grid_vis);
    grid_vis.markers.clear();

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.ns = "b_traj/grid_path";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;

    int idx = 0;
    for(int i = 0; i < int(grid_path.size()); i++)
    {
        mk.id = idx;

        mk.pose.position.x = grid_path[i](0); 
        mk.pose.position.y = grid_path[i](1); 
        mk.pose.position.z = grid_path[i](2);  

        mk.scale.x = _resolution;
        mk.scale.y = _resolution;
        mk.scale.z = _resolution;

        idx ++;
        grid_vis.markers.push_back(mk);
    }

    _grid_path_vis_pub.publish(grid_vis);
}

void visExpNode( vector<GridNodePtr> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "b_traj/visited_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.3;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i]->coord;
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _nodes_vis_pub.publish(node_vis);
}
