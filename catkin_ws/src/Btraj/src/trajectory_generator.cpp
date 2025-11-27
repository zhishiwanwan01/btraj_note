#include "trajectory_generator.h"
using namespace std;
using namespace Eigen;

static void MSKAPI printstr(void* handle, MSKCONST char str[]) {
  printf("%s", str);
}

/**
 * @brief 贝塞尔曲线轨迹优化函数，使用 Mosek QP
 * 求解器生成满足约束的最优贝塞尔系数
 *
 * @param corridor
 * 飞行走廊（多段立方体安全区域），每段轨迹的控制点必须位于对应立方体内
 * @param MQM 优化代价矩阵（Minimum-order Quadratic Matrix），用于构建 QP
 * 问题的二次目标函数
 * @param pos 位置边界条件矩阵（2×3）：行0=起点xyz，行1=终点xyz
 * @param vel
 * 速度边界条件矩阵（2×3）：行0=起点速度，行1=终点速度（通常为零向量表示悬停）
 * @param acc
 * 加速度边界条件矩阵（2×3）：行0=起点加速度，行1=终点加速度（通常为零向量）
 * @param maxVel 最大速度限制 (m/s)，约束轨迹上任意点各轴速度分量的绝对值上限
 * @param maxAcc 最大加速度限制
 * (m/s²)，约束轨迹上任意点各轴加速度分量的绝对值上限
 * @param traj_order 贝塞尔多项式阶数（默认8），决定每段轨迹的控制点数量 =
 * traj_order + 1
 * @param minimize_order 优化目标阶数（1=速度, 2=加速度, 3=jerk, 4=snap,
 * 小数=线性插值如2.5）
 * @param margin 飞行走廊安全边距 (m)，控制点约束在 [box.min+margin,
 * box.max-margin] 范围内
 * @param isLimitVel 是否启用速度约束（true=添加速度可行性约束）
 * @param isLimitAcc 是否启用加速度约束（true=添加加速度可行性约束）
 * @param[out] obj 输出：QP 求解器返回的最优目标函数值
 * @param[out] PolyCoeff 输出：贝塞尔系数矩阵（段数 ×
 * 3*(traj_order+1)），每行存储一段轨迹的xyz控制点
 *
 * @return int 返回0表示求解成功，返回-1表示求解失败
 */
int TrajectoryGenerator::BezierPloyCoeffGeneration(
    const vector<Cube>& corridor,
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
    MatrixXd& PolyCoeff)  // define the order to which we minimize.   1 --
                          // velocity, 2 -- acceleration, 3 -- jerk, 4 -- snap
{
#define ENFORCE_VEL \
  isLimitVel  // whether or not adding extra constraints for ensuring the
              // velocity feasibility
#define ENFORCE_ACC \
  isLimitAcc  // whether or not adding extra constraints for ensuring the
              // acceleration feasibility

  // 提取飞行走廊的信息
  double initScale = corridor.front().t;  // 第一段的时间分配
  double lstScale = corridor.back().t;    // 最后一段的时间分配
  int segment_num = corridor.size();      // 轨迹段数

  // 计算控制点数量和约束数量
  int n_poly = traj_order + 1;          // 控制点数量 = Bézier 多项式阶数 + 1
  int s1d1CtrlP_num = n_poly;           // 每段单轴控制点数量
  int s1CtrlP_num = 3 * s1d1CtrlP_num;  // xyz 三轴控制点总数

  // 计算等式约束数量 - 起点、终点、各段连接处、以及总和约束数量
  int equ_con_s_num = 3 * 3;  // p, v, a in x, y, z axis at the start point
  int equ_con_e_num = 3 * 3;  // p, v, a in x, y, z axis at the end point
  int equ_con_continuity_num = 3 * 3 * (segment_num - 1);
  int equ_con_num = equ_con_s_num + equ_con_e_num +
                    equ_con_continuity_num;  // p, v, a in x, y, z axis in each
                                             // segment's joint position

  // 高阶约束数量 - 速度、加速度约束数量 = xyz三轴 * 每段约束点数 * 段数
  int vel_con_num = 3 * traj_order * segment_num;
  int acc_con_num = 3 * (traj_order - 1) * segment_num;

  // 如果用户不启用速度/加速度限制，则将对应约束数量设为0
  // myNOTE 现代C++建议直接使用isLimitVel和isLimitAcc，而不是宏定义
  if (!ENFORCE_VEL)
    vel_con_num = 0;

  if (!ENFORCE_ACC)
    acc_con_num = 0;

  // 高阶约束总数
  int high_order_con_num = vel_con_num + acc_con_num;
  // int high_order_con_num = 0; //3 * traj_order * segment_num;

  int con_num = equ_con_num + high_order_con_num;  // 总约束数量
  int ctrlP_num = segment_num * s1CtrlP_num;       // 总控制点数（优化变量）

  double x_var[ctrlP_num];  // 存储优化结果的数组
  double primalobj;         // 存储最优目标函数值

  //
  MSKrescodee r;  // Mosek 返回码变量
  // 约束边界容器（有文档）, MSKboundkeye: 边界类型枚举
  vector<pair<MSKboundkeye, pair<double, double> > > con_bdk;

  // 速度约束边界
  if (ENFORCE_VEL) {
    /***  Stack the bounding value for the linear inequality for the velocity
     * constraints  ***/
    for (int i = 0; i < vel_con_num; i++) {
      pair<MSKboundkeye, pair<double, double> > cb_ie =
          make_pair(MSK_BK_RA, make_pair(-maxVel, +maxVel));
      con_bdk.push_back(cb_ie);
    }
  }

  // 加速度约束边界
  if (ENFORCE_ACC) {
    /***  Stack the bounding value for the linear inequality for the
     * acceleration constraints  ***/
    for (int i = 0; i < acc_con_num; i++) {
      pair<MSKboundkeye, pair<double, double> > cb_ie =
          make_pair(MSK_BK_RA, make_pair(-maxAcc, maxAcc));
      con_bdk.push_back(cb_ie);
    }
  }

  // ROS_WARN("[Bezier Trajectory] equality bound %d", equ_con_num);
  // 设置等式约束边界
  for (int i = 0; i < equ_con_num; i++) {
    double beq_i;
    if (i < 3)
      beq_i = pos(0, i);
    else if (i >= 3 && i < 6)
      beq_i = vel(0, i - 3);
    else if (i >= 6 && i < 9)
      beq_i = acc(0, i - 6);
    else if (i >= 9 && i < 12)
      beq_i = pos(1, i - 9);
    else if (i >= 12 && i < 15)
      beq_i = vel(1, i - 12);
    else if (i >= 15 && i < 18)
      beq_i = acc(1, i - 15);
    else
      beq_i = 0.0;

    pair<MSKboundkeye, pair<double, double> > cb_eq = make_pair(
        MSK_BK_FX, make_pair(beq_i, beq_i));  // # cb_eq means: constriants
                                              // boundary of equality constrain
    con_bdk.push_back(cb_eq);
  }

  // 创建一个容器，存储每个优化变量（控制点）的边界类型和上下界
  /* ## define a container for control points' boundary and boundkey ## */
  /* ## dataType in one tuple is : boundary type, lower bound, upper bound ## */
  vector<pair<MSKboundkeye, pair<double, double> > > var_bdk;

  // 填充var_bdk
  // 外层 k：遍历每个轨迹段（segment）
  // 中层 i：遍历 x, y, z 三个坐标轴
  // 内层 j：遍历每段轨迹的所有控制点
  for (int k = 0; k < segment_num; k++) {
    Cube cube_ = corridor[k];
    double scale_k = cube_.t;

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < n_poly; j++) {
        pair<MSKboundkeye, pair<double, double> > vb_x;

        double lo_bound, up_bound;
        if (k > 0) {
          lo_bound = (cube_.box[i].first + margin) / scale_k;
          up_bound = (cube_.box[i].second - margin) / scale_k;
        } else {
          lo_bound = (cube_.box[i].first) / scale_k;
          up_bound = (cube_.box[i].second) / scale_k;
        }

        vb_x = make_pair(
            MSK_BK_RA,  // Mosek 的边界类型，表示 Range Bound（范围约束）
            make_pair(lo_bound,
                      up_bound));  // # vb_x means: varialbles boundary of
                                   // unknowns x (Polynomial coeff)

        var_bdk.push_back(vb_x);
      }
    }
  }

  MSKint32t j, i;
  MSKenv_t env;    // Mosek 环境句柄
  MSKtask_t task;  // Mosek 任务句柄
  // Create the mosek environment.
  r = MSK_makeenv(&env, NULL);

  // BUG 这里严格来讲不该这个写, 应该先检查r的值如:
  /**
   * if (r = MSK_RES_OK) {
   *     r= MSK_makeenv(&env, NULL);
   * }
   * else {
   *   打印错误信息
   * }
   */
  // Create the optimization task.
  r = MSK_maketask(env, con_num, ctrlP_num, &task);

  // Parameters used in the optimizer
  // ######################################################################
  // MSK_putintparam (task, MSK_IPAR_OPTIMIZER , MSK_OPTIMIZER_INTPNT );
  /**
   * 从上到下依次是:
   * 1. 使用单线程求解 (避免多线程开销)
   * 2. 凸性检查的相对容差 (1%)
   * 3. 对偶可行性容差－对偶问题的约束满足精度
   * 4. 原始可行性容差－原问题的约束满足精度
   * 5. 不可行性容差－判定问题无解的阈值
   * */
  MSK_putintparam(task, MSK_IPAR_NUM_THREADS, 1);
  MSK_putdouparam(task, MSK_DPAR_CHECK_CONVEXITY_REL_TOL, 1e-2);
  MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_DFEAS, 1e-4);
  MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_PFEAS, 1e-4);
  MSK_putdouparam(task, MSK_DPAR_INTPNT_TOL_INFEAS, 1e-4);
  // MSK_putdouparam (task, MSK_DPAR_INTPNT_TOL_REL_GAP, 5e-2 );
  // ######################################################################

  // r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);
  //  Append empty constraints.
  // The constraints will initially have no bounds.
  if (r == MSK_RES_OK)
    // 添加 con_num 个约束（初始无边界）
    r = MSK_appendcons(task, con_num);

  // Append optimizing variables. The variables will initially be fixed at zero
  // (x=0).
  if (r == MSK_RES_OK)
    // 添加 ctrlP_num 个优化变量（初始值为0）
    r = MSK_appendvars(task, ctrlP_num);

  // ROS_WARN("set variables boundary");
  // 设置变量边界（控制点的空间范围）
  for (j = 0; j < ctrlP_num && r == MSK_RES_OK; ++j) {
    if (r == MSK_RES_OK)
      r = MSK_putvarbound(
          task,
          j,                          // Index of variable.
          var_bdk[j].first,           // Bound key.
          var_bdk[j].second.first,    // Numerical value of lower bound.
          var_bdk[j].second.second);  // Numerical value of upper bound.
  }

  // Set the bounds on constraints.
  //   for i=1, ...,con_num : blc[i] <= constraint i <= buc[i]
  // 设置约束边界
  for (i = 0; i < con_num && r == MSK_RES_OK; i++) {
    r = MSK_putconbound(
        task,
        i,                          // Index of constraint.
        con_bdk[i].first,           // Bound key.
        con_bdk[i].second.first,    // Numerical value of lower bound.
        con_bdk[i].second.second);  // Numerical value of upper bound.
  }

  // ROS_WARN("[Bezier Trajectory] Start stacking the Linear Matrix A,
  // inequality part");
  int row_idx = 0;
  // 构建速度约束
  // The velocity constraints
  if (ENFORCE_VEL) {
    for (int k = 0; k < segment_num; k++) {
      for (int i = 0; i < 3; i++) {  // for x, y, z loop
        for (int p = 0; p < traj_order; p++) {
          int nzi = 2;
          MSKint32t asub[nzi];
          double aval[nzi];

          aval[0] = -1.0 * traj_order;
          aval[1] = 1.0 * traj_order;

          asub[0] = k * s1CtrlP_num + i * s1d1CtrlP_num + p;
          asub[1] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 1;

          r = MSK_putarow(task, row_idx, nzi, asub, aval);
          row_idx++;
        }
      }
    }
  }

  // The acceleration constraints
  // 构建加速度约束
  if (ENFORCE_ACC) {
    for (int k = 0; k < segment_num; k++) {
      for (int i = 0; i < 3; i++) {
        for (int p = 0; p < traj_order - 1; p++) {
          int nzi = 3;
          MSKint32t asub[nzi];
          double aval[nzi];

          aval[0] = 1.0 * traj_order * (traj_order - 1) / corridor[k].t;
          aval[1] = -2.0 * traj_order * (traj_order - 1) / corridor[k].t;
          aval[2] = 1.0 * traj_order * (traj_order - 1) / corridor[k].t;
          asub[0] = k * s1CtrlP_num + i * s1d1CtrlP_num + p;
          asub[1] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 1;
          asub[2] = k * s1CtrlP_num + i * s1d1CtrlP_num + p + 2;

          r = MSK_putarow(task, row_idx, nzi, asub, aval);
          row_idx++;
        }
      }
    }
  }
  /*   Start position  */
  // 为起点添加边界条件约束
  {
    // position :
    for (int i = 0; i < 3; i++) {  // loop for x, y, z
      int nzi = 1;
      MSKint32t asub[nzi];
      double aval[nzi];
      aval[0] = 1.0 * initScale;
      asub[0] = i * s1d1CtrlP_num;
      r = MSK_putarow(task, row_idx, nzi, asub, aval);
      row_idx++;
    }
    // velocity :
    for (int i = 0; i < 3; i++) {  // loop for x, y, z
      int nzi = 2;
      MSKint32t asub[nzi];
      double aval[nzi];
      aval[0] = -1.0 * traj_order;
      aval[1] = 1.0 * traj_order;
      asub[0] = i * s1d1CtrlP_num;
      asub[1] = i * s1d1CtrlP_num + 1;
      r = MSK_putarow(task, row_idx, nzi, asub, aval);
      row_idx++;
    }
    // acceleration :
    for (int i = 0; i < 3; i++) {  // loop for x, y, z
      int nzi = 3;
      MSKint32t asub[nzi];
      double aval[nzi];
      aval[0] = 1.0 * traj_order * (traj_order - 1) / initScale;
      aval[1] = -2.0 * traj_order * (traj_order - 1) / initScale;
      aval[2] = 1.0 * traj_order * (traj_order - 1) / initScale;
      asub[0] = i * s1d1CtrlP_num;
      asub[1] = i * s1d1CtrlP_num + 1;
      asub[2] = i * s1d1CtrlP_num + 2;
      r = MSK_putarow(task, row_idx, nzi, asub, aval);
      row_idx++;
    }
  }

  /*   End position  */
  // 为终点添加边界条件约束
  // ROS_WARN(" end position");
  {
    // position :
    for (int i = 0; i < 3; i++) {  // loop for x, y, z
      int nzi = 1;
      MSKint32t asub[nzi];
      double aval[nzi];
      asub[0] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num;
      aval[0] = 1.0 * lstScale;
      r = MSK_putarow(task, row_idx, nzi, asub, aval);
      row_idx++;
    }
    // velocity :
    for (int i = 0; i < 3; i++) {
      int nzi = 2;
      MSKint32t asub[nzi];
      double aval[nzi];
      asub[0] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num - 1;
      asub[1] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num;
      aval[0] = -1.0;
      aval[1] = 1.0;
      r = MSK_putarow(task, row_idx, nzi, asub, aval);
      row_idx++;
    }
    // acceleration :
    for (int i = 0; i < 3; i++) {
      int nzi = 3;
      MSKint32t asub[nzi];
      double aval[nzi];
      asub[0] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num - 2;
      asub[1] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num - 1;
      asub[2] = ctrlP_num - 1 - (2 - i) * s1d1CtrlP_num;
      aval[0] = 1.0 / lstScale;
      aval[1] = -2.0 / lstScale;
      aval[2] = 1.0 / lstScale;
      r = MSK_putarow(task, row_idx, nzi, asub, aval);
      row_idx++;
    }
  }

  /*   joint points  */
  // 为各段连接处添加连续性约束
  // ROS_WARN(" joint position");
  {
    int sub_shift = 0;
    double val0, val1;
    for (int k = 0; k < (segment_num - 1); k++) {
      double scale_k = corridor[k].t;
      double scale_n = corridor[k + 1].t;
      // position :
      val0 = scale_k;
      val1 = scale_n;
      for (int i = 0; i < 3; i++) {  // loop for x, y, z
        int nzi = 2;
        MSKint32t asub[nzi];
        double aval[nzi];

        // This segment's last control point
        aval[0] = 1.0 * val0;
        asub[0] = sub_shift + (i + 1) * s1d1CtrlP_num - 1;

        // Next segment's first control point
        aval[1] = -1.0 * val1;
        asub[1] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;
        r = MSK_putarow(task, row_idx, nzi, asub, aval);
        row_idx++;
      }

      for (int i = 0; i < 3; i++) {
        int nzi = 4;
        MSKint32t asub[nzi];
        double aval[nzi];

        // This segment's last velocity control point
        aval[0] = -1.0;
        aval[1] = 1.0;
        asub[0] = sub_shift + (i + 1) * s1d1CtrlP_num - 2;
        asub[1] = sub_shift + (i + 1) * s1d1CtrlP_num - 1;
        // Next segment's first velocity control point
        aval[2] = 1.0;
        aval[3] = -1.0;

        asub[2] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;
        asub[3] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 1;

        r = MSK_putarow(task, row_idx, nzi, asub, aval);
        row_idx++;
      }
      // acceleration :
      val0 = 1.0 / scale_k;
      val1 = 1.0 / scale_n;
      for (int i = 0; i < 3; i++) {
        int nzi = 6;
        MSKint32t asub[nzi];
        double aval[nzi];

        // This segment's last velocity control point
        aval[0] = 1.0 * val0;
        aval[1] = -2.0 * val0;
        aval[2] = 1.0 * val0;
        asub[0] = sub_shift + (i + 1) * s1d1CtrlP_num - 3;
        asub[1] = sub_shift + (i + 1) * s1d1CtrlP_num - 2;
        asub[2] = sub_shift + (i + 1) * s1d1CtrlP_num - 1;
        // Next segment's first velocity control point
        aval[3] = -1.0 * val1;
        aval[4] = 2.0 * val1;
        aval[5] = -1.0 * val1;
        asub[3] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num;
        asub[4] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 1;
        asub[5] = sub_shift + s1CtrlP_num + i * s1d1CtrlP_num + 2;

        r = MSK_putarow(task, row_idx, nzi, asub, aval);
        row_idx++;
      }

      sub_shift += s1CtrlP_num;
    }
  }

  // ROS_WARN("[Bezier Trajectory] Start stacking the objective");

  int min_order_l = floor(minimize_order);
  int min_order_u = ceil(minimize_order);

  int NUMQNZ = 0;
  for (int i = 0; i < segment_num; i++) {
    int NUMQ_blk =
        (traj_order + 1);  // default minimize the jerk and minimize_order = 3
    NUMQNZ += 3 * NUMQ_blk * (NUMQ_blk + 1) / 2;
  }
  MSKint32t qsubi[NUMQNZ], qsubj[NUMQNZ];
  double qval[NUMQNZ];

  {
    int sub_shift = 0;
    int idx = 0;
    for (int k = 0; k < segment_num; k++) {
      double scale_k = corridor[k].t;
      for (int p = 0; p < 3; p++)
        for (int i = 0; i < s1d1CtrlP_num; i++)
          for (int j = 0; j < s1d1CtrlP_num; j++)
            if (i >= j) {
              qsubi[idx] = sub_shift + p * s1d1CtrlP_num + i;
              qsubj[idx] = sub_shift + p * s1d1CtrlP_num + j;
              // qval[idx]  = MQM(i, j) /(double)pow(scale_k, 3);
              if (min_order_l == min_order_u)
                qval[idx] =
                    MQM(i, j) / (double)pow(scale_k, 2 * min_order_u - 3);
              else
                qval[idx] = ((minimize_order - min_order_l) /
                                 (double)pow(scale_k, 2 * min_order_u - 3) +
                             (min_order_u - minimize_order) /
                                 (double)pow(scale_k, 2 * min_order_l - 3)) *
                            MQM(i, j);
              idx++;
            }

      sub_shift += s1CtrlP_num;
    }
  }

  ros::Time time_end1 = ros::Time::now();

  if (r == MSK_RES_OK)
    r = MSK_putqobj(task, NUMQNZ, qsubi, qsubj, qval);

  if (r == MSK_RES_OK)
    r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);

  // ros::Time time_opt = ros::Time::now();
  bool solve_ok = false;
  if (r == MSK_RES_OK) {
    // ROS_WARN("Prepare to solve the problem ");
    MSKrescodee trmcode;
    r = MSK_optimizetrm(task, &trmcode);
    MSK_solutionsummary(task, MSK_STREAM_LOG);

    if (r == MSK_RES_OK) {
      MSKsolstae solsta;
      MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

      switch (solsta) {
        case MSK_SOL_STA_OPTIMAL:
        case MSK_SOL_STA_NEAR_OPTIMAL:

          r = MSK_getxx(task,
                        MSK_SOL_ITR,  // Request the interior solution.
                        x_var);

          r = MSK_getprimalobj(task, MSK_SOL_ITR, &primalobj);

          obj = primalobj;
          solve_ok = true;

          break;

        case MSK_SOL_STA_DUAL_INFEAS_CER:
        case MSK_SOL_STA_PRIM_INFEAS_CER:
        case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
        case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
          printf("Primal or dual infeasibility certificate found.\n");
          break;

        case MSK_SOL_STA_UNKNOWN:
          printf("The status of the solution could not be determined.\n");
          // solve_ok = true; // debug
          break;
        default:
          printf("Other solution status.");
          break;
      }
    } else {
      printf("Error while optimizing.\n");
    }
  }

  if (r != MSK_RES_OK) {
    // In case of an error print error code and description.
    char symname[MSK_MAX_STR_LEN];
    char desc[MSK_MAX_STR_LEN];

    printf("An error occurred while optimizing.\n");
    MSK_getcodedesc(r, symname, desc);
    printf("Error %s - '%s'\n", symname, desc);
  }

  MSK_deletetask(&task);
  MSK_deleteenv(&env);

  ros::Time time_end2 = ros::Time::now();
  ROS_WARN("time consume in optimize is :");
  cout << time_end2 - time_end1 << endl;

  if (!solve_ok) {
    ROS_WARN("In solver, falied ");
    return -1;
  }

  VectorXd d_var(ctrlP_num);
  for (int i = 0; i < ctrlP_num; i++)
    d_var(i) = x_var[i];

  PolyCoeff = MatrixXd::Zero(segment_num, 3 * (traj_order + 1));

  int var_shift = 0;
  for (int i = 0; i < segment_num; i++) {
    for (int j = 0; j < 3 * n_poly; j++)
      PolyCoeff(i, j) = d_var(j + var_shift);

    var_shift += 3 * n_poly;
  }

  return 1;
}