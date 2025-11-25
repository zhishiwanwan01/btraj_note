---
description: 为 src 中的 cpp 文件生成函数文档，自动放入对应的 code docs 目录
argument-hint: "<function-name> <source-file-path>"
allowed-tools: Read,Write,Glob,Grep
---

为指定函数生成标准格式的文档。

## 存放规则

**源文件路径** → **文档目录映射**：
- `catkin_ws/src/Btraj/src/XXX.cpp` 中的函数
- → 文档保存到 `docs/code docs/XXX.cpp/函数名.md`

示例：
- 源文件：`catkin_ws/src/Btraj/src/trajectory_generator.cpp`
- 函数：`BezierPloyCoeffGeneration`
- 文档位置：`docs/code docs/trajectory_generator.cpp/BezierPloyCoeffGeneration.md`

## 文档格式标准

参考现有文档（如 `docs/code docs/b_traj_node.cpp/rcvOdometryCallbck.md`）：

```markdown
---
tags:
aliases:
date created: [当前日期时间，格式：Day, Month DDth YYYY, H:MM:SS pm]
date modified: [同上]
---
# 函数名 - 简短中文描述
**位置**：`{bash} catkin_ws/src/Btraj/src/文件名.cpp:行号`
**函数签名**：`{cpp} 完整函数签名`

## What（功能）
函数的功能描述，使用编号列表：
1. **第一个功能**：详细说明
2. **第二个功能**：详细说明

## Why（目的）
为什么需要这个函数：
1. **目的1**：说明
2. **目的2**：说明

## 输入参数详解

### 参数名1 - 参数简短描述
| 属性 | 值 |
|------|-----|
| **类型** | `类型` |
| **来源** | 变量来源 |
| **含义** | 参数的含义 |

**矩阵结构**（如果是矩阵）：
```
详细的矩阵结构说明
```

**在函数中的使用**（第XX行）：
```cpp
代码示例
```

### 参数名2 - 参数简短描述
[同上格式]

## 输出参数详解

### 输出参数名 - 描述
| 属性 | 值 |
|------|-----|
| **类型** | `类型&` |
| **含义** | 输出参数的含义 |

## 关键变量说明
（如果适用）

## 相关函数
- `{cpp} 函数名()`: 功能说明 - [[wiki-link]]
```

## 任务步骤

1. **解析参数**：
   - 第一个参数 `$1`：函数名
   - 第二个参数 `$2`：源文件完整路径（如 `catkin_ws/src/Btraj/src/trajectory_generator.cpp`）

2. **定位源文件**：
   - 读取源文件
   - 找到指定函数的定义
   - 记录行号和完整签名

3. **确定输出路径**：
   - 从源文件路径提取文件名（如 `trajectory_generator.cpp`）
   - 构建文档目录：`docs/code docs/文件名/`
   - 文档文件：`docs/code docs/文件名/函数名.md`

4. **生成文档**：
   - 按照上述格式标准
   - **重点详细说明每个输入参数**：类型、来源、含义、矩阵结构、使用位置
   - 包含代码示例和相关函数链接

5. **保存文件**：
   - 确保目录存在
   - 保存到正确位置

## 示例调用

```bash
# 为 trajectory_generator.cpp 中的函数生成文档
/gen-func-doc BezierPloyCoeffGeneration catkin_ws/src/Btraj/src/trajectory_generator.cpp

# 为 b_traj_node.cpp 中的函数生成文档
/gen-func-doc trajPlanning catkin_ws/src/Btraj/src/b_traj_node.cpp
```
