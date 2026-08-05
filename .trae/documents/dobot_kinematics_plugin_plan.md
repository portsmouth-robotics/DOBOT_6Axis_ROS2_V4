# 用控制器正逆解服务替换 KDL 理论 DH 的 MoveIt2 运动学插件

## Context（为什么做这个改动）

**问题**：项目所有 `*_moveit/config/kinematics.yaml` 都用 `kdl_kinematics_plugin/KDLKinematicsPlugin`，它直接吃 URDF 的**理论 DH 参数**做正逆解，导致 MoveIt 规划与真实控制器存在 TCP 位姿偏差。原始诉求是"研发添加自动从控制器读取实际 DH 参数的功能"。

**解决思路**：`dobot_bringup_v4` 节点已经把控制器的正逆解能力封装成 ROS 服务（用真实标定 DH 在控制器内解算）。与其逐台读取 DH，不如**写一个 MoveIt2 运动学插件，把 FK/IK 直接委托给这两个服务**。这样：
- 用的是每台机器自己的真实 DH（控制器内部），无需读取；
- 可靠性高（服务层已处理 TCP 重连/超时）；
- 机型无关，一套插件覆盖全部 CR 系列。

**用户已确认的决策**：
1. 应用到**全部 CR 系列机型**（12 个 moveit 包）。
2. 控制器不可用时**用 KDL 兜底**（保留理论 DH 退化路径，便于离线/仿真）。
3. 姿态 `rx,ry,rz` 默认按 **旋转向量(轴角) + 度**（Dobot CR 仪表盘 API 标准），做成可配置。

---

## 一、用户假设 vs 项目实际（已逐行核对源码）

| 项 | 用户假设 | 项目实际 | 影响 |
|---|---|---|---|
| FK 服务名 | `/dobot_api/get_positive_kin` | `/dobot_bringup_ros2/srv/PositiveKin` | 改服务名 |
| IK 服务名 | `/dobot_api/get_inverse_kin` | `/dobot_bringup_ros2/srv/InverseKin` | 改服务名 |
| FK 响应 | `geometry_msgs/Pose` | `string robot_return`=`"{x,y,z,rx,ry,rz}"` + `int32 res` | 要解析字符串 |
| IK 请求 | `Pose + seed_joints[]` | `float64 x,y,z,rx,ry,rz + string joint_near/user/tool` | 重组请求 |
| IK 响应 | `float64[] + bool success` | `string robot_return`=`"{j1..j6}"` + `int32 res` | 要解析字符串 |
| 单位 | 隐含弧度+米 | 控制器用**度+毫米**；MoveIt/URDF 用弧度+米 | **必须换算** |

**关键证据**：
- 服务注册：`dobot_bringup_v4/src/cr_robot_ros2.cpp:193-194`（服务名拼接见 `cr_robot_ros2.cpp:65-66`，单机器人时前缀为空）
- `.srv`：`dobot_msgs_v4/srv/PositiveKin.srv`、`InverseKin.srv`
- 返回字符串格式：`dobot_bringup_v4/src/command.cpp:164-213`（`doTcpCmd_f` 解析 `err_id{data}`，`robot_return` 带 `{}`）
- 单位证据：`dobot_bringup_v4/src/command.cpp:53` `current_joint_[i] = deg2Rad(real_time_data_->q_actual[i])`（控制器=度，发布时转弧度）
- 请求字符串构造：`dobot_bringup_v4/src/parseTool.cpp:208-229`
- 当前所有 `kinematics.yaml` 用 `kdl_kinematics_plugin/KDLKinematicsPlugin`
- 项目**无任何自定义运动学插件**（无 `PLUGINLIB_EXPORT_CLASS`），`dobot_moveit` 是 Python 包不能承载 C++ 插件 → **必须新建 C++ 包**
- SRDF 示例 `cr3af_moveit/config/cr3af_robot.srdf`：group `cr3af_group`，chain `base_link→Link6`，关节 `joint1..joint6`
- bringup 独立启动：`dobot_bringup_v4/launch/dobot_bringup_ros2.launch.py`（不在 MoveIt launch 内，插件连外部服务）

**已确认的 12 个目标包与组名**（`<robot>_group` 模式）：
`cr3af/cr5af/cr10af/cr20af/cr30h/cr3/cr5/cr7/cr10/cr12/cr16/cr20`（注意 `cr10_moveit` 非 af 也属 CR 系列，纳入）。

---

## 二、架构

```
move_group 进程(MultiThreadedExecutor)
  └─ DobotKinematicsPlugin : kinematics::KinematicsBase
       ├─ 优先: ROS service call → /dobot_bringup_ros2/srv/{Positive,Inverse}Kin
       │        (rad↔deg, m↔mm, quaternion↔旋转向量)
       └─ 失败兜底: pluginlib 加载 kdl_kinematics_plugin/KDLKinematicsPlugin
                    (用同一 RobotModel/Group)
                ↑ TCP
       dobot_bringup_v4 进程(已存在) → 控制器(真实标定DH)
```

**MoveIt2 Humble API 要点**（Plan agent 核对源码确认）：
- `initialize(rclcpp::Node::SharedPtr, RobotModel, group, base, tips, search_disc)` 第一参即 Node，**无需自建 Node**；move_group 的 executor 在后台 spin。
- 基类 `storeValues(...)` 必须显式调用以填充 `base_frame_`/`tip_frames_` 等。
- 基类 `lookupParam(node, key, val, default)` 自动从 `robot_description_kinematics.<group>.<key>` 读参数 → **kinematics.yaml 自定义参数自动进入插件，无需改 launch**。
- 纯虚需 override：`getPositionFK`、`getPositionIK`、`searchPositionIK`(4 重载)、`getJointNames`、`getLinkNames`。
- 服务调用用 `async_send_request` + `future.wait_for(timeout)`（MultiThreadedExecutor 下不死锁）。

---

## 三、新建包 `dobot_kinematics_plugin/`（C++ ament_cmake）

目录：`d:\越疆机器人\更新\github\DOBOT_6Axis_ROS2_V4_feature\dobot_kinematics_plugin\`

### 文件清单
| 文件 | 用途 |
|---|---|
| `package.xml` | 依赖：moveit_core, moveit_kinematics, pluginlib, rclcpp, dobot_msgs_v4, tf2_eigen, eigen。`<export><moveit_core plugin="${prefix}/dobot_kinematics_plugin.xml"/></export>` |
| `CMakeLists.txt` | `pluginlib_export_plugin_description_file(moveit_core dobot_kinematics_plugin.xml)`；`add_library(SHARED src/dobot_kinematics_plugin.cpp)`；`ament_target_dependencies(...)`；install lib/include/xml |
| `dobot_kinematics_plugin.xml` | pluginlib 描述符：`<class name="dobot_kinematics_plugin/DobotKinematicsPlugin" type="dobot_kinematics_plugin::DobotKinematicsPlugin" base_class_type="kinematics::KinematicsBase">` |
| `include/dobot_kinematics_plugin/dobot_kinematics_plugin.hpp` | 类声明，override 8 纯虚 + initialize |
| `src/dobot_kinematics_plugin.cpp` | 实现 |
| `include/dobot_kinematics_plugin/string_parser.hpp` | `parse_brace_list("{a,b,...}")` → `std::array<double,6>` |
| `include/dobot_kinematics_plugin/units.hpp` | 旋转向量(度)↔四元数、RPY(度)↔四元数、deg↔rad |

### 类核心结构（签名）
```cpp
class DobotKinematicsPlugin : public kinematics::KinematicsBase {
  bool initialize(Node, RobotModel, group, base, tips, search_disc) override;  // 读参、建 client、加载 KDL 子插件
  bool getPositionFK(link_names, joint_angles, poses) const override;         // 服务优先→KDL
  bool getPositionIK(ik_pose, seed, solution, error, options) const override;  // → solveIK
  bool searchPositionIK(... 4 重载 ...) const override;                         // 全部转发 solveIK
  const std::vector<std::string>& getJointNames() const override;
  const std::vector<std::string>& getLinkNames() const override;
private:
  bool solveIK(...) const;                    // 归一化 IK：控制器→校验→KDL
  bool callControllerFK(joint_rad, pose&) const;
  bool callControllerIK(pose, seed_rad, sol_rad&) const;
  bool kdlFK(...) const;  bool kdlIK(...) const;
  rclcpp::Client<PositiveKin>::SharedPtr fk_client_;
  rclcpp::Client<InverseKin>::SharedPtr ik_client_;
  std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> kdl_loader_;
  std::unique_ptr<kinematics::KinematicsBase> kdl_plugin_;
  // 配置（lookupParam 读）
  std::string fk_service_name_, ik_service_name_, user_frame_="0", tool_frame_="0";
  double service_timeout_sec_=1.0, position_scale_=0.001, angle_scale_=M_PI/180.0;
  bool use_kdl_fallback_=true;
  enum { ROTATION_VECTOR, RPY } orient_conv_=ROTATION_VECTOR;
};
PLUGINLIB_EXPORT_CLASS(dobot_kinematics_plugin::DobotKinematicsPlugin, kinematics::KinematicsBase)
```

### 关键换算（在 `callControllerFK`/`callControllerIK` 内）
- **FK 请求**：`req->jN = joint_rad[N] / angle_scale_`（rad→deg）；`user/tool` 字符串。
- **FK 响应**：`parse_brace_list(robot_return)`→6 个数；`position *= position_scale_`(mm→m)；`rx,ry,rz`(deg 旋转向量)→`Eigen::Quaterniond`。
- **IK 请求**：`req->x/y/z = pose.pos / position_scale_`(m→mm)；四元数→旋转向量(deg)；`use_joint_near="1"`、`joint_near` 传 seed(deg)（格式实测确认，见验证）。
- **IK 响应**：`parse_brace_list`→6 关节角(deg)→`* angle_scale_`→rad；做角度归一化。
- **兜底触发**：`wait_for_service` 超时 / `future.wait_for` 超时 / `res!=0` / 解析失败 / `solution_callback` 拒绝 → `kdl_plugin_` 委托。

### 复用的既有代码（只读引用，不改）
- `dobot_bringup_v4/src/cr_robot_ros2.cpp`（服务注册、服务名拼接）
- `dobot_bringup_v4/src/parseTool.cpp`（请求字符串格式权威依据）
- `dobot_bringup_v4/src/command.cpp`（返回字符串解析、单位换算依据）
- `dobot_msgs_v4/srv/*.srv`

---

## 四、改造 12 个 moveit 包

### 4.1 `config/kinematics.yaml`（每个包改一份，组名替换）
以 `cr3af_moveit/config/kinematics.yaml` 为模板：
```yaml
cr3af_group:
  kinematics_solver: dobot_kinematics_plugin/DobotKinematicsPlugin
  kinematics_solver_search_resolution: 0.005      # 保留原值
  kinematics_solver_timeout: 0.005                  # 保留原值（cr30h 原为 0.05）
  fk_service_name: /dobot_bringup_ros2/srv/PositiveKin
  ik_service_name: /dobot_bringup_ros2/srv/InverseKin
  service_timeout: 1.0
  use_kdl_fallback: true
  position_unit: mm
  angle_unit: deg
  orientation_convention: rotation_vector   # rotation_vector | rpy
  user_frame: "0"
  tool_frame: "0"
```
12 个文件：`{cr3af,cr5af,cr10af,cr20af,cr30h,cr3,cr5,cr7,cr10,cr12,cr16,cr20}_moveit/config/kinematics.yaml`，组名 `{...}_group`。多机器人部署时 `fk/ik_service_name` 加 `/<robot_node_name>` 前缀。

### 4.2 每个 `package.xml` 加一行（共 12 个）
在 `<exec_depend>moveit_kinematics</exec_depend>` 后加：
```xml
<exec_depend>dobot_kinematics_plugin</exec_depend>
```

### 4.3 launch / SRDF / joint_limits / ros2_controllers / moveit_controllers
**全部不动**。`MoveItConfigsBuilder` 自动把 `kinematics.yaml` 加载到 `robot_description_kinematics.<group>.*` 参数下，插件经 `lookupParam` 读取。

---

## 五、实施顺序

1. 建 `dobot_kinematics_plugin/` 目录与 7 个文件骨架（package.xml、CMakeLists、xml、hpp、cpp、string_parser.hpp、units.hpp）。
2. 写 `string_parser.hpp` + `units.hpp`（header-only，无依赖）。
3. 写 `dobot_kinematics_plugin.hpp` + `.cpp`：initialize（含 KDL 子插件加载）→ getPositionFK → solveIK + 4 重载 → getJointNames/getLinkNames → PLUGINLIB_EXPORT_CLASS。
4. `colcon build --packages-select dobot_kinematics_plugin`（依赖 dobot_msgs_v4 已存在）。
5. 试点改 `cr3af_moveit` 的 kinematics.yaml + package.xml，跑验证。
6. 验证通过后批量改其余 11 个包。

---

## 六、验证（端到端）

### 6.1 构建前 API 核对（在已装 MoveIt2 Humble 环境）
```bash
ros2 pkg list | grep moveit_kinematics
find /opt/ros/humble -name "kdl_kinematics_plugin.xml"   # 确认 base_class_type=kinematics::KinematicsBase 且导出到 moveit_core 命名空间
```

### 6.2 实测服务返回字符串格式（决定 parser）
```bash
ros2 service call /dobot_bringup_ros2/srv/PositiveKin dobot_msgs_v4/srv/PositiveKin \
  "{j1: 0, j2: 0, j3: 90, j4: 0, j5: 90, j6: 0, user: '0', tool: '0'}"
# 期望 robot_return 形如 "{x,y,z,rx,ry,rz}"，res=0；核对是否带前导 "0{...}"
# 同样测 InverseKin，核对 joint_near 字段是否被接受
```

### 6.3 离线兜底验证（无真机）
```bash
# 不启 bringup
ros2 launch cr3af_moveit dobot_moveit.launch.py
# RViz 拖目标→Plan：日志应出 "Controller FK failed, falling back to KDL"，规划仍成功
```

### 6.4 真机验证（控制器真实 DH）
```bash
# T1: ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py   (设 IP_address/DOBOT_TYPE)
# T2: ros2 launch cr3af_moveit dobot_moveit.launch.py
```
对比：用 6.2 的 service call 取某关节构型的控制器 FK 位姿 A；RViz 把机器人拖到同构型读 MotionPlanning 当前位姿 B。若 A≈B（位置 mm 级、姿态 1° 内）→ 插件确实走控制器真实 DH。

### 6.5 兜底切换验证
规划成功后 kill bringup 节点，再 Plan → 应出 KDL fallback 警告且仍能规划；重启 bringup → 自动恢复控制器路径。

---

## 七、需在构建/运行时确认的点（Flag）

1. **KDL 子插件命名空间**：`pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase")` 第一参应为 `moveit_core`（Humble 上 `moveit_kinematics` 把 KDL 插件导出到该命名空间）。6.1 步核对。
2. **`joint_near` 字段格式**：`parseTool.cpp:227` 拼成 `jointNear=<原样字符串>`，但控制器端解析规则仓库内不可见。6.2 步实测；若不接受 `{j1,...,j6}` 则 `use_joint_near="0"` 让控制器自由解。
3. **user/tool 字符串值**：默认 `"0"`，6.2 步确认控制器接受。
4. **RPY 顺序**（仅当用户切到 `orientation_convention: rpy` 时生效）：控制器 RPY 顺序需运行时确认，默认不动 RPY 分支。
5. **executor 线程模型**：依赖 move_group 默认 MultiThreadedExecutor。若用户改单线程需改用独立 Node+独立 spin 线程。
6. **性能**：每次 IK = 1 次 TCP 往返真机（5~50ms），MoveIt 规划上百次 IK 调用 → 规划可能耗时数秒。`service_timeout` 调小可让超时快速降级 KDL，但真机路径仍慢。这是用真实 DH 的代价，已与用户确认接受。
7. **CR 系列以外机型**（Nova/ME6 等）控制器协议不同，不覆盖，保持 KDL。

---

## 八、改动文件总览

**新建（1 个包，7 文件）**：
- `dobot_kinematics_plugin/package.xml`
- `dobot_kinematics_plugin/CMakeLists.txt`
- `dobot_kinematics_plugin/dobot_kinematics_plugin.xml`
- `dobot_kinematics_plugin/include/dobot_kinematics_plugin/dobot_kinematics_plugin.hpp`
- `dobot_kinematics_plugin/src/dobot_kinematics_plugin.cpp`
- `dobot_kinematics_plugin/include/dobot_kinematics_plugin/string_parser.hpp`
- `dobot_kinematics_plugin/include/dobot_kinematics_plugin/units.hpp`

**修改（12 包 × 2 文件 = 24 文件）**：
- 12 个 `*_moveit/config/kinematics.yaml`（换 solver + 加自定义参数）
- 12 个 `*_moveit/package.xml`（加 `<exec_depend>dobot_kinematics_plugin</exec_depend>`）
```

---
*计划完成。等待用户确认后开始实施。*
