<a id="api-reference"></a>

#### WARNING
Before running any code examples, make sure to start the robot first by executing either:

# API Reference

<a id="kuavo-strategy"></a>

#### WARNING
Before running any code examples, make sure to start the robot first by executing either:

# Kuavo Strategy 策略模块

## 基础策略接口

### *class* kuavo_humanoid_sdk.kuavo_strategy.kuavo_strategy.KuavoRobotStrategyBase(robot: KuavoRobot, robot_state: KuavoRobotState, robot_tools: KuavoRobotTools, robot_vision: KuavoRobotVision)

Bases: `ABC`

Kuavo机器人策略基础类，提供策略执行的抽象接口

#### *abstract* arm_move_to_target(target_pose, \*\*kwargs)

手臂移动到特定的位置(闭环)

* **Parameters:**
  * **target_pose** – 目标位置姿态
  * **\*\*kwargs** – 其他参数
* **Returns:**
  是否成功移动到目标位置
* **Return type:**
  bool

#### *abstract* head_find_target(target_info, \*\*kwargs)

寻找特定ID的目标

* **Parameters:**
  * **target_id** – 目标的ID标识
  * **\*\*kwargs** – 其他参数
* **Returns:**
  是否成功找到目标
* **Return type:**
  bool

#### *abstract* walk_approach_target(target_info, target_distance=0.5, \*\*kwargs)

走/接近特定的目标到指定距离

* **Parameters:**
  * **target_id** – 目标的ID标识
  * **target_distance** – 与目标的期望距离(米)
  * **\*\*kwargs** – 其他参数
* **Returns:**
  是否成功接近目标
* **Return type:**
  bool

## 箱子抓取策略

### 箱子信息数据结构

### *class* kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy.BoxInfo(pose: KuavoPose, size: Tuple[float, float, float] = (0.3, 0.2, 0.15), mass: float = 1.0)

Bases: `object`

箱子信息数据类

描述箱子的位置、尺寸和质量信息，用于箱子抓取策略

#### pose

箱子的位姿信息

* **Type:**
  KuavoPose

#### size

箱子的尺寸 (长, 宽, 高) 单位: 米

* **Type:**
  Tuple[float, float, float]

#### mass

箱子的质量 单位: 千克

* **Type:**
  float

#### mass *: float* *= 1.0*

#### pose *: KuavoPose*

#### size *: Tuple[float, float, float]* *= (0.3, 0.2, 0.15)*

### 箱子抓取策略类

### *class* kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy.KuavoGraspBox(robot: KuavoRobot, robot_state: KuavoRobotState, robot_tools: KuavoRobotTools, robot_vision: KuavoRobotVision)

Bases: [`KuavoRobotStrategyBase`](#kuavo_humanoid_sdk.kuavo_strategy.kuavo_strategy.KuavoRobotStrategyBase)

箱子抓取策略类，继承自基础策略类

#### arm_move_to_target(target_pose: KuavoPose, approach_speed=0.15, \*\*kwargs)

手臂移动到目标位置

* **Parameters:**
  * **target_pose** – 目标位置，可以是KuavoPose对象或包含position的字典
  * **approach_speed** – 接近速度(米/秒)
* **Returns:**
  是否成功移动到目标位置
* **Return type:**
  bool

#### arm_transport_target_down(target_info: BoxInfo, arm_mode='manipulation_mpc')

实现手臂放下箱子的功能

* **Parameters:**
  * **target_info** – 目标放置位置信息
  * **arm_mode** – 手臂控制模式
* **Returns:**
  是否成功放下目标
* **Return type:**
  bool

#### arm_transport_target_up(target_info: BoxInfo, arm_mode='manipulation_mpc')

实现手臂搬起箱子的功能

* **Parameters:**
  * **target_info** – 目标信息，包含位置、尺寸等
  * **arm_mode** – 手臂控制模式
* **Returns:**
  是否成功搬起目标
* **Return type:**
  bool

#### head_find_target(target_info: AprilTagData, max_search_time=None, search_pattern='rotate_head', \*\*kwargs)

使用头部旋转寻找AprilTag目标

* **Parameters:**
  * **target_info** – AprilTag的信息
  * **max_search_time** – 最大搜索时间(秒)，如果为None则使用默认值
  * **search_pattern** – 搜索模式，”rotate_head”或”rotate_body”
* **Returns:**
  是否成功找到目标
* **Return type:**
  bool

#### walk_approach_target(target_info: AprilTagData, target_distance=0.5, approach_speed=0.15, \*\*kwargs)

走路接近AprilTag目标

* **Parameters:**
  * **target_info** – AprilTag的信息
  * **target_distance** – 与目标的期望距离(米)
  * **approach_speed** – 接近速度(米/秒)
* **Returns:**
  是否成功接近目标
* **Return type:**
  bool

## 使用示例

以下是一个使用箱子抓取策略的基本示例:

```python
import time
from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, AprilTagData, PoseQuaternion
from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox, BoxInfo

# 初始化机器人及相关组件
robot = KuavoRobot()
robot_state = KuavoRobotState()
robot_tools = KuavoRobotTools()
robot_vision = KuavoRobotVision()

# 初始化箱子抓取策略
grasp_strategy = KuavoGraspBox(robot, robot_state, robot_tools, robot_vision)

# 创建AprilTag数据对象
target_april_tag = AprilTagData(
    id=[42],  # AprilTag ID
    size=[0.1],  # AprilTag 标签尺寸
    pose=[PoseQuaternion(
        position=(0.5, 0.0, 0.4),  # 位置
        orientation=(0.0, 0.0, 0.0, 1.0)  # 四元数方向
    )]
)

# 使用头部寻找目标
find_success = grasp_strategy.head_find_target(
    target_april_tag,
    max_search_time=15.0,
    search_pattern="rotate_head"
)

if find_success:
    print("目标找到成功!")
```
