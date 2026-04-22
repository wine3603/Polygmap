import os
import yaml
import time
from SimpleSDK import RUIWOTools

# 速度相关参数，方便调整
MOTION_DURATION = 2 # 每个动作的执行时间（秒）
POS_KP = 30
POS_KD = 5

# 500Hz 的更新频率
UPDATE_FREQUENCY = 50  # Hz

UPDATE_INTERVAL = 1 / UPDATE_FREQUENCY  # 秒

# 定义零点文件路径
def get_zero_path():
    return '/home/lab/.config/lejuconfig/arms_zero.yaml'

# 读取零点位置
def read_zero_positions():
    zeros_path = get_zero_path()
    if os.path.exists(zeros_path):
        with open(zeros_path, 'r') as file:
            zeros_config = yaml.safe_load(file)
        return zeros_config['arms_zero_position'][:12]  # 只取前 12 个值
    else:
        print("[RUIWO motor]:Warning: zero_position file does not exist, will use 0 as zero value.")
        return [0.0] * 12  # 返回长度为 12 的列表

# 获取用户输入的测试时长
def get_test_duration():
    # 计算完成一个完整动作周期所需的时间
    cycle_time = len(base_actions) * MOTION_DURATION
    while True:
        try:
            duration = int(input(f"\n请输入测试时长（大于 {cycle_time:.1f} 秒）："))
            if duration >= cycle_time:
                return duration
            else:
                print(f"输入的时长小于一个完整动作周期的时间（大于 {cycle_time:.1f} 秒），请重新输入。")
        except ValueError:
            print("输入无效，请输入一个整数。")

# 获取当前时间戳
def get_timestamp():
    return time.strftime("%H:%M:%S", time.localtime()) + f".{int(time.time() % 1 * 1000):03d}"

# 读取电机正反转配置
def read_motor_reverse_config():
    config_path = '/home/lab/.config/lejuconfig/config.yaml'
    if os.path.exists(config_path):
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        return config.get('negtive_address', [])
    else:
        print("[RUIWO motor]:Warning: config.yaml file does not exist, no motor reverse config will be applied.")
        return []

# 读取关节 ID 列表
def read_joint_ids():
    config_path = '/home/lab/.config/lejuconfig/config.yaml'
    if os.path.exists(config_path):
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        return list(config.get('address', {}).values())[:12]  # 只取前 12 个值
    else:
        print("[RUIWO motor]:Warning: config.yaml file does not exist, using default joint IDs.")
        return [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C]

# 读取机器人的版本号
def get_robot_version():
    # 获取用户的主目录
    home_dir = os.path.expanduser('/home/lab/')
    bashrc_path = os.path.join(home_dir, '.bashrc')

    if os.path.exists(bashrc_path):
        with open(bashrc_path, 'r') as file:
            lines = file.readlines()
        for line in reversed(lines):  # 从文件末尾开始读取
            line = line.strip()  # 去除行首和行尾的空白字符
            if line.startswith("export ROBOT_VERSION=") and "#" not in line:
                version = line.split("=")[1].strip()
                print(f"---------- 检测到 ROBOT_VERSION = {version} ----------")
                if version in ["41", "42", "45"]:
                    return version
                else:
                    print(f"[RUIWO motor]:Warning: ROBOT_VERSION '{version}' 不是有效值。")
                    break
    print("[RUIWO motor]:Warning: ROBOT_VERSION 未找到或无效，需要手动输入。")
    while True:
        version = input("请输入机器人版本号（41 42 或 45）：").strip()
        if version in ["41", "42", "45"]:
            return version
        else:
            print("输入的版本号无效，请输入 41 42 或 45。")
    return version  # 这行永远不会执行，但因完整性保留

def check_motor_status(joint_ids):
    disabled_motors = []  # 用于记录失能的电机ID
    for dev_id in joint_ids:
        state = ruiwo.enter_motor_state(dev_id)
        if isinstance(state, list):
            # 检查故障码是否为15
            if state[-2] == 15:  # 倒数第二个元素是故障码
                disabled_motors.append(dev_id)  # 记录失能的电机ID
        else:
            print(f"\033[91m电机 {dev_id} 状态获取失败！\033[0m")
            return False  # 状态获取失败，直接返回False
    # 如果有失能的电机，输出失能的电机列表
    if disabled_motors:
        print(f"\033[91m以下电机失能：{disabled_motors}\033[0m")
        return False  # 返回False表示有电机失能
    return True  # 所有电机状态正常

# 定义长手和短手的动作
long_arm_actions = [
    [0.23, 0.00, 0.00, 0.00, 0.00, -0.24],
    [1.30, 1.00, -1.40, 1.30, 0.40, -0.60],
    [2.00, -0.50, -0.30, 1.50, 0.80, 0.70],
    [1.10, -2.00, -2.30, -1.50, 0.00, -0.24],
    [0.23, -0.40, -1.30, 1.00, -0.80, 0.70],
    [1.25, 0.00, -1.90, 0.70, 0.00, -0.60],
    [0.23, 0.00, 0.00, 0.00, 0.00, -0.24]
]

short_arm_actions = [
    [0.00, 0.00, 0.00, 0.00, -0.10, 0.00],
    [0.85, -1.30, -0.20, 1.40, 1.30, -1.30],
    [1.90, 0.40, -0.50, -1.40, 0.52, -0.80],
    [1.31, 1.30, -0.90, 0.00, -0.10, 1.00],
    [1.90, 0.31, -1.30, -1.40, -0.72, 0.00],
    [1.40, 0.90, -0.60, 0.90, -1.50, -1.00],
    [0.40, 0.20, 0.00, 0.20, -0.10, 0.00]
]

# 读取机器人的版本号
robot_version = get_robot_version()

# 根据版本号选择动作
if robot_version == "41" or robot_version == "42":
    base_actions = short_arm_actions
    print("现在将要执行 KUAVO 机器的短手版。")
elif robot_version == "45":
    base_actions = long_arm_actions
    print("现在将要执行 KUAVO 机器的长手版。")
else:
    print(f"[RUIWO motor]:Error: 未知的 ROBOT_VERSION '{robot_version}'，程序退出。")
    exit(1)

# 等待用户确认
input("请确认机器型号无误后，按回车键继续...")

# 读取关节 ID 列表
joint_ids = read_joint_ids()

# 读取零点位置
zero_positions = read_zero_positions()

# 读取电机正反转配置
reverse_addresses = read_motor_reverse_config()

# 生成包含左右手的完整动作序列
full_base_actions = []
left_joint_ids = joint_ids[:6]
right_joint_ids = joint_ids[6:]
for action in base_actions:
    left_action = action
    right_action = []
    for i in range(len(left_action)):
        left_id = left_joint_ids[i]
        right_id = right_joint_ids[i]
        # 判断两个关节在反向列表中的情况
        if (left_id in reverse_addresses) ^ (right_id in reverse_addresses):
            # 一个在反向列表，一个不在，说明是镜像安装，右手取反
            right_action.append(-action[i])
        elif (left_id in reverse_addresses) and (right_id in reverse_addresses):
            # 两个都在反向列表，说明是倒着安装，右手取反
            right_action.append(-action[i])
        else:
            # 两个都不在反向列表，右手取反以实现对称运动
            right_action.append(-action[i])
    full_action = left_action + right_action
    full_base_actions.append(full_action)

# 创建对象
ruiwo = RUIWOTools()

# 打开CAN总线
open_canbus = ruiwo.open_canbus()
if not open_canbus:
    print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")
    exit(1)
print("[RUIWO motor]:Canbus status:","[",open_canbus,"]")

# 使能所有关节电机
enable_all_success = True  # 用于标记是否所有电机都使能成功
for dev_id in joint_ids:
    state = ruiwo.enter_motor_state(dev_id)
    if isinstance(state, list):
        print(f"[RUIWO motor]:ID: {dev_id} Enable:  [Succeed]")
    else:
        print(f"[RUIWO motor]:ID: {dev_id} Enable:  [{state}]")
        enable_all_success = False  # 只要有一个电机使能失败，就标记为失败
if not enable_all_success:
    print("有电机使能失败，程序退出。")
    exit(1)  # 退出程序

# 获取用户输入的测试时长
test_duration = get_test_duration()
start_time = time.perf_counter()

# 计算一个完整动作周期所需的时间
cycle_time = len(base_actions) * MOTION_DURATION

while True:
    current_time = time.perf_counter()
    elapsed_time = current_time - start_time
    remaining_time = test_duration - elapsed_time

    # 输出剩余时间
    print(f"{get_timestamp()} 总剩余时间：{remaining_time:.2f} 秒")

    # 检查是否开始新的完整周期
    if elapsed_time % cycle_time < MOTION_DURATION:
        # 判断剩余时间是否足够完成一个完整周期
        if remaining_time < cycle_time:
            print(f"{get_timestamp()} 剩余时间不足完成一个动作周期，提前结束。")
            break

        # 检查所有电机状态，是否出现失能情况
        if not check_motor_status(joint_ids):
            print(f"\033[91m检测到电机失能，程序停止！\033[0m")
            # 失能所有关节电机
            for dev_id in joint_ids:
                state = ruiwo.enter_reset_state(dev_id)
                if isinstance(state, list):
                    print(f"{get_timestamp()} [RUIWO motor]:ID: {dev_id} Disable:  [Succeed]")
                else:
                    print(f"{get_timestamp()} [RUIWO motor]:ID: {dev_id} Disable:  [{state}]")
            # 关闭CAN总线
            close_canbus = ruiwo.close_canbus()
            if close_canbus:
                print(f"{get_timestamp()} [RUIWO motor]:Canbus status: [Close]")
            exit(1)  # 退出程序
        print(f"{get_timestamp()} 开始下一轮动作...")

    if elapsed_time >= test_duration:
        break

    # 根据实际时间计算当前应该执行的关键帧索引
    current_frame_index = int(elapsed_time // MOTION_DURATION) % len(full_base_actions)
    next_frame_index = (current_frame_index + 1) % len(full_base_actions)

    current_positions = full_base_actions[current_frame_index]
    target_positions = full_base_actions[next_frame_index]

    # 检查长度是否匹配
    if len(current_positions) != len(joint_ids) or len(target_positions) != len(joint_ids):
        raise ValueError(f"动作 {current_frame_index + 1} 的位置列表长度不匹配。当前长度: {len(current_positions)}，目标长度: {len(target_positions)}，关节数量: {len(joint_ids)}")

    steps = MOTION_DURATION * UPDATE_FREQUENCY  # MOTION_DURATION 秒内发送的步数
    step_start_time = elapsed_time % MOTION_DURATION

    for step in range(int(steps)):
        loop_start = time.perf_counter()
        for joint_index, dev_id in enumerate(joint_ids):
            # 计算当前位置到目标位置的插值
            interpolated_pos = current_positions[joint_index] + (target_positions[joint_index] - current_positions[joint_index]) * (step / steps)
            zero_position = zero_positions[joint_index]
            compensated_pos = interpolated_pos + zero_position  # 应用零点补偿
            state = ruiwo.run_ptm_mode(dev_id, compensated_pos, 0, POS_KP, POS_KD, 0)
            if isinstance(state, list):
                pass
            else:
                print(f"{get_timestamp()} ID: {dev_id} Run ptm mode:  [{state}]")

        loop_end = time.perf_counter()
        elapsed_time = loop_end - loop_start
        remaining_time = UPDATE_INTERVAL - elapsed_time
        if remaining_time > 0:
            time.sleep(remaining_time)

    print(f"{get_timestamp()} 动作 {current_frame_index + 1} 执行完成，开始向位置 {next_frame_index + 1} 运动")

# 失能所有关节电机
for dev_id in joint_ids:
    state = ruiwo.enter_reset_state(dev_id)
    if isinstance(state, list):
        print(f"{get_timestamp()} [RUIWO motor]:ID: {dev_id} Disable:  [Succeed]")
    else:
        print(f"{get_timestamp()} [RUIWO motor]:ID: {dev_id} Disable:  [{state}]")

# 关闭CAN总线
close_canbus = ruiwo.close_canbus()
if close_canbus:
    print(f"{get_timestamp()} [RUIWO motor]:Canbus status: [Close]")