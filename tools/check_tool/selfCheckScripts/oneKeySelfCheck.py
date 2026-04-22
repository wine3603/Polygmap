import subprocess
import sys
import threading
import signal
import os
import rospy
import select

roslaunch_running = False
h12_msg = False

h12_msg_valid = False
h12_msg = None
current_process = None

# 定义要打印的日志信息列表
target_logs = [
    "IMU 总采样次数: ",
    "数据读取失败次数: ",
    "数据读取成功率: ", 
    "Yaw角漂移检测失败！漂移量：",
    "可能原因：IMU未校准、陀螺仪零偏过大或设备晃动",
    "Yaw角稳定性检测通过！漂移量：",
    "当前IMU（HIPNUC）不支持时间戳，无法统计更新频率.",
    "IMU更新频率统计:",
    "加速度计: 均值=",
    "陀螺仪:   均值=",
    "四元数:   均值=",
    "获取IMU数据更新频率失败!!!",
    "IMU 初始化成功!!!",
    "IMU 初始化失败!!!",
    "末端执行器(灵巧手)初始化成功!!!",
    "末端执行器(灵巧手)初始化失败!!!",
    "电机初始化成功!!!",
    "电机初始化失败!!!",
    "测试手臂电机>>",
    "手臂关节",
    "测试手臂电机失败, 请用户确认相应关节是否使能!!!",
    "测试手臂电机成功!!!",
    "测试腿部电机>>",
    "测试腿部电机失败!!!",
    "测试手臂电机成功!!!",
    "测试手臂电机失败!!!",
    "当前cali位置是否正确，[y/n] 输入[y]进入到准备姿态(执行完后可以输入[x]退出程序)",
    "输入x退出程序："
]

# 获取当前脚本所在的目录
current_dir = os.path.dirname(os.path.abspath(__file__))
# 构建 LidarCheck.py 的绝对路径
OneKeyCheckSalveComputer_path = os.path.join(current_dir, 'OneKeyCheckSlaveComputer.py')

# 反推kuavo-ros-control的路径
kuavo_ros_control_path = os.path.abspath(os.path.join(current_dir, "../../.."))
# 打印kuavo-ros-control的路径
print(f"kuavo-ros-control的路径为: {kuavo_ros_control_path}")

# 获取当前Python脚本所在目录
shell_script_path = os.path.join(current_dir, "compiling_dependent_lib.sh")

# 计算相对路径
launch_path = os.path.join(kuavo_ros_control_path, "hardware_node hardwareSelfCheck.launch")

def print_colored_text(text, color="green", bold=False):
    color_codes = {
        "green": ("32", "92"),
        "yellow": ("33", "93"),
        "red": ("31", "91")
    }
    base_code = "\033["
    if bold:
        base_code += "1;"
    color_code = color_codes.get(color, ("32", "92"))[1]
    print(f"{base_code}{color_code}m{text}\033[0m")

def h12_topic_callback(msg):
    global h12_msg
    global h12_msg_valid
    if h12_msg:
        h12_msg = msg
        channels = msg.channels
        has_non_zero = any(channel != 0 for channel in channels)
        if has_non_zero:
            h12_msg_valid = True
        # else:
            # print(f"收到 /h12pro_channel 话题消息，但消息中的数据全为 0: {msg}")

def h12_remote_control_detection():
    global h12_msg
    global h12_msg_valid

    workspace_path  = "../../../devel/lib/python3/dist-packages"
    if workspace_path not in sys.path:
        sys.path.append(workspace_path)

    # 检查并赋予执行权限（关键修复）
    if not os.access(shell_script_path, os.X_OK):
        os.chmod(shell_script_path, 0o755)
        # print(f"已为 {shell_script_path} 添加执行权限")

    try:
        result = os.system(shell_script_path)
        if result != 0:
            print(f"Error occurred while running the shell script. Return code: {result}")
        from h12pro_controller_node.msg import h12proRemoteControllerChannel
        print("成功导入消息模块")
    except ImportError as e:
        print(f"导入模块时出错: {e}")
        return

    rospy.Subscriber('/h12pro_channel', h12proRemoteControllerChannel, h12_topic_callback)
    start_time = rospy.Time.now()
    h12_msg = True
    while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 0.5:
        
        rospy.rostime.wallsleep(0.1)
    
    h12_msg = False
    if h12_msg_valid:
        print_colored_text(f"收到 /h12pro_channel 话题消息成功", color="green", bold=True)
    else:
        print_colored_text(f"收到 /h12pro_channel 话题消息异常", color="red", bold=True)

def input_thread(process):
    global roslaunch_running
    while process and process.poll() is None:
        try:
            user_input = input().strip()  # 获取输入并清理空格
            if process.stdin:
                # 直接发送原始输入内容
                process.stdin.write(user_input + "\n")
                process.stdin.flush()
                
                # 如果输入x则退出
                if user_input.lower() == 'x':
                    roslaunch_running = False
                    break
                    
        except (BrokenPipeError, AttributeError):
            break
        except Exception as e:
            print(f"输入线程错误: {e}", file=sys.stderr)
            break

def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    global current_process, roslaunch_running
    print("\n收到Ctrl+C, 正在关闭程序...")
    if current_process and current_process.poll() is None:
        # 发送退出命令
        if current_process.stdin:
            current_process.stdin.write('x\n')  # 发送退出命令
            current_process.stdin.flush()
        current_process.terminate()  # 发送SIGTERM
        try:
            current_process.wait(timeout=3)  # 等待3秒
        except subprocess.TimeoutExpired:
            current_process.kill()  # 强制终止
    roslaunch_running = False
    rospy.signal_shutdown("Received Ctrl+C")
    sys.exit(0)

def register_signal_handler(process):
    """注册信号处理函数"""
    global current_process
    current_process = process
    handler = lambda sig, frame: signal_handler(sig, frame)
    signal.signal(signal.SIGINT, handler)

def check_package_compiled(package_names):
    """检查指定的ROS包是否已经编译"""
    # 获取当前脚本所在目录
    current_script_dir = os.path.dirname(os.path.abspath(__file__))
    # 假设 ROS 工作空间在当前脚本目录的上两级目录
    workspace_root = os.path.abspath(os.path.join(current_script_dir, '..', '..', '..'))
    workspace_setup = os.path.join(workspace_root, 'devel', 'setup.bash')
    if not os.path.exists(workspace_setup):
        print(f"devel路径不存在...")
        print(f"workspace_setup的路径为: {workspace_setup}")
        return False
    # 检查包对应的目录是否存在于devel空间中
    for package_name in package_names:
        # 检查包对应的目录是否存在于devel空间中
        package_devel_path = os.path.join(os.path.dirname(workspace_setup), "lib", package_name)
        if not os.path.exists(package_devel_path):
            print_colored_text(f"ROS包 {', '.join(package_names)} 中存在未编译的包，请先编译这些包。", color="red", bold=True)
            return False
    return True

def run_roslaunch():
    global roslaunch_running
    global target_logs

    package_name_list = [
                    "hardware_node"
                    ]
    if not check_package_compiled(package_name_list):
        return 1
    try:
        # 构造完整的启动命令
        full_command = (
            "source /opt/ros/noetic/setup.bash && "
            f"source {kuavo_ros_control_path}/devel/setup.bash && "
            f"roslaunch hardware_node hardwareSelfCheck.launch"
        )

        # 定义输出文件路径
        output_file_path = "roslaunch_output.log"

        # 打开文件以写入模式
        with open(output_file_path, 'w') as output_file:
            # 启动roslaunch
            process = subprocess.Popen(
                ['bash', '-c', full_command],
                stdin=subprocess.PIPE,  # 允许输入
                stdout=subprocess.PIPE,  # 捕获标准输出
                stderr=subprocess.STDOUT,  # 将错误输出合并到标准输出
                text=True,
                bufsize=1,  # 行缓冲
                universal_newlines=True,
            )

            # 注册信号处理
            register_signal_handler(process)

            # 生成支持点击的超链接
            clickable_link = f"\x1b]8;;file://{output_file_path}\x1b\\{output_file_path}\x1b]8;;\x1b\\"
            print(f"ROS launch已启动，输出已保存到当前路径下 {clickable_link}，你可以点击该链接查看详细信息。按Ctrl+C退出...")
            roslaunch_running = True

            # 启动输入线程
            thread = threading.Thread(target=input_thread, args=(process,), daemon=True)
            thread.start()

            # 监控标准输出
            while process.poll() is None:
                rlist, _, _ = select.select([process.stdout], [], [], 0.1)
                if rlist:
                    line = process.stdout.readline()
                    # 检查是否为目标日志信息
                    for target_log in target_logs:
                        if target_log in line:
                            print(line.strip())
                    # 将所有输出写入日志文件
                    output_file.write(line)
                    output_file.flush()

            # 主线程等待进程结束
            process.wait()

    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 确保进程已终止
        if process and process.poll() is None:
            process.terminate()
            process.wait(timeout=2)
            if process.poll() is None:
                process.kill()


if __name__ == "__main__":
    run_flag = True
    rospy.init_node('main_node', anonymous=True)
    while run_flag:
        if not roslaunch_running:
            print("\n请选择要执行的操作:")
            print("1. 下位机检测(启动hardwareSelfCheck.cc程序)")
            print("2. 上位机检测")
            print("3. h12遥控器信号检测")
            print("0. 退出")
            choice = input("输入选项: ")

            if choice == '1':
                run_roslaunch()  # 启动roslaunch（假设其内部有独立的退出逻辑）
            elif choice == '2':
                subprocess.run(['python3', OneKeyCheckSalveComputer_path])
            elif choice == '3':
                h12_remote_control_detection()
            elif choice == '0':
                print("退出程序。")
                run_flag = False
            else:
                print_colored_text(f"无效的选项，请重新输入...", color="yellow", bold=True)