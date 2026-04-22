#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
import rospkg
import numpy as np
import copy
import cv2
import os
import yaml

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from modules.mapmanager import Polygon3D, MapManager
from modules.image_processes_cupy import process_polygon, project_points_to_plane, depth_to_point_cloud_region, fit_plane_ransac, anisotropic_diffusion, depth_to_normals, add_border_conditionally
from modules.ros_node import MappingNode

def nothing(x):
    """Placeholder function for trackbar."""
    pass

def load_config(config_file):
    """Load configuration from a YAML file."""  # 文档字符串：说明函数用途
    with open(config_file, 'r') as file:  # 安全打开文件（自动处理文件关闭）
        config = yaml.safe_load(file)     # 使用 PyYAML 的 safe_load 解析 YAML
    return config                        # 返回配置字典

def create_dataset_folder(base_path):
    """Create a unique dataset folder and subfolders for RGB and depth images."""
    dataset_index = 1  # 起始索引
    while True:  # 循环直到找到可用的文件夹名
        dataset_name = f"dataset_{dataset_index}"  # 生成文件夹名（如 dataset_1）
        dataset_path = os.path.join(base_path, dataset_name)  # 拼接完整路径
        if not os.path.exists(dataset_path):  # 检查路径是否已存在
            # 创建文件夹及其子目录（exist_ok=True 避免竞态条件报错）
            os.makedirs(os.path.join(dataset_path, "rgb"), exist_ok=True)
            os.makedirs(os.path.join(dataset_path, "depth"), exist_ok=True)
            return dataset_path  # 返回新创建的路径
        dataset_index += 1  # 索引递增继续尝试

def set_cv_config(thresh1=50, thresh2=100, num_iter=60, kappa=134, gamma=2):
    """Initialize OpenCV windows and trackbars for image processing parameters."""
    # 创建第一个窗口（Canny 边缘检测）
    cv2.namedWindow('Filtered Image with Largest Contour')
    # 添加控制 Canny 阈值的滑动条
    cv2.createTrackbar('Canny thresh1', 'Filtered Image with Largest Contour', thresh1, max(thresh1, 1000), nothing)
    cv2.createTrackbar('Canny thresh2', 'Filtered Image with Largest Contour', thresh2, max(thresh2, 1000), nothing)

    # 创建第二个窗口（深度图平滑）
    cv2.namedWindow('smoothed_depth_img')
    # 添加控制平滑参数的滑动条
    cv2.createTrackbar('num_iter', 'smoothed_depth_img', num_iter, max(num_iter, 360), nothing)
    cv2.createTrackbar('kappa', 'smoothed_depth_img', kappa, max(kappa, 500), nothing)
    cv2.createTrackbar('gamma', 'smoothed_depth_img', gamma, max(gamma, 80), nothing)

l515_color_image = None
l515_depth_image = None
def rgb_callback(msg):
    global l515_color_image
    bridge = CvBridge()
    try:
        # 将ROS的图像消息转换为OpenCV图像
        l515_color_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        print(f"Received RGB frame: {l515_color_image.shape}")
    except Exception as e:
        print(f"Error converting RGB image: {e}")

def depth_callback(msg):
    global l515_depth_image
    bridge = CvBridge()
    try:
        # 将深度图像消息转换为NumPy数组
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        l515_depth_image = depth_image * 4
        print(f"Received Depth frame: {l515_depth_image.shape}")
    except Exception as e:
        print(f"Error converting Depth image: {e}")

def main():
    # Initialize ROS package manager
    # 读取配置文件
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('polygon_mapping')  # Get package path
    config_path = os.path.join(package_path, 'config', 'config_param.yaml')  # Construct path to config file
    config_param = load_config(config_path)  # Load configuration
    # 订阅RGB和深度图像话题
    rospy.Subscriber('/l515/color/image_raw', Image, rgb_callback)
    rospy.Subscriber('/l515/depth/image_rect_raw', Image, depth_callback)
    try:
        fx, fy, cx, cy = config_param['depth_fx'], config_param['depth_fy'], \
                config_param['depth_cx'], config_param['depth_cy']
        # 初始化地图管理器(MapManager)，设置Z轴多边形合并阈值
        # 从config_param配置字典中获取'merge_dis_thresh'参数作为z_threshold值
        # 这个阈值用于判断在Z轴方向上，两个多边形距离小于此值时将被合并
        # 典型值范围：0.02-0.1米，具体取决于场景需求
        map_manager = MapManager(z_threshold=config_param['merge_dis_thresh'])

        node = MappingNode(map_manager, 
                        map_frame=config_param['map_frame'], 
                        camera_frame=config_param['camera_frame'])

        time.sleep(1)  # Wait 3 seconds for listener setup

        marker_new_pub = rospy.Publisher('/polygon_new_markers', Marker, queue_size=20)

        # 配置OpenCV图像处理参数
        # 该函数设置多个图像预处理的关键参数，主要用于边缘检测和图像增强
        set_cv_config(
            config_param['Canny_thresh1'],  # Canny边缘检测低阈值（建议值：50-100）
            config_param['Canny_thresh2'],  # Canny边缘检测高阈值（建议是低阈值的2-3倍）
            config_param['anisotropic_diffusion_num_iter'],  # 各向异性扩散迭代次数（通常4-10次）
            config_param['anisotropic_diffusion_kappa'],  # 扩散系数（建议20-100）
            config_param['anisotropic_diffusion_gamma']  # 时间步长（建议0.1-0.25）
        )

        # 设置形态学闭运算的核大小
        # 闭运算用于填充小孔洞和连接断裂边缘
        # 核大小应为奇数，典型值3x3或5x5
        # 值越大，填充效果越强但可能过度平滑
        closing_kernel = config_param['closing_kernel']  # 例如：np.ones((3,3), np.uint8)

        # 检查配置中是否启用了数据保存功能
        # 通过config_param['save_data']布尔值控制（建议在yaml中设为true/false）
        if config_param['save_data']:#我关了
            # 初始化图像索引计数器
            # 用于生成序列化文件名（如0001.jpg, 0002.jpg...）
            image_index = 1  # 从1开始计数更符合人类习惯

            # 构建数据集根目录路径
            # 通常位于ROS包的data子目录下（如~/catkin_ws/src/polygon_mapping/data）
            dataset_root = os.path.join(package_path, 'data')
            
            # 创建带有时间戳的唯一数据集文件夹
            # 示例实现：create_dataset_folder()可能生成类似"20230815_142530"格式的目录名
            # 避免多次运行数据被覆盖
            dataset_path = create_dataset_folder(dataset_root)
            
            # 创建RGB图像存储子目录
            # exist_ok=True表示目录已存在时不报错
            # 典型路径：~/catkin_ws/src/polygon_mapping/data/20230815_142530/rgb
            os.makedirs(os.path.join(dataset_path, "rgb"), exist_ok=True)
            
            # 创建深度图像存储子目录
            # 深度图像通常保存为16位PNG格式
            # 典型路径：~/catkin_ws/src/polygon_mapping/data/20230815_142530/depth
            os.makedirs(os.path.join(dataset_path, "depth"), exist_ok=True)
            
            # 定义位姿变换矩阵文件路径
            # 用于存储相机相对于世界坐标系的变换矩阵
            # 典型内容：每行包含时间戳和4x4变换矩阵
            t_file_path = os.path.join(dataset_path, "transformation_matrix.txt")

        # Main loop 
        while True:
            # 将彩色帧数据转为numpy数组
            color_image = np.asanyarray(l515_color_image)

            # 记录循环开始时刻的时间戳（用于计算处理耗时）注意：在Linux系统上典型精度约1微秒
            time_start_loop = time.time()
            
            # 将深度帧数据转换为numpy数组
            # depth_image = np.asanyarray(l515_depth_image)
            depth_image = l515_depth_image


            # 获取当前系统时间戳
            timestamp = time.time()

            # 深度拷贝变换矩阵（保证线程安全）
            T_lock = copy.deepcopy(node.T)  # 创建完全独立的矩阵副本

            # 记录各向异性扩散处理的开始时间
            time_start = time.time()
            # 应用各向异性扩散算法平滑深度图像
            # 函数参数说明（假设实现如下）：
            # depth_image: 输入深度图（uint16或float32类型，单位毫米/米）
            # 返回值smoothed_depth_img: 平滑后的深度图（保持原始数据类型）
            # 算法特性：
            # 1. 保留重要边缘（如物体边界）
            # 2. 平滑同质区域（如墙面）
            # 3. 对噪声和无效值（0或NaN）具有鲁棒性
            smoothed_depth_img = anisotropic_diffusion(depth_image)
            # print('Anisotropic diffusion time cost', 1000 * (time.time() - time_start), 'ms')

            # 从平滑后的深度图像计算表面法向量
            # 参数说明：
            # - smoothed_depth_img: 经过各向异性扩散处理的深度图（单位：米，float32类型）
            # - fx/fy: 相机X/Y轴焦距（像素单位），来自相机内参
            # - cx/cy: 相机光心坐标（像素单位），来自相机内参
            # 返回值normals: 表面法向量图（与输入同尺寸，shape=[H,W,3]）
            #   每个像素位置存储单位法向量[x,y,z]，值范围[-1, 1]
            normals = depth_to_normals(smoothed_depth_img, fx, fy, cx, cy)#!!deepseek说可以用gpu加速

            # 从OpenCV的轨迹栏(trackbar)获取Canny边缘检测的双阈值参数
            # 'Filtered Image with Largest Contour' 是显示窗口的名称
            # thresh1: 低阈值（弱边缘阈值）
            # thresh2: 高阈值（强边缘阈值）
            # 注意：阈值范围通常为0-255，高阈值建议是低阈值的2-3倍
            thresh1 = cv2.getTrackbarPos('Canny thresh1', 'Filtered Image with Largest Contour')
            thresh2 = cv2.getTrackbarPos('Canny thresh2', 'Filtered Image with Largest Contour')

            # 定义锐化卷积核（拉普拉斯增强型）
            # 核结构说明：
            # [[ 0, -1,  0],
            #  [-1,  5, -1],
            #  [ 0, -1,  0]]
            # 中心权重为正，周围为负，增强高频成分
            kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])

            # 根据配置决定是否应用锐化滤波
            if config_param['use_sharpening']:
                # 应用二维卷积滤波
                # 参数说明：
                # - normals: 输入图像（法向量图或深度图）
                # - -1: 输出图像深度与输入相同
                # - kernel: 3x3锐化核
                filtered_img = cv2.filter2D(normals, -1, kernel)
            else:
                # 直接使用原始图像（不锐化）
                filtered_img = normals

            filtered_img = add_border_conditionally(filtered_img)
            edges = cv2.Canny(filtered_img, thresh1, thresh2)

            kernel = np.ones((closing_kernel, closing_kernel), np.uint8)
            # 应用形态学闭运算（先膨胀后腐蚀）
            # 作用：
            # 1. 填充边缘中的小孔洞和断裂
            # 2. 平滑边缘轮廓
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

            # 在边缘图像中查找轮廓
            # 参数说明：
            # - edges: 处理后的二值图像
            # - cv2.RETR_TREE: 检索所有轮廓并重建完整层次结构
            # - cv2.CHAIN_APPROX_SIMPLE: 压缩水平/垂直/对角线方向的冗余点
            # 返回值：
            # - contours: 轮廓点列表（每个轮廓是Nx1x2的numpy数组）
            # - hierarchy: 轮廓层次关系（[Next, Previous, First_Child, Parent]）
            contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # 从配置中读取最小面积阈值（用于过滤小轮廓）    5000
            min_area_thresh = config_param['min_area_thresh']
            # 初始化新多边形列表（用于存储过滤后的有效轮廓）
            new_polygon_list = []

            # 遍历所有检测到的轮廓
            for i, contour in enumerate(contours):
                # 跳过没有父轮廓的顶层轮廓（只处理内嵌轮廓）
                # hierarchy结构：[Next, Previous, First_Child, Parent]
                # Parent=-1表示是顶层轮廓
                if hierarchy[0][i][3] == -1:
                    continue

                # 跳过面积小于阈值的轮廓（过滤噪声）
                # cv2.contourArea计算轮廓包围的区域面积（像素单位）
                if cv2.contourArea(contour) < min_area_thresh:
                    continue

                # 使用多边形近似简化轮廓（减少顶点数量）
                # epsilon: 近似精度（周长百分比，0.02表示保留98%形状特征）
                # approx: 近似后的顶点坐标数组（Nx1x2格式）
                epsilon = 0.02 * cv2.arcLength(contour, True)  # True表示轮廓闭合
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                # 检查简化后的轮廓面积是否仍满足阈值
                if cv2.contourArea(approx) < min_area_thresh:
                    continue

                # 可视化：在原图上绘制绿色轮廓线和红色顶点
                cv2.drawContours(filtered_img, [approx], 0, (0, 255, 0), 2)  # 绿色，线宽2
                for point in approx:
                    cv2.circle(filtered_img, tuple(point[0]), 5, (0, 0, 255), -1)  # 红色实心圆

                # 将轮廓顶点映射到深度图生成3D点云区域
                # approx.reshape(-1,2): 将顶点转为Nx2格式
                # 返回的polygon_points是Nx3数组（x,y,z坐标）
                polygon_points = depth_to_point_cloud_region(smoothed_depth_img, approx.reshape(-1, 2), fx, fy, cx, cy)
                
                # 检查点云是否有效（避免空数据）
                if polygon_points.size == 0:
                    # print("No valid points in the polygon region")
                    continue

                try:
                    # 使用RANSAC算法拟合3D平面
                    # 记录开始时间用于性能分析
                    time_start = time.time()
                    
                    # 平面拟合参数（从配置文件获取）：
                    # normal: 平面法向量（单位向量）
                    # d: 平面方程常数项（ax+by+cz+d=0）
                    normal, d = fit_plane_ransac(
                        polygon_points,
                        max_trials=config_param['ran_max_trials'],        # RANSAC最大迭代次数
                        min_samples=config_param['ran_min_samples'],      # 最小内点数量
                        residual_threshold=config_param['ran_residual_threshold'],  # 内点阈值（米）
                        outlier_threshold=config_param['ran_outlier_threshold']     # 离群点阈值（米）
                    )
                    # print('fit_plane_ransac time cost', 1000 * (time.time() - time_start), 'ms')

                    # 将轮廓顶点投影到拟合平面
                    projected_points = project_points_to_plane((*normal, d),  # 平面参数打包
                        fx, fy, cx, cy,  # 相机内参
                        approx.reshape(-1, 2)  # 输入顶点
                    )

                    # 处理多边形并转换到世界坐标系
                    # angle: 多边形主要方向角（可选）
                    # world_vertices: 世界坐标系下的顶点坐标（Nx3）
                    angle, world_vertices = process_polygon(projected_points, normal, T_lock)
                    
                    # 存储有效多边形
                    if world_vertices is not None:
                        new_polygon_list.append(world_vertices)

                except ValueError as e:
                    print("Error in fitting plane:", e)
                    # 常见错误：RANSAC无法找到有效平面（点共线或噪声过大）

            #得到一帧图像的世界坐标系下的多边形列表new_polygon_list
            for i, polygon in enumerate(new_polygon_list):
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "polygons_new"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.lifetime = rospy.Duration(0.5)  # Marker automatically disappears after 0.5 seconds if not updated
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.05
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0

                # Add polygon vertices
                for vertex in polygon:
                    point = Point()
                    point.x = vertex[0]
                    point.y = vertex[1]
                    point.z = vertex[2]
                    marker.points.append(point)
                if len(polygon) > 2:
                    point = Point()
                    point.x = polygon[0][0]
                    point.y = polygon[0][1]
                    point.z = polygon[0][2]
                    marker.points.append(point)  # Close the polygon

                marker_new_pub.publish(marker)




            # 将新检测到的多边形列表添加到地图管理器
            # new_polygon_list: 包含多个多边形的列表，每个多边形是世界坐标系下的3D顶点集合（Nx3数组）
            map_manager.add_polygon_list(new_polygon_list)

            # 典型实现逻辑（在MapManager类中）：
            # 1. 多边形融合处理：
            #    - 将新多边形与现有地图多边形进行对比
            #    - 根据z_threshold参数判断是否合并重叠/邻近的多边形
            # 2. Z轴漂移估计：
            #    - 比较新增多边形与已有多边形在Z轴方向的差异
            #    - 计算相机高度方向的累计漂移量
            # 3. 地图更新：
            #    - 维护多边形拓扑关系
            #    - 触发地图更新事件通知其他模块

            # 检查地图管理器中是否存在已构建的多边形
            # map_manager.polygons 是存储所有多边形的列表
            if map_manager.polygons:
                # 调用地图管理器的绘图方法生成地图图像
                # 该方法应返回一个OpenCV格式的BGR图像（numpy数组）
                map_img = map_manager.plot_polygons()
                
                # 使用OpenCV显示地图图像
                # 窗口标题为'Polygon Map'
                # cv2.imshow('Polygon Map', map_img)

            # Save images and transformation matrix if data saving is enabled
            if config_param['save_data']:
                rgb_filename = os.path.join(dataset_path, "rgb", f"{image_index:06d}.png")
                cv2.imwrite(rgb_filename, color_image)

                depth_filename = os.path.join(dataset_path, "depth", f"{image_index:06d}.png")
                cv2.imwrite(depth_filename, depth_image)

                with open(t_file_path, 'a') as t_file:
                    t_file.write(f"{image_index},{T_lock.flatten()},{timestamp}\n")
                image_index += 1

            # # Display results
            cv2.imshow('Filtered Image with Largest Contour', filtered_img)
            cv2.imshow('smoothed_depth_img', cv2.applyColorMap(cv2.convertScaleAbs(smoothed_depth_img, alpha=0.03), cv2.COLORMAP_JET))
            # cv2.imshow('normals Image', normals)
            # cv2.imshow('Color Image', color_image)
            # cv2.imshow('Edges', edges)

            # print('Loop time cost', 1000 * (time.time() - time_start_loop), 'ms')

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        # Ensure pipeline stops and all OpenCV windows close
        cv2.destroyAllWindows()

        if config_param.get('save_data', False):
            localtime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            map_manager.save_polygons_to_file(os.path.join(dataset_path, f'{localtime}_polygons.txt'))
            cv2.imwrite(os.path.join(dataset_path, f'{localtime}_map_img.jpg'), map_img)

if __name__ == '__main__':
    main()
