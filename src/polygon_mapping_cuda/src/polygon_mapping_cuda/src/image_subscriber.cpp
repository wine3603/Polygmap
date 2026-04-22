#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/core/cuda.hpp>  
#include <opencv2/cudafilters.hpp>  
#include <opencv2/cudaarithm.hpp>

#include <chrono> 
#include <vector>
#include "anisotropic_diffusion_cuda.h"
#include "normal_estimation_cuda.h"
#include "sharpen_cuda.h"
#include "pointcloud_cuda.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h> // 确保引入

#include <tf/transform_listener.h>

// 在文件开头的常量定义区域添加：
const float DEPTH_THRESHOLD_MM = 2000.0f;  // 2米=2000毫米

ros::Publisher pub;
ros::Publisher pcl_pub;

// 硬编码相机内参（根据您提供的参数）
const float fx = 366.7247314453125f;
const float fy = 366.7999572753906f;
const float cx = 322.73638916015625f;
const float cy = 242.31497192382812f;
const bool camera_info_received = true;  // 直接设置为true
tf::TransformListener* tf_listener = nullptr;


bool transformPointCloudToOdom(const std::vector<float>& cloud_in,
    std::vector<float>& cloud_out,
    const std::string& source_frame,
    const std::string& target_frame,
    tf::TransformListener* listener)
{
cloud_out.clear();
try {
tf::StampedTransform transform;
listener->waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.5));
listener->lookupTransform(target_frame, source_frame, ros::Time(0), transform);

tf::Vector3 t = transform.getOrigin();
tf::Matrix3x3 R = transform.getBasis();

for (size_t i = 0; i < cloud_in.size(); i += 3) {
tf::Vector3 pt(cloud_in[i], cloud_in[i+1], cloud_in[i+2]);
tf::Vector3 pt_transformed = R * pt + t;
cloud_out.push_back(pt_transformed.x());
cloud_out.push_back(pt_transformed.y());
cloud_out.push_back(pt_transformed.z());
}

return true;
} catch (tf::TransformException &ex) {
ROS_WARN("Transform failed: %s", ex.what());
return false;
}
}


bool fitPlaneAndEvaluate(const std::vector<float>& cloud_data, float& fit_ratio, Eigen::Vector3f& normal)
{
    // 输入点数量
    if (cloud_data.size() < 9) return false;  // 少于3个点

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = cloud_data.size() / 3;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width);

    for (size_t i = 0, j = 0; i < cloud->width; ++i, j += 3) {
        cloud->points[i].x = cloud_data[j];
        cloud->points[i].y = cloud_data[j + 1];
        cloud->points[i].z = cloud_data[j + 2];
    }

    // 平面模型拟合
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);  // 阈值根据你的点云尺度调整，单位米
    seg.setInputCloud(cloud);
    seg.segment(*inliers, coefficients);

    if (inliers->indices.empty()) {
        fit_ratio = 0.0;
        return false;
    }

    // 计算内点比例
    fit_ratio = static_cast<float>(inliers->indices.size()) / cloud->width;

    // 提取平面法向量
    normal = Eigen::Vector3f(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    normal.normalize();

    return true;
}


void publishPointCloudColored(const std::vector<float>& point_cloud,
    const std::vector<uint8_t>& colors,
    const std_msgs::Header& header,
    ros::Publisher& pcl_pub) {
if (point_cloud.empty() || colors.empty()) return;

size_t num_points = point_cloud.size() / 3;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
cloud->width = static_cast<uint32_t>(num_points);
cloud->height = 1;
cloud->is_dense = false;
cloud->points.resize(num_points);

for (size_t i = 0, j = 0; i < num_points; ++i, j += 3) {
cloud->points[i].x = point_cloud[i * 3 + 0];
cloud->points[i].y = point_cloud[i * 3 + 1];
cloud->points[i].z = point_cloud[i * 3 + 2];

// colors vector 里是 B G R 顺序
cloud->points[i].b = colors[j + 0];
cloud->points[i].g = colors[j + 1];
cloud->points[i].r = colors[j + 2];
}

sensor_msgs::PointCloud2 output;
pcl::toROSMsg(*cloud, output);
output.header = header;

pcl_pub.publish(output);
}

void publishPointCloud(const std::vector<float>& point_cloud, const std_msgs::Header& header, ros::Publisher& pcl_pub) {
    sensor_msgs::PointCloud2 cloud_msg;

    cloud_msg.header = header;
    cloud_msg.header.frame_id = "l515_depth_optical_frame";
    cloud_msg.height = 1;
    cloud_msg.width = point_cloud.size() / 3;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(cloud_msg.width);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (size_t i = 0; i < point_cloud.size(); i += 3) {
        *iter_x = point_cloud[i];
        *iter_y = point_cloud[i + 1];
        *iter_z = point_cloud[i + 2];
        ++iter_x; ++iter_y; ++iter_z;
    }

    pcl_pub.publish(cloud_msg);
}

void rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow("RGB Image", rgb);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("RGB cv_bridge error: %s", e.what());
    }
}

// 条件边框添加函数
cv::Mat addBorderConditionally(const cv::Mat& image) {
    CV_Assert(image.type() == CV_8UC3);  // 确保输入是三通道8位图

    int rows = image.rows;
    int cols = image.cols;

    cv::Mat new_image = image.clone();

    // 判断边缘像素是否满足条件：是否不等于 (127,127,127)
    auto checkNot127 = [](const cv::Vec3b& pixel) {
        return pixel[0] != 127 || pixel[1] != 127 || pixel[2] != 127;
    };

    // 顶部边缘
    for (int c = 0; c < cols; ++c) {
        if (checkNot127(image.at<cv::Vec3b>(0, c))) {
            new_image.at<cv::Vec3b>(0, c) = cv::Vec3b(255, 255, 255);
        }
    }

    // 底部边缘
    for (int c = 0; c < cols; ++c) {
        if (checkNot127(image.at<cv::Vec3b>(rows - 1, c))) {
            new_image.at<cv::Vec3b>(rows - 1, c) = cv::Vec3b(255, 255, 255);
        }
    }

    // 左侧边缘
    for (int r = 0; r < rows; ++r) {
        if (checkNot127(image.at<cv::Vec3b>(r, 0))) {
            new_image.at<cv::Vec3b>(r, 0) = cv::Vec3b(255, 255, 255);
        }
    }

    // 右侧边缘
    for (int r = 0; r < rows; ++r) {
        if (checkNot127(image.at<cv::Vec3b>(r, cols - 1))) {
            new_image.at<cv::Vec3b>(r, cols - 1) = cv::Vec3b(255, 255, 255);
        }
    }

    return new_image;
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!camera_info_received) {
        ROS_WARN_THROTTLE(2.0, "Camera info not available");
        return;
    }
    auto start_time = std::chrono::steady_clock::now();

    std::vector<float> point_cloud;
    try {
        cv::Mat depth = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        cv::Mat depth_f;
        depth.convertTo(depth_f, CV_32F);  // 单位毫米
        // ⚡优化建议: 可将深度阈值过滤和类型转换放到 CUDA 中执行（用 CUDA kernel 或 cv::cuda::threshold）

        // ---- GPU版深度过滤 ----
        cv::cuda::GpuMat depth_gpu, depth_filtered_gpu;
        depth_gpu.upload(depth_f);  // 上传到GPU
        // mask = depth_gpu > threshold
        cv::cuda::GpuMat mask;
        cv::cuda::compare(depth_gpu, DEPTH_THRESHOLD_MM, mask, cv::CMP_GT);
        // 克隆输入到输出
        depth_filtered_gpu = depth_gpu.clone();
        // 将超出阈值的像素设为0
        depth_filtered_gpu.setTo(0.0f, mask);
        // 下载回CPU（如果后续流程还在CPU上）
        cv::Mat depth_filtered;
        depth_filtered_gpu.download(depth_filtered);
        // ⚡优化建议: 上面双层for循环是CPU瓶颈之一，可改成OpenCV并行或GPU操作

        int w = depth.cols;
        int h = depth.rows;

        // std::vector<float> input(depth_f.begin<float>(), depth_f.end<float>());
        // std::vector<float> output(w * h);
        std::vector<float> input(depth_filtered.begin<float>(), depth_filtered.end<float>());
        std::vector<float> output(w * h);
        
        // CUDA 各向异性扩散
        anisotropic_diffusion_cuda(input.data(), output.data(), w, h, 20, 30.0f, 0.2f, 1);
        // ⚡优化建议: 确保 anisotropic_diffusion_cuda 内部使用流式并行（stream），避免同步阻塞CPU

        cv::Mat result(h, w, CV_32F, output.data());

        // 显示归一化深度图
        cv::Mat result_u16;
        result.convertTo(result_u16, CV_16U);
        // cv::Mat normalized;
        // result_u16.convertTo(normalized, CV_8U, 255.0 / 10000.0);  
        // cv::imshow("Smoothed Depth", normalized);
        // cv::waitKey(1);

        // 法线估计并显示
        std::vector<unsigned char> normal_rgb(w * h * 3);
        estimate_normals_cuda(output.data(), normal_rgb.data(), w, h, fx, fy);
        // ⚡优化建议: 若 estimate_normals_cuda 返回 GPU 内存，可直接传给 sharpen_cuda，无需CPU中转。

        cv::Mat normal_img(h, w, CV_8UC3, normal_rgb.data());
        // cv::imshow("Surface Normals", normal_img);
        // cv::waitKey(1);

        // --- 对法线图做 CUDA 锐化 ---
        cv::Mat normal_sharpened(normal_img.size(), normal_img.type());
        sharpen_cuda(normal_img.data, normal_sharpened.data, w, h);
        // ⚡优化建议: 若 normal_img 已经是 GPU Mat，可直接操作，不必先下载到 CPU 内存。

        // cv::imshow("Surface Normals Sharpened CUDA", normal_sharpened);
        // cv::waitKey(1);

        cv::Mat filtered_img = normal_sharpened;
        cv::Mat bordered_img = addBorderConditionally(filtered_img);
        // ⚡优化建议: addBorderConditionally 是 CPU 循环，可以改成 CUDA kernel 并行判断。

        // cv::imshow("Bordered Image", bordered_img);
        // cv::waitKey(1);

        // --- CUDA Canny ---
        cv::Mat bordered_gray;
        cv::cvtColor(bordered_img, bordered_gray, cv::COLOR_BGR2GRAY);
        cv::cuda::GpuMat d_input(bordered_gray);
        cv::cuda::GpuMat d_edges;

        double thresh1 = 100.0;
        double thresh2 = 150.0;
        int apertureSize = 3;
        bool L2gradient = false;
        cv::Ptr<cv::cuda::CannyEdgeDetector> canny =
            cv::cuda::createCannyEdgeDetector(thresh1, thresh2, apertureSize, L2gradient);
        // ⚡优化建议: canny 对象每帧都创建和销毁，建议定义为 static 或全局，减少构造销毁开销。

        canny->detect(d_input, d_edges);
        // ⚡优化建议: 可直接在 GPU 上继续形态学，不下载回CPU（减少带宽开销）

        // cv::Mat edges_cpu;
        // d_edges.download(edges_cpu);
        // cv::imshow("CUDA Canny Edges", edges_cpu);
        // cv::waitKey(1);

        // CUDA 闭运算
        int kernel_size = 5;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
        // ⚡优化建议: kernel 可定义为 static，每帧重复创建浪费时间。

        cv::Ptr<cv::cuda::Filter> dilate_filter = cv::cuda::createMorphologyFilter(
            cv::MORPH_DILATE, CV_8UC1, kernel);
        cv::Ptr<cv::cuda::Filter> erode_filter = cv::cuda::createMorphologyFilter(
            cv::MORPH_ERODE, CV_8UC1, kernel);
        // ⚡优化建议: 同样应缓存 filter 对象，避免重复创建。

        cv::cuda::GpuMat d_dilated, d_closed;
        dilate_filter->apply(d_edges, d_dilated);
        erode_filter->apply(d_dilated, d_closed);

        cv::Mat closed_edges;
        d_closed.download(closed_edges);
        // cv::imshow("CUDA Morphological Closing", closed_edges);
        // cv::waitKey(1);   

        // 提取轮廓（CPU）
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(closed_edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        // ⚡优化建议: findContours 是纯 CPU 操作且较慢，可考虑用并行化或 GPU 替代（OpenCV CUDA 暂不支持）
        // ⚡优化建议: 若无需显示全部轮廓，只找最大区域即可减少运算。

        // cv::Mat contour_vis = cv::Mat::zeros(closed_edges.size(), CV_8UC3);
        // cv::drawContours(contour_vis, contours, -1, cv::Scalar(0, 255, 0), 1);
        // cv::imshow("Contours", contour_vis);
        // cv::waitKey(1);

        float min_area_thresh = 3000.0f;
        int interp_num_points = 100;
        cv::Mat red_mask = cv::Mat::zeros(filtered_img.size(), CV_8UC1);

        int contours_num=0;
        std::vector<float> merged_cloud;
        std::vector<uint8_t> merged_colors;
        ROS_INFO("contours.size(): %ld ", contours.size());
        for (size_t i = 0; i < contours.size(); ++i) {
            if (hierarchy[i][3] == -1)
                continue;
            if (cv::contourArea(contours[i]) < min_area_thresh)
                continue;
            else
                contours_num++;

            // ⚡优化建议: 以下弧长 + 插值操作可以在 GPU 上实现（cuPy 或自定义 CUDA kernel），CPU上较慢。

            std::vector<cv::Point2f> approx;
            const std::vector<cv::Point>& contour = contours[i];
            std::vector<float> arc_lengths(contour.size(), 0.f);
            float total_length = 0.f;
            for (size_t j = 1; j < contour.size(); ++j) {
                float seg = cv::norm(contour[j] - contour[j - 1]);
                arc_lengths[j] = arc_lengths[j - 1] + seg;
            }
            total_length = arc_lengths.back();
            if (total_length < 1e-3)
                continue;

            for (int k = 0; k < interp_num_points; ++k) {
                float t = total_length * k / interp_num_points;
                size_t j = 1;
                while (j < arc_lengths.size() && arc_lengths[j] < t)
                    ++j;
                if (j >= arc_lengths.size())
                    break;
                float t0 = arc_lengths[j - 1];
                float t1 = arc_lengths[j];
                float ratio = (t - t0) / (t1 - t0 + 1e-5f);
                cv::Point2f pt = contour[j - 1] + ratio * (contour[j] - contour[j - 1]);
                approx.push_back(pt);
            }

            if (approx.size() < 3)
                continue;

            std::vector<cv::Point> approx_int;
            for (const auto& pt : approx)
                approx_int.emplace_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));

            if (cv::contourArea(approx_int) < min_area_thresh)
                continue;

            cv::drawContours(filtered_img, std::vector<std::vector<cv::Point>>{approx_int}, 0, cv::Scalar(0, 255, 0), 2);

            for (const auto& pt : approx) {
                cv::Point ipt(static_cast<int>(pt.x), static_cast<int>(pt.y));
                if (ipt.x >= 0 && ipt.x < filtered_img.cols && ipt.y >= 0 && ipt.y < filtered_img.rows) {
                    cv::circle(filtered_img, ipt, 3, cv::Scalar(0, 0, 255), -1);
                    red_mask.at<uchar>(ipt) = 1;
                }
            }

            // point_cloud = depthToPointCloudRegionCUDA(result, red_mask, fx, fy, cx, cy);
            // ⚡优化建议: 若连续多个轮廓要生成点云，建议合并 mask 一次生成，避免重复调用 CUDA kernel。

            // ROS_INFO("Extracted %zu points from red mask", point_cloud.size() / 3);
            // 当前轮廓对应点云
            std::vector<float> contour_cloud = depthToPointCloudRegionCUDA(result, red_mask, fx, fy, cx, cy);

            float fit_ratio = 0.0f;////////
            Eigen::Vector3f plane_normal;
            std::vector<uint8_t> contour_colors(contour_cloud.size() / 3 * 3); // 每个点3个字节 BGR
            
            std::vector<float> contour_cloud_odom;
            if (transformPointCloudToOdom(contour_cloud, contour_cloud_odom, "l515_depth_optical_frame", "odom", tf_listener)) {
                if (fitPlaneAndEvaluate(contour_cloud_odom, fit_ratio, plane_normal)) {
                    // 计算与 odom Z 轴夹角
                    Eigen::Vector3f odom_z(0, 0, 1);  // odom 坐标系的 Z 轴
                    float angle_deg = std::acos(std::abs(plane_normal.dot(odom_z))) * 180.0f / M_PI;
                    // ROS_INFO("Contour %zu: Plane fit ratio=%.2f, angle with odom Z=%.2f deg", i, fit_ratio, angle_deg);
            
                    // 根据条件给颜色
                    if (fit_ratio > 0.5 && angle_deg < 45.0f) {
                        ROS_INFO("Contour %zu: Plane fit ratio=%.2f, angle with odom Z=%.2f deg", i, fit_ratio, angle_deg);
                        for (size_t p = 0; p < contour_cloud.size() / 3; ++p) {
                            merged_colors.push_back(0);   // B
                            merged_colors.push_back(255);   // G
                            merged_colors.push_back(0); // R
                        }
                    } else {
                        for (size_t p = 0; p < contour_cloud.size() / 3; ++p) {
                            merged_colors.push_back(255); // B
                            merged_colors.push_back(255); // G
                            merged_colors.push_back(255); // R
                        }
                    }
            
                } else {
                    ROS_INFO("Contour %zu: Plane fitting failed in odom frame", i);
                    for (size_t p = 0; p < contour_cloud.size() / 3; ++p) {
                        merged_colors.push_back(0); // B
                        merged_colors.push_back(0); // G
                        merged_colors.push_back(255); // R
                    }
                }
            } else {
                ROS_WARN("Contour %zu: Transform to odom failed", i);
                for (size_t p = 0; p < contour_cloud.size() / 3; ++p) {
                    merged_colors.push_back(0); // B
                    merged_colors.push_back(0); // G
                    merged_colors.push_back(255); // R
                }
            }
                       
            merged_cloud.insert(merged_cloud.end(), contour_cloud.begin(), contour_cloud.end());
        }

        // ROS_INFO("contours.size %zu ", contours_num);
        // cv::imshow("Filtered Contours", filtered_img);
        // cv::waitKey(1);
        // 发布合并的彩色点云
        if (!merged_cloud.empty()) {
            publishPointCloudColored(merged_cloud, merged_colors, msg->header, pcl_pub);
        }
        // publishPointCloud(point_cloud, msg->header, pcl_pub);
        // ⚡优化建议: 可考虑将点云发布放入独立线程，与视觉处理并行（使用 ROS 异步 spinner）

        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "16UC1", result_u16).toImageMsg();
        pub.publish(out_msg);

    } catch (const std::exception& e) {
        ROS_ERROR("Depth processing error: %s", e.what());
    }

    auto end_time = std::chrono::steady_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    ROS_INFO("depthCallback processing time: %ld ms", duration_ms);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "image_subscriber_node");
    ros::NodeHandle nh;

    // 打印 OpenCV 版本和 CUDA 支持信息
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;

    if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
        std::cout << "[INFO] OpenCV CUDA is available. CUDA-enabled device count: "
                  << cv::cuda::getCudaEnabledDeviceCount() << std::endl;
        int device_id = 0;
        cv::cuda::setDevice(device_id);
        cv::cuda::DeviceInfo dev_info(device_id);
        std::cout << "[INFO] Using CUDA device: " << dev_info.name() << std::endl;
    } else {
        std::cerr << "[ERROR] OpenCV was compiled without CUDA or no CUDA device found!" << std::endl;
    }

    pub = nh.advertise<sensor_msgs::Image>("/smoothed_depth", 1);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/polygon_points", 1);
    
    ros::Subscriber rgb_sub = nh.subscribe("/l515/color/image_raw", 10, rgbCallback);
    // ros::Subscriber depth_sub = nh.subscribe("/l515/depth/image_raw", 10, depthCallback);
    ros::Subscriber depth_sub = nh.subscribe("/l515/depth/image_rect_raw", 10, depthCallback);
    // 移除了 info_sub 订阅，因为现在使用硬编码参数
    tf_listener = new tf::TransformListener(nh);

    cv::startWindowThread();

    ros::spin();
    return 0;
}