#include <cuda_runtime.h>               // CUDA运行时API头文件
#include <vector>                       // C++标准向量容器
#include <opencv2/opencv.hpp>          // OpenCV图像处理库
#include <iostream>                    // 标准输入输出库

// CUDA kernel：遍历图像所有像素，mask为1的点计算3D坐标，写入point_cloud数组（线性存储xyzxyz...）
__global__ void maskToPointCloudKernel(const float* d_depth, const unsigned char* d_mask,
                                       int width, int height,
                                       float fx, float fy, float cx, float cy,
                                       float* d_point_cloud, int* d_count) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;  // 当前线程处理的像素索引
    int total = width * height;                       // 总像素数
    if (idx >= total) return;                         // 越界检查

    if (d_mask[idx] == 1) {                            // 如果该像素在mask中标记为1
        float z = d_depth[idx] * 0.001f;               // 深度单位为毫米，转换为米
        if (z <= 0.0f) return;                         // 忽略无效深度

        int x_img = idx % width;                       // 计算图像中x坐标
        int y_img = idx / width;                       // 计算图像中y坐标

        // 使用相机内参计算实际3D坐标（假设pinhole模型）
        float x = (x_img - cx) * z / fx;
        float y = (y_img - cy) * z / fy;

        // 原子加法获取该线程写入点云数组的位置索引
        int out_idx = atomicAdd(d_count, 1);
        d_point_cloud[out_idx * 3 + 0] = x;
        d_point_cloud[out_idx * 3 + 1] = y;
        d_point_cloud[out_idx * 3 + 2] = z;
    }
}

// 主机函数，调用CUDA kernel生成点云
std::vector<float> depthToPointCloudRegionCUDA(const cv::Mat& depth_img, const cv::Mat& mask,
                                               float fx, float fy, float cx, float cy) {
    int width = depth_img.cols;                    // 图像宽度
    int height = depth_img.rows;                   // 图像高度
    int total = width * height;                    // 总像素数

    // 设备端指针
    float* d_depth = nullptr;
    unsigned char* d_mask = nullptr;
    float* d_point_cloud = nullptr;
    int* d_count = nullptr;

    // 分配GPU内存
    cudaMalloc(&d_depth, total * sizeof(float));         // 分配深度图内存
    cudaMalloc(&d_mask, total * sizeof(unsigned char));  // 分配掩码图内存
    cudaMalloc(&d_point_cloud, total * 3 * sizeof(float)); // 最坏情况每个像素都生成点
    cudaMalloc(&d_count, sizeof(int));                   // 分配计数器内存
    cudaMemset(d_count, 0, sizeof(int));                 // 初始化点数为0

    // 拷贝图像数据到GPU
    cudaMemcpy(d_depth, depth_img.ptr<float>(), total * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_mask, mask.ptr<unsigned char>(), total * sizeof(unsigned char), cudaMemcpyHostToDevice);

    // 配置并启动CUDA kernel
    int threadsPerBlock = 256;
    int blocks = (total + threadsPerBlock - 1) / threadsPerBlock;
    maskToPointCloudKernel<<<blocks, threadsPerBlock>>>(d_depth, d_mask, width, height,
                                                         fx, fy, cx, cy,
                                                         d_point_cloud, d_count);
    cudaDeviceSynchronize();  // 等待kernel执行完毕

    // 从GPU读取有效点数
    int h_count = 0;
    cudaMemcpy(&h_count, d_count, sizeof(int), cudaMemcpyDeviceToHost);

    // 分配主机内存并拷贝点云数据回主机
    std::vector<float> point_cloud(h_count * 3);
    if (h_count > 0) {
        cudaMemcpy(point_cloud.data(), d_point_cloud, h_count * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    }

    // 释放GPU内存
    cudaFree(d_depth);
    cudaFree(d_mask);
    cudaFree(d_point_cloud);
    cudaFree(d_count);

    return point_cloud;  // 返回点云向量
}
