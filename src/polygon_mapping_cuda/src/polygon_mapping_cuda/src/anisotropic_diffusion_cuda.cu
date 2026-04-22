#include <cuda_runtime.h>          // CUDA 运行时 API 头文件
#include <device_launch_parameters.h>  // CUDA 内核启动参数定义
#include <cmath>                  // C++ 数学库，包含 expf 函数

// CUDA 内核函数：对输入图像做一次迭代的各向异性扩散
__global__
void anisotropic_diffusion_kernel(const float* input, float* output,
                                  int width, int height,
                                  float kappa, float gamma, int option)
{
    // 计算当前线程处理的像素坐标 (x, y)
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    // 越界判断，如果超出图像范围则直接返回
    if (x >= width || y >= height) return;

    // 计算一维数组索引（行优先）
    int idx = y * width + x;

    // 边界像素不做处理，直接复制原值，避免访问越界
    if (x == 0 || y == 0 || x == width-1 || y == height-1) {
        output[idx] = input[idx];
        return;
    }

    // 读取当前像素值及其上下左右邻居像素值
    float center = input[idx];
    float north = input[(y - 1) * width + x];
    float south = input[(y + 1) * width + x];
    float west  = input[y * width + (x - 1)];
    float east  = input[y * width + (x + 1)];

    // 计算邻居与中心像素的差值（梯度）
    float dN = north - center;
    float dS = south - center;
    float dW = west  - center;
    float dE = east  - center;

    float cN, cS, cW, cE;  // 四个方向的扩散系数

    // 根据 option 选择扩散系数计算方式：
    if (option == 1) {
        // Perona-Malik 第一类扩散函数（指数型）
        cN = expf(-(dN / kappa) * (dN / kappa));
        cS = expf(-(dS / kappa) * (dS / kappa));
        cW = expf(-(dW / kappa) * (dW / kappa));
        cE = expf(-(dE / kappa) * (dE / kappa));
    } else {
        // Perona-Malik 第二类扩散函数（倒数型）
        cN = 1.0f / (1.0f + (dN / kappa) * (dN / kappa));
        cS = 1.0f / (1.0f + (dS / kappa) * (dS / kappa));
        cW = 1.0f / (1.0f + (dW / kappa) * (dW / kappa));
        cE = 1.0f / (1.0f + (dE / kappa) * (dE / kappa));
    }

    // 更新当前像素值 = 旧值 + gamma * (扩散系数加权梯度和)
    output[idx] = center + gamma * (cN * dN + cS * dS + cW * dW + cE * dE);
}

// 主机函数：多次迭代调用CUDA内核，完成各向异性扩散
void anisotropic_diffusion_cuda(const float* input_host, float* output_host,
                                int width, int height,
                                int iterations, float kappa, float gamma, int option)
{
    // 计算图像数据大小（字节数）
    size_t size = width * height * sizeof(float);

    float *d_img1 = nullptr, *d_img2 = nullptr;

    // 在设备上分配两块显存用于双缓冲输入输出
    cudaMalloc(&d_img1, size);
    cudaMalloc(&d_img2, size);

    // 将输入图像数据从主机拷贝到设备的 d_img1
    cudaMemcpy(d_img1, input_host, size, cudaMemcpyHostToDevice);

    // 定义CUDA线程块和网格大小
    dim3 block(16, 16); // 每个线程块16x16个线程
    dim3 grid((width + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);  

    // 迭代调用扩散内核
    for (int i = 0; i < iterations; i++) {
        anisotropic_diffusion_kernel<<<grid, block>>>(d_img1, d_img2, width, height, kappa, gamma, option);
        cudaDeviceSynchronize();  // 等待当前迭代完成

        // 交换输入输出指针，为下一次迭代做准备（双缓冲）
        float* tmp = d_img1;
        d_img1 = d_img2;
        d_img2 = tmp;
    }

    // 将最终结果从设备复制回主机的输出缓冲区
    cudaMemcpy(output_host, d_img1, size, cudaMemcpyDeviceToHost);

    // 释放设备显存
    cudaFree(d_img1);
    cudaFree(d_img2);
}
