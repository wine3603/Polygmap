#ifndef NORMAL_ESTIMATION_CUDA_H
#define NORMAL_ESTIMATION_CUDA_H

// 接口改为使用 unsigned char*
void estimate_normals_cuda(const float* depth_host, unsigned char* rgb_host,
                           int width, int height, float fx, float fy);

#endif  // NORMAL_ESTIMATION_CUDA_H
