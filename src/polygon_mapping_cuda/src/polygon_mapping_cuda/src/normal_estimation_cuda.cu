#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cmath>

__global__
void compute_normals_kernel(const float* depth, uchar3* normals_rgb,
                            int width, int height, float fx, float fy)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x <= 0 || y <= 0 || x >= width - 1 || y >= height - 1) return;

    int idx = y * width + x;
    float zc = depth[idx];

    if (zc == 0) {
        normals_rgb[idx] = make_uchar3(0, 0, 0);
        return;
    }

    float dzdx = (depth[y * width + (x + 1)] - depth[y * width + (x - 1)]) / 2.0f;
    float dzdy = (depth[(y + 1) * width + x] - depth[(y - 1) * width + x]) / 2.0f;

    float nx = -dzdx * fx;
    float ny = -dzdy * fy;
    float nz = 1.0f;

    float norm = sqrtf(nx * nx + ny * ny + nz * nz) + 1e-8f;

    nx /= norm;
    ny /= norm;
    nz /= norm;

    normals_rgb[idx] = make_uchar3(
        (unsigned char)((nx + 1.0f) * 127.5f),
        (unsigned char)((ny + 1.0f) * 127.5f),
        (unsigned char)((nz + 1.0f) * 127.5f)
    );
}

void estimate_normals_cuda(const float* depth_host, unsigned char* rgb_host,
                           int width, int height, float fx, float fy)
{
    size_t size_depth = width * height * sizeof(float);
    size_t size_rgb = width * height * sizeof(uchar3);

    float* d_depth;
    uchar3* d_rgb;

    cudaMalloc(&d_depth, size_depth);
    cudaMalloc(&d_rgb, size_rgb);
    cudaMemcpy(d_depth, depth_host, size_depth, cudaMemcpyHostToDevice);

    dim3 block(16, 16);
    dim3 grid((width + 15) / 16, (height + 15) / 16);

    compute_normals_kernel<<<grid, block>>>(d_depth, d_rgb, width, height, fx, fy);
    cudaDeviceSynchronize();

    cudaMemcpy(rgb_host, d_rgb, size_rgb, cudaMemcpyDeviceToHost);

    cudaFree(d_depth);
    cudaFree(d_rgb);
}
