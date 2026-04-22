#include <cuda_runtime.h>
#include <stdio.h>

__global__
void sharpen_kernel(const unsigned char* input, unsigned char* output, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    int idx = y * width + x;
    int idx3 = idx * 3; // 三通道索引

    // 对每个通道应用相同的3x3锐化核
    for (int c = 0; c < 3; ++c) {
        int center_val = input[idx3 + c] * 5;
        int sum = center_val;

        // 上
        if (y > 0) sum -= input[(y - 1) * width * 3 + x * 3 + c];
        // 下
        if (y < height - 1) sum -= input[(y + 1) * width * 3 + x * 3 + c];
        // 左
        if (x > 0) sum -= input[y * width * 3 + (x - 1) * 3 + c];
        // 右
        if (x < width - 1) sum -= input[y * width * 3 + (x + 1) * 3 + c];

        sum = sum < 0 ? 0 : (sum > 255 ? 255 : sum);
        output[idx3 + c] = static_cast<unsigned char>(sum);
    }
}

void sharpen_cuda(const unsigned char* input_host, unsigned char* output_host, int width, int height)
{
    size_t size = width * height * 3 * sizeof(unsigned char);
    unsigned char *d_input = nullptr;
    unsigned char *d_output = nullptr;

    cudaMalloc(&d_input, size);
    cudaMalloc(&d_output, size);

    cudaMemcpy(d_input, input_host, size, cudaMemcpyHostToDevice);

    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);

    sharpen_kernel<<<grid, block>>>(d_input, d_output, width, height);
    cudaDeviceSynchronize();

    cudaMemcpy(output_host, d_output, size, cudaMemcpyDeviceToHost);

    cudaFree(d_input);
    cudaFree(d_output);
}

