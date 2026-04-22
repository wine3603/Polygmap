#ifndef SHARPEN_CUDA_H
#define SHARPEN_CUDA_H

// 声明锐化函数接口
void sharpen_cuda(const unsigned char* input_host, unsigned char* output_host, int width, int height);

#endif // SHARPEN_CUDA_H

