#ifndef ANISOTROPIC_DIFFUSION_CUDA_H
#define ANISOTROPIC_DIFFUSION_CUDA_H

void anisotropic_diffusion_cuda(const float* input_host, float* output_host,
                                int width, int height,
                                int iterations, float kappa, float gamma, int option);

#endif // ANISOTROPIC_DIFFUSION_CUDA_H
