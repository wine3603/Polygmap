#ifndef POINTCLOUD_CUDA_H
#define POINTCLOUD_CUDA_H

#include <vector>
#include <opencv2/opencv.hpp>

std::vector<float> depthToPointCloudRegionCUDA(const cv::Mat& depth_img, const cv::Mat& mask,
                                               float fx, float fy, float cx, float cy);

#endif

