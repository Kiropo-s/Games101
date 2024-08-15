//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols / 2;
        height = image_data.rows / 2;
        cv::pyrDown(image_data, image_data ,cv::Size(width, height));
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u = std::fmax(u, 0);
        u = std::fmin(u, 1);
        v = std::fmax(v, 0);
        v = std::fmin(v, 1);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        // Bilinear
        float u_min = floor(u_img);
        float u_max = ceil(u_img);
        float v_min = floor(v_img);
        float v_max = ceil(v_img);

        auto u00 = image_data.at<cv::Vec3b>(v_min, u_min);
        auto u10 = image_data.at<cv::Vec3b>(v_max, u_min);
        auto u01 = image_data.at<cv::Vec3b>(v_min, u_max);
        auto u11 = image_data.at<cv::Vec3b>(v_max, u_max);

        auto s = (u_img - u_min) / (u_max - u_min), t = (v_img - v_min) / (v_max - v_min);
        auto u0 = u00 + s * (u10 - u00);
        auto u1 = u01 + s * (u11 - u01); 

        auto color = u1 + t * (u0 - u1);

        //auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif // RASTERIZER_TEXTURE_H
