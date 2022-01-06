//
// Created by lacie-life on 19/10/2021.
//

#pragma once

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <algorithm>
#include <iostream>
#include <time.h>

class Yolo {
    public:
        Yolo(std::string model_path, std::string names_path);
        ~Yolo();

        std::vector<torch::Tensor> non_max_suppression (torch::Tensor preds, float score_thresh, float iou_thresh);

        std::vector<torch::Tensor> detect(cv::Mat frame);

        cv::Mat drawObject(cv::Mat frame);

    private:
        torch::jit::script::Module module;
        std::vector<std::string> classnames;
};


