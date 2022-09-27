#pragma once

#include <onnxruntime_cxx_api.h>
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "C:/Users/y4236/Documents/pcws/src/pointpkg/onnxruntime-win-x64-gpu-1.12.1/lib/onnxruntime.lib")

#include <string>
using namespace std;

#include "development_commands.h"


//https://github.com/itsnine/yolov5-onnxruntime/blob/master/src/detector.cpp


#pragma once
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <utility>

#include "yolo_utils.h"


class YOLODetector{
public:
    explicit YOLODetector(std::nullptr_t) {};
    YOLODetector(const std::string& modelPath,
        const bool& isGPU,
        const cv::Size& inputSize);

    std::vector<Detection> detect(cv::Mat& image);
    void setConfThreshold(float confThreshold_) {
        confThreshold = confThreshold_;
    }
    void setIouThreshold(float iouThreshold_) {
        iouThreshold = iouThreshold_;
    }
private:
    std::vector<std::string> classNames_;
    float confThreshold = 0.4;// 0.25f;
    float iouThreshold = 0.45;// 0.65f;
    Ort::Env env{ nullptr };
    Ort::SessionOptions sessionOptions{ nullptr };
    Ort::Session session{ nullptr };

    void preprocessing(cv::Mat& image, float*& blob, std::vector<int64_t>& inputTensorShape);
    std::vector<Detection> postprocessing(const cv::Size& resizedImageShape,
        const cv::Size& originalImageShape,
        std::vector<Ort::Value>& outputTensors,
        const float& confThreshold, const float& iouThreshold);

    static void getBestClassInfo(std::vector<float>::iterator it, const int& numClasses,
        float& bestConf, int& bestClassId);

    std::vector<const char*> inputNames;
    std::vector<const char*> outputNames;
    bool isDynamicInputShape{};
    cv::Size2f inputImageShape;

};