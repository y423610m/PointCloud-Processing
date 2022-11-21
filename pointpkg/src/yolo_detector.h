//https://github.com/itsnine/yolov5-onnxruntime/blob/master/src/detector.cpp を元にYOLOv7用に変更．高速化

#pragma once

#include <onnxruntime_cxx_api.h>
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "C:/Users/y4236/Documents/pcws/src/pointpkg/onnxruntime-win-x64-gpu-1.12.1/lib/onnxruntime.lib")

#include <string>
using namespace std;

#include "development_commands.h"




#pragma once
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <utility>

#include "yolo_utils.h"

/*
命名整理
InputTensorShape, OutputTensorShape: YOLOに学習させる画像サイズ．つまり，学習に使った画像サイズ
InputImageSize : このクラスが引数として受け取る画像サイズ．カメラの画像サイズ．


*/
class YOLODetector{
public:
    explicit YOLODetector(std::nullptr_t) {};
    YOLODetector(const std::string& modelPath,
        const bool& isGPU,
        const cv::Size& inputSize,
        const cv::Size& originalImageSize);

    std::vector<Detection> detect(cv::Mat& image);
    void setConfThreshold(float confThreshold) {
        confThreshold_ = confThreshold;
    }
    void setIouThreshold(float iouThreshold) {
        iouThreshold_ = iouThreshold;
    }
private:
    //for detect();
    bool initialized_ = false;
    //float* blob = nullptr;
    cv::Size inputImageShape_;
    cv::Size originalImageShape_;
    std::vector<float> inputTensorValues_;
    std::vector<int64_t> inputTensorShape_;
    std::vector<Ort::Value> inputTensors_;
    std::vector<Ort::Value> outputTensors_;
    std::vector<Detection> result_;
    size_t inputTensorSize_;
    unique_ptr<Ort::MemoryInfo> memoryInfo_;

    std::vector<std::string> classNames_;
    float confThreshold_ = 0.4;// 0.25f;
    float iouThreshold_ = 0.45;// 0.65f;
    Ort::Env env{ nullptr };
    Ort::SessionOptions sessionOptions{ nullptr };
    Ort::Session session{ nullptr };

    cv::Mat resizedImage_, RGBImage_, floatImage_;
    std::vector<cv::Mat> chw_;
    std::vector<Detection> detections_;
    std::vector<int> indices_;
    std::vector<float> output_;
    //void preprocessing(cv::Mat& image, float*& blob, std::vector<int64_t>& inputTensorShape);
    void preprocessing(cv::Mat& image, std::vector<float>& blob);


    std::vector<cv::Rect> boxes_;
    std::vector<float> confs_;
    std::vector<int> classIds_;
    std::vector<Detection> postprocessing(std::vector<Ort::Value>& outputTensors);


    static void getBestClassInfo(std::vector<float>::iterator it, const int& numClasses,
        float& bestConf, int& bestClassId);

    std::vector<const char*> inputNames;
    std::vector<const char*> outputNames;
    bool isDynamicInputShape{};
    //cv::Size2f inputImageShape;

};