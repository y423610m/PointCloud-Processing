#include "yolo_detector.h"

YOLODetector::YOLODetector(const std::string& modelPath,
    const bool& isGPU = true,
    const cv::Size& inputSize = cv::Size(640, 640),
    const cv::Size& originalImageShape = cv::Size(640, 640))

:inputImageShape_(inputSize),
originalImageShape_(originalImageShape)
{
    env = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "ONNX_DETECTION");
    sessionOptions = Ort::SessionOptions();

    std::vector<std::string> availableProviders = Ort::GetAvailableProviders();
    auto cudaAvailable = std::find(availableProviders.begin(), availableProviders.end(), "CUDAExecutionProvider");
    OrtCUDAProviderOptions cudaOption;

    if (isGPU && (cudaAvailable == availableProviders.end()))
    {
        std::cout << "GPU is not supported by your ONNXRuntime build. Fallback to CPU." << std::endl;
        std::cout << "Inference device: CPU" << std::endl;
    }
    else if (isGPU && (cudaAvailable != availableProviders.end()))
    {
        std::cout << "Inference device: GPU" << std::endl;
        sessionOptions.AppendExecutionProvider_CUDA(cudaOption);
    }
    else
    {
        std::cout << "Inference device: CPU" << std::endl;
    }

#ifdef _WIN32
    std::wstring w_modelPath = utils::charToWstring(modelPath.c_str());
    session = Ort::Session(env, w_modelPath.c_str(), sessionOptions);
#else
    session = Ort::Session(env, modelPath.c_str(), sessionOptions);
#endif

    Ort::AllocatorWithDefaultOptions allocator;

    Ort::TypeInfo inputTypeInfo = session.GetInputTypeInfo(0);
    std::vector<int64_t> inputTensorShape = inputTypeInfo.GetTensorTypeAndShapeInfo().GetShape();
    this->isDynamicInputShape = false;
    // checking if width and height are dynamic
    if (inputTensorShape[2] == -1 && inputTensorShape[3] == -1)
    {
        std::cout << "Dynamic input shape" << std::endl;
        this->isDynamicInputShape = true;
    }

    for (auto shape : inputTensorShape)
        std::cout << "Input shape: " << shape << std::endl;

    inputNames.push_back(session.GetInputName(0, allocator));
    outputNames.push_back(session.GetOutputName(0, allocator));

    std::cout << "Input name: " << inputNames[0] << std::endl;
    std::cout << "Output name: " << outputNames[0] << std::endl;

    //this->inputImageShape = cv::Size2f(inputSize);

    classNames_ = { "Tool", "Yellow", "Green", "Blue", "Pink" };
}

//for old yolo? not for yolov7
void YOLODetector::getBestClassInfo(std::vector<float>::iterator it, const int& numClasses,
    float& bestConf, int& bestClassId)
{
    // first 5 element are box and obj confidence
    bestClassId = 5;
    bestConf = 0;

    for (int i = 5; i < numClasses + 5; i++)
    {
        if (it[i] > bestConf)
        {
            bestConf = it[i];
            bestClassId = i - 5;
        }
    }

}

//void YOLODetector::preprocessing(cv::Mat& image, float*& blob, std::vector<int64_t>& inputTensorShape)
void YOLODetector::preprocessing(cv::Mat& image, std::vector<float>& blob)
{
    cv::cvtColor(image, RGBImage_, cv::COLOR_BGR2RGB);

    cv::resize(RGBImage_, resizedImage_, inputImageShape_);

    resizedImage_.convertTo(floatImage_, CV_32FC3, 1.0 / 255.0);

    // hwc -> chw èââÒÇÃÇ›é¿çs
    if (chw_.size() < floatImage_.channels()) {
        cv::Size floatImage_Size{ floatImage_.cols, floatImage_.rows };
        chw_.resize(floatImage_.channels());
        blob.resize(floatImage_.cols * floatImage_.rows * floatImage_.channels());

        for (int i = 0; i < floatImage_.channels(); ++i) {
            chw_[i] = cv::Mat(floatImage_Size, CV_32FC1, &(blob[0]) + i * floatImage_Size.width * floatImage_Size.height);
        }
    }

    cv::split(floatImage_, chw_);

}

std::vector<Detection> YOLODetector::postprocessing(std::vector<Ort::Value>& outputTensors)
{


    auto* rawOutput = outputTensors[0].GetTensorData<float>();
    std::vector<int64_t> outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
    size_t count = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
    //std::vector<float> output_(rawOutput, rawOutput + count);
    if (output_.size()) output_.clear();
    for (int i = 0; i < count; i++) output_.emplace_back(rawOutput[i]);
    //EL(output);
    /*
    outputShape -> å©Ç¬ÇØÇΩï®ëÃ*7
    0:?
    1-4:x1,y1,x2,y2
    5:classID
    6:confidence    
    */

    // for (const int64_t& shape : outputShape)
    //     std::cout << "Output Shape: " << shape << std::endl;

    // first 5 elements are box[4] and obj confidence
    //int numClasses = (int)outputShape[1] - 5;
    int elementsInBatch = (int)(outputShape[0] * outputShape[1]);

    // only for batch size = 1
    for (auto it = output_.begin(); it < output_.begin() + elementsInBatch; it += outputShape[1])
    {
        float confidence = it[6];
        if (confidence > confThreshold_)
        {
            int left = (int)(it[1]);
            int top = (int)(it[2]);
            int right = (int)(it[3]);
            int bottom = (int)(it[4]);
            int classId = it[5];

            int width = right - left;
            int height = bottom - top;

            boxes_.emplace_back(left, top, width, height);
            confs_.emplace_back(confidence);
            classIds_.emplace_back(classId);
        }
    }

    if (indices_.size()) indices_.clear();
    //cv::dnn::NMSBoxes(boxes_, confs_, confThreshold_, iouThreshold_, indices_);

    if (detections_.size()) detections_.clear();

    //for (int idx : indices_)
    for(int idx = 0;idx< boxes_.size();idx++)
    {
        Detection det;
        det.box = cv::Rect(boxes_[idx]);
        //utils::scaleCoords(resizedImageShape, det.box, originalImageShape);
        det.box.width = 1.0 * originalImageShape_.width / inputImageShape_.width * det.box.width;
        det.box.x = 1.0 * originalImageShape_.width / inputImageShape_.width * det.box.x;
        det.box.height = 1.0 * originalImageShape_.height / inputImageShape_.height * det.box.height;
        det.box.y = 1.0 * originalImageShape_.height / inputImageShape_.height * det.box.y;

        det.conf = confs_[idx];
        det.classId = classIds_[idx];
        detections_.emplace_back(det);
    }

    boxes_.clear();
    confs_.clear();
    classIds_.clear();

    return detections_;
}

std::vector<Detection> YOLODetector::detect(cv::Mat& image){
    if (!initialized_) {
        inputTensorShape_ = { 1, 3, inputImageShape_.height, inputImageShape_.width };
        inputTensorSize_ = utils::vectorProduct(inputTensorShape_);

        memoryInfo_.reset(new Ort::MemoryInfo(nullptr));
        *memoryInfo_ = Ort::MemoryInfo::CreateCpu(
            OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

        initialized_ = true;
    }

    this->preprocessing(image, inputTensorValues_);

    if (inputTensors_.size()) inputTensors_.clear();//1*Tensor
    inputTensors_.emplace_back(Ort::Value::CreateTensor<float>(
        *memoryInfo_, inputTensorValues_.data(), inputTensorSize_,
        inputTensorShape_.data(), inputTensorShape_.size()
        ));

    if (outputTensors_.size()) outputTensors_.clear();
    outputTensors_ = this->session.Run(Ort::RunOptions{ nullptr },
        inputNames.data(),
        inputTensors_.data(),
        1,
        outputNames.data(),
        1);

    if (result_.size()) result_.clear();
    result_ = this->postprocessing(outputTensors_);

    //utils::visualizeDetection(image, result, classNames_);

    return result_;
}