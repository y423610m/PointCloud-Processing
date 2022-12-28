#include "marker_position_compensater.h"
#include "development_commands.h"

std::wstring charToWstring(const char* str)
{
	typedef std::codecvt_utf8<wchar_t> convert_type;
	std::wstring_convert<convert_type, wchar_t> converter;

	return converter.from_bytes(str);
}



MarkerPositionCompensater::MarkerPositionCompensater(string model_path) {
	EL("aaa");
	//session = Ort::Session(env, w_modelPath.c_str(), sessionOptions);
	env = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "ONNX_COMPENSATATION");

#ifdef _WIN32
	std::wstring w_modelPath = charToWstring(model_path.c_str());
	session_ = Ort::Session(env, w_modelPath.c_str(), sessionOptions);
#else
	session_ = Ort::Session(env, model_path.c_str(), sessionOptions);
#endif

	memoryInfo_.reset(new Ort::MemoryInfo(nullptr));
	*memoryInfo_ = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

	Ort::AllocatorWithDefaultOptions allocator;
	inputNames.push_back(session_.GetInputName(0, allocator));
	outputNames.push_back(session_.GetOutputName(0, allocator));
	EL(inputNames[0]);
	EL(outputNames[0]);
	//Ort::SessionOptions sessionOptions;
	//std::wstring path = charToWstring(model_path.c_str());
	//session_ = Ort::Session(env, path.c_str(), sessionOptions);
	EL("bbb");

	Ort::TypeInfo inputTypeInfo = session_.GetInputTypeInfo(0);
	std::vector<int64_t> inputTensorShape = inputTypeInfo.GetTensorTypeAndShapeInfo().GetShape();
	for (int i = 0; i < inputTensorShape.size(); i++) EL(inputTensorShape[i]);
	Ort::TypeInfo outputTypeInfo = session_.GetOutputTypeInfo(0);
	std::vector<int64_t> outputTensorShape = outputTypeInfo.GetTensorTypeAndShapeInfo().GetShape();
	for (int i = 0; i < outputTensorShape.size(); i++) EL(outputTensorShape[i]);


}


void MarkerPositionCompensater::predict(array<float, 3>& M1, array<float, 3>& M2){
	array<float, 6> inputValues;
	for (int i = 0; i < 3; i++) inputValues[i] = M1[i]*1000;
	for (int i = 0; i < 3; i++) inputValues[3+i] = M2[i]*1000;

	std::vector<Ort::Value> inputTensors_;
	inputTensors_.emplace_back(Ort::Value::CreateTensor<float>(
		*memoryInfo_, inputValues.data(), inputValues.size(),
		input_shape_.data(), input_shape_.size()
		));
	//PL("created input tensor")
	std::vector<Ort::Value> outputTensors_;
	//try {
		outputTensors_ = this->session_.Run(Ort::RunOptions{ nullptr },
			inputNames.data(),
			inputTensors_.data(),
			1,
			outputNames.data(),
			1);
		auto* rawOutput = outputTensors_[0].GetTensorData<float>();
		std::vector<int64_t> outputShape = outputTensors_[0].GetTensorTypeAndShapeInfo().GetShape();
		size_t count = outputTensors_[0].GetTensorTypeAndShapeInfo().GetElementCount();
		//std::vector<float> output_(rawOutput, rawOutput + count);
		//if (output_.size()) output_.clear();
		//for (int i = 0; i < count; i++) output_.emplace_back(rawOutput[i]);
		//for (int i = 0; i < 6; i++) EL(rawOutput[i]);
		for (int i = 0; i < 3; i++) M1[i] = rawOutput[i]/1000;
		for (int i = 0; i < 3; i++) M2[i] = rawOutput[3+i]/1000;
	//}
	//catch (Ort::Exception& e) {
	//	EL(e.what())
	//}
}
