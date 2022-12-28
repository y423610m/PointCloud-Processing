#pragma once 

#include <string>
#include <array>
#include <codecvt>
#include <fstream>
#include <memory>
using namespace std;

#include <onnxruntime_cxx_api.h>
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "C:/Users/y4236/Documents/pcws/src/pointpkg/onnxruntime-win-x64-gpu-1.12.1/lib/onnxruntime.lib")
//https://github.com/lutzroeder/netron

class MarkerPositionCompensater {
private:
	Ort::Env env{ nullptr };
	Ort::Session session_{ nullptr };
	Ort::SessionOptions sessionOptions{ nullptr };
	unique_ptr<Ort::MemoryInfo> memoryInfo_;

	//Ort::Value input_tensor_{ nullptr };
	std::array<int64_t, 2> input_shape_{ 1,6 };
	std::vector<const char*> inputNames;

	//Ort::Value output_tensor_{ nullptr };
	std::array<int64_t, 2> output_shape_{ 1,6 };
	std::vector<const char*> outputNames;

	////const char* input_names[] = { "dense_6_input" };
	////const char* output_names[] = { "dense_8" };
public:
	MarkerPositionCompensater(string model_path);

	void predict(array<float, 3>& M1, array<float, 3>& M2);
};