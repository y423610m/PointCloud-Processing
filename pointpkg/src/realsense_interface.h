//realsense.h
#pragma once

#include <librealsense2/rs.hpp>
//#include "../samples/example.hpp"

#include <map>
#include <thread>

#include "yolo_detector.h"
#include <memory>


/*

getPointCloud()


/////////

void updateQueue:enqueueにfilterd_frameを加える

thread:while(1) updateEnqueue

getCloud:enqueueから取り出してvectorにセット


///////////////////
framesetはframe(depth, color)を内包



*/

struct filter_slider_ui
{
	std::string name;
	std::string label;
	std::string description;
	bool is_int;
	float value;
	rs2::option_range range;

	static bool is_all_integers(const rs2::option_range& range);
};


//  [rs2::filter&] -> [rs2::filter]
class filter_options
{
public:
	filter_options(const std::string name, rs2::filter& filter);
	filter_options(filter_options&& other);
	std::string filter_name;                                   //Friendly name of the filter
	rs2::filter filter;                                       //The filter in use
	std::map<rs2_option, filter_slider_ui> supported_options;  //maps from an option supported by the filter, to the corresponding slider
	std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not
};

class RealSenseInterface {
private:

	bool initialized_ = false;

	rs2::pipeline pipe_;
	rs2::pointcloud pc_;
	rs2::points points_;
	rs2::config cfg_;

	rs2::frame_queue depth_que_;
	rs2::frame_queue color_que_;

	std::thread thread_;

	std::vector<filter_options> filters_;
	std::vector<double> threshold_;

	const std::string disparity_filter_name = "Disparity";
	rs2::disparity_transform* disparity_to_depth;

	//for YOLO
	std::unique_ptr<YOLODetector> yolov7_;
	const float confThreshold = 0.25f;
	const float iouThreshold = 0.65f;


	void _setFilters();
	void _process();
	


public:
	RealSenseInterface();
	~RealSenseInterface();

	void getPointCloud(std::vector<float>& points, std::vector<int>& color);
	void getPointCloud2(std::vector<float>& points, std::vector<int>& color);
};


/*
	
	///ONNXに変換忘れるなー（笑）
	
std::string modelPath = "C:/Users/y4236/Documents/ToolDetectYOLO/yolov7/CobottaTool/result/train8/weights/last.onnx";
bool isGPU = false;
YOLODetector detector(modelPath, isGPU, cv::Size(480, 480));

std::string imagePath = "C:/Users/y4236/Documents/ToolDetectYOLO/yolov7/CobottaTool/train/141741821192.jpg";
cv::Mat image = cv::imread(imagePath);
cv::resize(image, image, cv::Size(480, 480));
EL(image.size())

std::vector<Detection> result;
result = detector.detect(image, confThreshold, iouThreshold);
std::vector<std::string> classNames = { "Tool", "Yellow", "Green", "Blue", "Pink" };
for (int i = 5; i <= 100; i++) classNames.push_back(to_string(i));
utils::visualizeDetection(image, result, classNames);
cv::imshow("result", image);
//cv::imwrite("result.jpg", image);
cv::waitKey(0);
*/