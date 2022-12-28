//realsense.h
#pragma once

#include <librealsense2/rs.hpp>
//#include "../samples/example.hpp"

#include <map>
#include <thread>
#include <array>
#include <mutex>

#include "yolo_detector.h"
#include <memory>
#include "marker_position_compensater.h"




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
	bool showTime_ = false;
	bool showImage_ = false;

	rs2::pipeline pipe_;
	rs2::pointcloud pc_;
	rs2::points points_;
	rs2::config cfg_;

	rs2::frame_queue depth_que_;
	rs2::frame_queue color_que_;

	//並列処理予定だったが，しないほうが速い...
	std::thread thread_;
	std::mutex mtx_, mtx2_;

	std::vector<filter_options> filters_;
	std::vector<float> threshold_;

	const std::string disparity_filter_name = "Disparity";
	rs2::disparity_transform* disparity_to_depth;

	int RSImageHeight_ = 480;
	int RSImageWidth_ = 640;
	//(W,H) = (640,480),(848,480), (1280,720)
	int RSDepthHeight_ = 720;
	int RSDepthWidth_ = 1280;
	int RSFps_ = 15;

	//for YOLO
	cv::Mat image_;
	cv::Mat HSVImage_;
	std::vector<std::array<float, 3>> classMeanPos_;
	std::unique_ptr<YOLODetector> yolov7_;
	bool showInference_ = false;
	//const float confThreshold = 0.25f;
	//const float iouThreshold = 0.65f;

	std::vector<Detection> detection_result_;
	void _findMarkers(cv::Mat& image, std::vector<Detection>& result);
	std::vector<std::vector<int>> classIdImage_;
	const int classNum = 6;
	// class3DPositions[i]:=classIdがiである点群のリスト
	//std::array<std::vector<std::array<double, 3>>, classNum> class3DPositions;
	std::vector<std::vector<std::array<float, 3>>> class3DPositions_;

	vector<bool> found_;
	//(先端～黄色マーカー)/(黄色マーカー～緑マーカー)の比率
	float ratio_ty_over_yg = 1.4;
	std::vector<float> ratio_tm1_over_m1m2_;
	bool enableMarkerPoseCompensater_ = false;
	unique_ptr<MarkerPositionCompensater> marker_position_compensater_;
	//ツールの半径(厳密には，ツール中心と，黄色マーカー～緑マーカーを結んだ直線の距離)
	double length_tc = 0.005;
	double ZratioLimit_ = 1.02;
	bool enableCompensateMarkersDistance_ = true;
	bool updateLastZ_ = true;
	void _calcPositions(std::vector<float>& points, std::vector<int>& color);

	void _getMatImage(cv::Mat& Image, const rs2::video_frame& texture, const uint8_t* texture_data);

	void _setFilters();
	void _process();


	int bytes_per_pixel = -1;
	int stride_in_bytes = -1;
	void _setBytesAndStride(const rs2::video_frame& texture);
	inline void RealSenseInterface::_getTextureColor(std::array<uint8_t, 3>& rgb, const rs2::video_frame& texture, const uint8_t* texture_data, float u, float v, bool& isOut, int& classId);


public:
	RealSenseInterface();
	~RealSenseInterface();

	void getPointCloud(std::vector<float>& points, std::vector<int>& color);
	void getPointCloud2(std::vector<float>& points, std::vector<int>& color);
};

/* TO DO


YOLO->480*480
RealSense->480*640

classImage->480*640?
result->?

480と640の使い分け，変換部分不十分．



*/