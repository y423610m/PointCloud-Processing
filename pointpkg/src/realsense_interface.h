//realsense.h
#pragma once

#include <librealsense2/rs.hpp>
//#include "../samples/example.hpp"

//class filter_options
//{
//public:
//	filter_options(const std::string name, rs2::process_interface& filter);
//	filter_options(filter_options&& other);
//	std::string filter_name;                                   //Friendly name of the filter
//	rs2::process_interface& filter;                            //The filter in use
//	std::map<rs2_option, filter_slider_ui> supported_options;  //maps from an option supported by the filter, to the corresponding slider
//	std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not
//};

class RealSenseInterface {
private:

	int initialized_ = 0;

	rs2::pipeline pipe_;
	rs2::pointcloud pc_;
	rs2::points points_;
	rs2::config cfg_;

	std:: vector<double> threshold_;
	


public:
	RealSenseInterface();
	~RealSenseInterface();

	void getPointCloud(std::vector<float>& points, std::vector<int>& color);
};