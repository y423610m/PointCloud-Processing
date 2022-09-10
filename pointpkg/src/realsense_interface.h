//realsense.h
#pragma once

#include <librealsense2/rs.hpp>
//#include "../samples/example.hpp"

#include <map>
#include <thread>

/*

getPointCloud()


/////////

void updateQueue:enqueue��filterd_frame��������

thread:while(1) updateEnqueue

getCloud:enqueue������o����vector�ɃZ�b�g


///////////////////
frameset��frame(depth, color)�����



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

	void _setFilters();
	void _process();
	


public:
	RealSenseInterface();
	~RealSenseInterface();

	void getPointCloud(std::vector<float>& points, std::vector<int>& color);
	void getPointCloud2(std::vector<float>& points, std::vector<int>& color);
};