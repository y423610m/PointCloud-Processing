//gui.h
#pragma once

#define CVUI_IMPLEMENTATION

#include <time.h>

#include <manager.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

class GUI
{
private:
	cv::Mat frame_;
	double a = 12.4;
	Manager* manager_;
	clock_t t;
	float delta = 0.005;
	int scale = 1;

public:
	GUI(const GUI&) = delete;
	GUI(Manager* operator_side_manager);
	~GUI();
	void update();
	bool check_loop();
};
