//gui.cpp
#include <gui.h>
#include <iostream>

#define WINDOW2_NAME "Window 2   Press ESCAPE Key to Stop This Program"
//#define CVUI_IMPLEMENTATION
#include "cvui.h"

GUI::GUI(Manager* operator_side_manager) :
	manager_(operator_side_manager)
	,t(clock())
{

	if (!ROSParam::getIntParam("GUI_enable_GUI")) return;

	//std::cerr << "GUI constructed" << std::endl;
	cvui::init(WINDOW2_NAME);
	frame_ = cv::Mat(cv::Size(800, 400), CV_8UC3);

	initialized_ = true;

}

GUI::~GUI() {
	PL("~GUI")
	//delete manager_;
}

void GUI::update() {
	if (!initialized_) return;

	frame_ = cv::Scalar(49, 52, 49);
	int x, y;
	//size
	x = 50; y = 20;
	cvui::printf(frame_, x, y, "original data size : %d", manager_->get_original_points_size());
	cvui::printf(frame_, x, y + 30, "output data size  : %d", manager_->get_points_size());
	auto now = clock();
	cvui::printf(frame_, x, y + 60, "time for 1 loop   : %d ms", now-t);
	t = now;

	cvui::text(frame_, x+220, y+50, "delta scale");
	cvui::trackbar(frame_, x+250, y + 50, 220, &scale, 0,100);

	delta = 0.001 * scale;


	//color track bar
	//x += 220;  y += -9;
	//cvui::text(frame_, x, y, "R");
	//cvui::text(frame_, x, y+50, "G");
	//cvui::text(frame_, x, y+100, "B");
	//x += 30; y += -8;
	//cvui::trackbar(frame_, x, y, 220, manager_->get_points_color(0), 0, 255);
	//cvui::trackbar(frame_, x, y + 50, 220, manager_->get_points_color(1), 0, 255);
	//cvui::trackbar(frame_, x, y + 100, 220, manager_->get_points_color(2), 0, 255);

	//coordinate transformation
	x = 300; y = 100;
	cvui::text(frame_, x, y, "dx");
	cvui::text(frame_, x + 100, y, "dy");
	cvui::text(frame_, x + 200, y, "dz");
	//cvui::text(frame_, x + 290, y, "angle");
	x -= 240;  y += 25;
	cvui::text(frame_, x, y, "translation");
	x += 200; y += -5;

	cvui::counter(frame_, x, y, manager_->get_transformation_parameters(0), delta); //0.001
	cvui::counter(frame_, x + 100, y, manager_->get_transformation_parameters(1), delta);
	cvui::counter(frame_, x + 200, y, manager_->get_transformation_parameters(2), delta);

	x += 40; y += 30;
	cvui::text(frame_, x, y, "ax");
	cvui::text(frame_, x + 100, y, "ay");
	cvui::text(frame_, x + 200, y, "az");
	//cvui::text(frame_, x + 290, y, "angle");
	x -= 240;  y += 25;
	cvui::text(frame_, x, y, "rotation");
	x += 200; y += -5;
	cvui::counter(frame_, x + 0, y, manager_->get_transformation_parameters(3), delta * 1);//0.1
	cvui::counter(frame_, x + 100, y, manager_->get_transformation_parameters(4), delta * 1);//0.1
	cvui::counter(frame_, x + 200, y, manager_->get_transformation_parameters(5), delta * 1);//0.1


	
	//grid lines
	//x = 50; y = 290;
	//cvui::text(frame_, x, y, "grid");
	//x += 100; y -= 10;
	//cvui::text(frame_, x, y, "right");
	//x += 130;
	//cvui::checkbox(frame_, x, y, "x", manager_->get_grids_enabled(0));
	//cvui::checkbox(frame_, x + 100, y, "y", manager_->get_grids_enabled(1));
	//cvui::checkbox(frame_, x + 200, y, "z", manager_->get_grids_enabled(2));
	//x -= 130; y += 20;
	//cvui::text(frame_, x, y, "left");
	//x += 130;
	//cvui::checkbox(frame_, x, y, "x", manager_->get_grids_enabled(3));
	//cvui::checkbox(frame_, x + 100, y, "y", manager_->get_grids_enabled(4));
	//cvui::checkbox(frame_, x + 200, y, "z", manager_->get_grids_enabled(5));
	
	//pcl
	x = 30; y = 200;
	cvui::checkbox(frame_, x, y, "use PCL", manager_->get_use_PCL());
	if (manager_->get_use_PCL()) {
		//PassThrough Filter
		x = 30; y = 250;
		cvui::checkbox(frame_, x, y, "passthrough filter", manager_->get_passthrough_filter_enabled());
		if (*manager_->get_passthrough_filter_enabled()) {
			x += 270; y -= 17;
			cvui::text(frame_, x, y, "sx");
			cvui::text(frame_, x + 100, y, "sy");
			cvui::text(frame_, x + 200, y, "sz");
			x -= 40; y += 20;
			cvui::counter(frame_, x, y, manager_->get_thresholds(1), delta);
			cvui::counter(frame_, x + 100, y, manager_->get_thresholds(3), delta);
			cvui::counter(frame_, x + 200, y, manager_->get_thresholds(5), delta);
			y += 25;
			cvui::counter(frame_, x, y, manager_->get_thresholds(0), delta);
			cvui::counter(frame_, x + 100, y, manager_->get_thresholds(2), delta);
			cvui::counter(frame_, x + 200, y, manager_->get_thresholds(4), delta);
		}
	}

	//show window
	cvui::imshow(WINDOW2_NAME, frame_);
	cv::waitKey(1);


}

bool GUI::check_loop() {

	return true;
}