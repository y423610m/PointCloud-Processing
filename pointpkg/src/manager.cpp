//operator_side_manager.cpp
#include <manager.h>
#include <iostream>
#include <vector>



Manager::Manager(ros::NodeHandle& nh, int rate) :
	ros_interface_(new ROSInterface(nh, rate)),
	coppeliasim_interface_(new CoppeliaSimInterface()),
	pcl_(new PCL(PCL_XYZRGBA))
{
	std::cout << "Manager constructed" << std::endl;
	pcl_->init(coppeliasim_interface_.get());
}

void Manager::update() {
	points_size_ = -1;
	points_.clear();
	color_.clear();



	/////////////////////PointCloudReceiver///////////////////////////////////////
	//RECEIVER_COLOR_SINGLE or RECEIVER_COLOR_COLORFUL are available for the 4th arguments
	ros_interface_->update(points_size_, points_, color_, RECEIVER_COLOR_COLORFUL);
	//std::cout << "ope ros" << std::endl;

	//ros setcloud z limit


	/////////////////////////////PCL//////////////////////////////////////////////
	pcl_->update(points_size_, points_, color_);
	//std::cout << "ope pcl" << std::endl;


	//////////////////////////PointCloudShower////////////////////////////////////
	coppeliasim_interface_->update(points_size_, points_, color_, COP_FUNC_MAIN);
	//std::cout << "ope cop" << std::endl;

	//coppeliasim_interface_->update(points_size_, points_, color_, COP_FUNC_APPEARED);
	//coppeliasim_interface_->update(pcl_->get_appeared_points_size(), pcl_->get_appeared_points(), nullptr,COP_FUNC_APPEARED);

}

double* Manager::get_transformation_parameters(int n) {
	//return ros_interface_->get_transformation_parameters(n);
	return pcl_->get_transformation_parameters(n);
}

double* Manager::get_thresholds(int n) {
	//return ros_interface_->get_thresholds(n);
	return pcl_->get_filterPassThrough_threshold(n);
}

bool* Manager::get_passthrough_filter_enabled() {
	//return ros_interface_->get_passthrough_filter_enabled();
	return pcl_->get_enable_filterPassThrough();
}

int Manager::get_original_points_size() {
	return ros_interface_->get_original_points_size();
}

int Manager::get_points_size() {
	return points_size_;
}

std::vector<float> Manager::get_points() {
	return points_;
}

bool* Manager::get_grids_enabled(int n) {
	return coppeliasim_interface_->get_grids_enabled(n);
}

int* Manager::get_points_color(int n) {
	return ros_interface_->get_points_color(n);
}