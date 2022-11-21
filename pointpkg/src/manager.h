//operator_side_manager.h
#pragma once

#include <ros_interface.h>
#include <coppeliasim_interface.h>
#include <pcl2.h>
#include <realsense_interface.h>

#include <memory>
#include <vector>
#include <queue>

#ifdef _WIN32
#include <windows.h>
#else

#endif //_WIN32


class Manager {
private:
	typedef pcl::PointXYZRGBA PointType;

	//std::unique_ptr<PointCloudReceiver> point_cloud_receiver_;
	std::unique_ptr<ROSInterface> ros_interface_;
	//std::unique_ptr<PointCloudShower> point_cloud_shower_;
	std::unique_ptr<CoppeliaSimInterface> coppeliasim_interface_;
	std::unique_ptr<PCL2<PointType> > pcl_;
	std::unique_ptr<RealSenseInterface> realsense_;

	std::vector<float> points_;
	std::vector<int> color_;

	bool showTime_ = false;



	int points_size_;

	//int receiver_color_option_ = POINTS_COLOR_COLORFUL;
	bool use_PCL_ = true;
	
public:
	//Manager(const Manager&) = delete;
	Manager();
	void update();
	bool check_loop() {
		return ros_interface_->check_loop()
			&& coppeliasim_interface_->check_loop()
		#ifdef _WIN32
			&& !(bool)(GetKeyState(VK_ESCAPE) & 0x8000)
		#else

		#endif //_WIN32
			;
	}

	//ROS
	int get_original_points_size();
	int get_points_size();
	std::vector<float> get_points();
	double* get_transformation_parameters(int n);
	double* get_thresholds(int n);
	bool* get_passthrough_filter_enabled();

	//CoppeliaSim
	bool* get_grids_enabled(int n);
	int* get_points_color(int n);

	//PCL
	bool* get_use_PCL() { return &use_PCL_; }

};