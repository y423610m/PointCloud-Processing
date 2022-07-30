//point_cloud_receiver
#pragma once

#include <vector>


////The class derived by ROSInterface
//class PointCloudReceiver {
//protected:
//
//public:
//	virtual ~PointCloudReceiver() {}
//	virtual void update(int& number_of_points, std::vector<float>& points, std::vector<int>& color, int color_option) {};
//	virtual void finalize() {}
//	virtual bool check_loop() { return true; }
//
//	virtual double* get_transformation_parameters(int n) { return nullptr; };
//	virtual double* get_thresholds(int n) { return nullptr; };//passthrough
//	virtual bool* get_passthrough_filter_enabled() { return nullptr; };//passthrough
//	virtual int get_original_points_size() { return 0; };//gui
//	virtual int get_points_size() { return 0; };//gui
//	virtual float* get_points() { return nullptr; };//pointer to array
//	virtual int* get_points_color(int n) { return nullptr; }
//
//};