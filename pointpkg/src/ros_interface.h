//ros_interface.h
#pragma once

#ifndef RECEIVER_COLOR_COLORFUL
#define RECEIVER_COLOR_COLORFUL 0
#endif 

#ifndef RECEIVER_COLOR_SINGLE
#define RECEIVER_COLOR_SINGLE 1
#endif 

#include <queue>
#include <string>
//#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//#include <point_cloud_receiver.h>



//ROSからポイントクラウドを受け取り，遅延発生後＆座標変換する
class ROSInterface {

private:
	ros::NodeHandle nh_;
	std::vector<float> points_;
	std::vector<int> color_;
	std::queue<sensor_msgs::PointCloud2*> queue_pc_;
	int points_color_[3] = { 20,20,20 };
	double transformation_parameters_[6];//x,y,z,anglex,y,z
	double threshold_[6];//Xmin, Xmax, Ymin,...
	int original_points_size_;
	bool passthrough_filter_enabled_ = false;



	float delay = 0.0;
	int rate = 6;

	ros::Subscriber	ros_sub_pointcloud;
	//void ros_CB(const sensor_msgs::PointCloud2 pCloud);
	void ros_CB(const sensor_msgs::PointCloud2& pCloud);
	void load_parameters(std::string file_name);
	void save_parameters(std::string file_name);

	//最短ポイントとの距離計算
	double LeftPose[7];
	double RightPose[7];
	double Ldist = 1.0, Rdist = 1.0;
	void _update_dist(double x, double y, double z);

	//座標変換あり，thresholdあり
	void _set_cloud(int& number_of_points, std::vector<float>& points, std::vector<int>& color, int color_option);
	//シンプル版
	void _set_cloud2(int& number_of_points, std::vector<float>& points, std::vector<int>& color, int color_option);

	//load and read parameters
	bool params_read_ = false;

public:
	ROSInterface();
	~ROSInterface();
	void update(int& number_of_points, std::vector<float>& points, std::vector<int>& color, int color_option);
	bool check_loop() { return ros::ok(); }

	double* get_transformation_parameters(int n) { return &transformation_parameters_[n]; };
	double* get_thresholds(int n) { return &threshold_[n]; };//passthrough
	bool* get_passthrough_filter_enabled() { return &passthrough_filter_enabled_; };//passthrough
	int get_original_points_size() { return original_points_size_; };//gui
	int get_points_size() { return (int)(points_.size() / 3); };//gui
	float* get_points() { return &points_[0]; };//pointer to array
	int* get_points_color(int n) { return &points_color_[n]; }


};