#pragma once
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
//simple reas and output
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

extern "C" {
#include "extApi.h"
}

//compression
#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/compression/octree_pointcloud_compression.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

#ifdef WIN32
//# define sleep(x) Sleep((x)*1000)
#endif



typedef enum {
	eNONE,
	eROS,
	ePRO,
	ePCL,
	eCOP
} eLastPhase;

static eLastPhase LastPhase = eNONE;


class Client {
private:

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Octree compression"));
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	int n; //ros_cb_pcl

	int cnt;//gui_update

	std::vector<float> points;
	
	//pcl::PointCloud<pcl::PointXYZ> cloud;

	// ROS
	ros::Subscriber	ros_sub_pointcloud;

	//processer 
	int reference_handle, right_tip_handle, left_tip_handle;
	float right_position[3] = { 0.0,0.0,0.0 }; //tool tip position
	float left_position[3] = { 0.1,0.1,0.1 };
	bool use_grid_right_x = true;
	bool use_grid_right_y = true;
	bool use_grid_right_z = true;
	bool use_grid_left_x = true;
	bool use_grid_left_y = true;
	bool use_grid_left_z = true;

	//coppeliasaim
	int ClientID = -1;
	int PortNumber = 19998;
	simxChar* simIP = "127.0.0.1";
	std::vector<int> bits;


	//GUI
	cv::Mat frame;
	int original_data_size = 0;
	int final_data_size = 0;

	//parameters
	//char* file_path = "C:/Users/y4236/Documents/pcws/src/pointpkg/parameters.txt";
	char* file_path = "src/pointpkg/parameters.txt";
	double dx = 1.515;
	double dy = -0.197;
	double dz = 0.554;
	double angle = 28.87;

	bool passthrough_filter_enabled = false;
	double sx = 0.15;
	double sy = 0.15;
	double sz = 0.089;

public:
	// ROS
	void ros_initialize(ros::NodeHandle& nh);
	void ros_CB(const sensor_msgs::PointCloud2 pCloud);
	void ros_initialize_PCL(ros::NodeHandle& nh);
	void ros_CB_PCL(const sensor_msgs::PointCloud2 pCloud);

	//PCL
	void pcl_initialize();
	void pcl_update();
	void pcl_finalize();
	void pcl_compression();

	//processer
	void pro_add_grid_initialize();
	void pro_add_grid_update();

	//coppeliasim
	void cop_initialize();
	void cop_update();
	void cop_finalize();

	//GUI
	void gui_initialize();
	void gui_update();
	void gui_finalize();

	//parameter
	void load_parameters();
	void save_parameters();
};