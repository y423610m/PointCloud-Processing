//Client
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <string.h>

#include <client.h>

#define WINDOW1_NAME "Window 1   Press ESCAPE Key to Stop This Program"
#define CVUI_IMPLEMENTATION
#include "cvui.h"

extern "C" {
#include "extApi.h"
}


////////////////////////////////////////////////////////////////////
/////////////////////////  ROS  ////////////////////////////////////
////////////////////////////////////////////////////////////////////

void Client::ros_initialize(ros::NodeHandle& nh) {
	//std::cout << "Client initialize" << std::endl;
	ros_sub_pointcloud = nh.subscribe("/camera/depth/color/points", 1, &Client::ros_CB, this);
}

void Client::ros_CB(const sensor_msgs::PointCloud2 pCloud) {
	std::cout << "ros cb " << std::endl;

	//std::cout << "Client" << std::endl;
	//ros_pointcloud = pCloud;
	//std::cout << ros_pointcloud.fields[1].offset << std::endl;
	int arrayPosition, arrayPosX, arrayPosY, arrayPosZ;
	unsigned int i, j;
	union { int i; float f; } x;
	union { int i; float f; } y;
	union { int i; float f; } z;
	original_data_size = pCloud.width * pCloud.height;
	points.clear();
	if (passthrough_filter_enabled) { //passthrough filter —LŒøŽž
		for (i = 0; i < pCloud.height; i++) {
			for (j = 0; j < pCloud.width; j++) {
				arrayPosition = i * pCloud.row_step + j * pCloud.point_step;
				arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
				arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
				arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
				x.i = 0;  y.i = 0;	z.i = 0;
				x.i = pCloud.data[arrayPosX + 3] << 24 | pCloud.data[arrayPosX + 2] << 16 | pCloud.data[arrayPosX + 1] << 8 | pCloud.data[arrayPosX + 0] << 0;
				y.i = pCloud.data[arrayPosY + 3] << 24 | pCloud.data[arrayPosY + 2] << 16 | pCloud.data[arrayPosY + 1] << 8 | pCloud.data[arrayPosY + 0] << 0;
				z.i = pCloud.data[arrayPosZ + 3] << 24 | pCloud.data[arrayPosZ + 2] << 16 | pCloud.data[arrayPosZ + 1] << 8 | pCloud.data[arrayPosZ + 0] << 0;

				//passthrough filter
				if (abs(x.f) < sx) { //x
					if (abs(y.f * cos(M_PI * angle / 180.0) + z.f * sin(M_PI * angle / 180.0) + dy) < sy) { //y
						if (y.f * sin(M_PI * angle / 180.0) - z.f * cos(M_PI * angle / 180.0) + dz > sz) { //z
							points.push_back(-x.f + dx);
							points.push_back(y.f * cos(M_PI * angle / 180.0) + z.f * sin(M_PI * angle / 180.0) + dy);
							points.push_back(y.f * sin(M_PI * angle / 180.0) - z.f * cos(M_PI * angle / 180.0) + dz);
						}
					}
				}
			}
		}
	}
	else {
		for (i = 0; i < pCloud.height; i++) {
			for (j = 0; j < pCloud.width; j++) {
				arrayPosition = i * pCloud.row_step + j * pCloud.point_step;
				arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
				arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
				arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
				x.i = 0;  y.i = 0;	z.i = 0;
				x.i = pCloud.data[arrayPosX + 3] << 24 | pCloud.data[arrayPosX + 2] << 16 | pCloud.data[arrayPosX + 1] << 8 | pCloud.data[arrayPosX + 0] << 0;
				y.i = pCloud.data[arrayPosY + 3] << 24 | pCloud.data[arrayPosY + 2] << 16 | pCloud.data[arrayPosY + 1] << 8 | pCloud.data[arrayPosY + 0] << 0;
				z.i = pCloud.data[arrayPosZ + 3] << 24 | pCloud.data[arrayPosZ + 2] << 16 | pCloud.data[arrayPosZ + 1] << 8 | pCloud.data[arrayPosZ + 0] << 0;

				points.push_back(-x.f + dx);
				points.push_back(y.f* cos(M_PI* angle / 180.0) + z.f * sin(M_PI * angle / 180.0) + dy);
				points.push_back(y.f* sin(M_PI* angle / 180.0) - z.f * cos(M_PI * angle / 180.0) + dz);
			}
		}
	}
	//LastPhase = eROS;
}

void Client::ros_initialize_PCL(ros::NodeHandle& nh) {
	std::cout << "ros initialize" << std::endl;
	ros_sub_pointcloud = nh.subscribe("/camera/depth/color/points", 1, &Client::ros_CB_PCL, this);
	points.push_back(1.0);
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_(new pcl::visualization::PCLVisualizer("Octree compression"));
	//viewer = viewer_;
}

void Client::ros_CB_PCL(const sensor_msgs::PointCloud2 pCloud) {
	std::cout << "" << std::endl;
	points.clear();
	n = 0;
	double a = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr decompressedCloud(new pcl::PointCloud<pcl::PointXYZ>);

	int arrayPosition, arrayPosX, arrayPosY, arrayPosZ;
	unsigned int i, j;
	union { int i; float f; } x;
	union { int i; float f; } y;
	union { int i; float f; } z;

	//std::cout << "morning" << std::endl;


	if (passthrough_filter_enabled) { //passthrough filter —LŒøŽž
		for (i = 0; i < pCloud.height; i++) {
			for (j = 0; j < pCloud.width; j++) {
				arrayPosition = i * pCloud.row_step + j * pCloud.point_step;
				arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
				arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
				arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
				x.i = 0;  y.i = 0;	z.i = 0;
				x.i = pCloud.data[arrayPosX + 3] << 24 | pCloud.data[arrayPosX + 2] << 16 | pCloud.data[arrayPosX + 1] << 8 | pCloud.data[arrayPosX + 0] << 0;
				y.i = pCloud.data[arrayPosY + 3] << 24 | pCloud.data[arrayPosY + 2] << 16 | pCloud.data[arrayPosY + 1] << 8 | pCloud.data[arrayPosY + 0] << 0;
				z.i = pCloud.data[arrayPosZ + 3] << 24 | pCloud.data[arrayPosZ + 2] << 16 | pCloud.data[arrayPosZ + 1] << 8 | pCloud.data[arrayPosZ + 0] << 0;

				//passthrough filter
				if (abs(x.f) < sx) { //x
					if (abs(y.f * cos(M_PI * angle / 180.0) + z.f * sin(M_PI * angle / 180.0) + dy) < sy) { //y
						if (y.f * sin(M_PI * angle / 180.0) - z.f * cos(M_PI * angle / 180.0) + dz > sz) { //z

							n++;
							/*
							points.push_back(-x.f + dx);
							points.push_back(y.f * cos(M_PI * angle / 180.0) + z.f * sin(M_PI * angle / 180.0) + dy);
							points.push_back(y.f * sin(M_PI * angle / 180.0) - z.f * cos(M_PI * angle / 180.0) + dz);
							*/
						}
					}
				}
			}
		}
		//std::cout << "petit dejouner" << std::endl;
		cloud->width = n;
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width* cloud->height);
		n = 0;
		for (i = 0; i < pCloud.height; i++) {
			for (j = 0; j < pCloud.width; j++) {
				arrayPosition = i * pCloud.row_step + j * pCloud.point_step;
				arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
				arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
				arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
				x.i = 0;  y.i = 0;	z.i = 0;
				x.i = pCloud.data[arrayPosX + 3] << 24 | pCloud.data[arrayPosX + 2] << 16 | pCloud.data[arrayPosX + 1] << 8 | pCloud.data[arrayPosX + 0] << 0;
				y.i = pCloud.data[arrayPosY + 3] << 24 | pCloud.data[arrayPosY + 2] << 16 | pCloud.data[arrayPosY + 1] << 8 | pCloud.data[arrayPosY + 0] << 0;
				z.i = pCloud.data[arrayPosZ + 3] << 24 | pCloud.data[arrayPosZ + 2] << 16 | pCloud.data[arrayPosZ + 1] << 8 | pCloud.data[arrayPosZ + 0] << 0;

				//passthrough filter
				if (abs(x.f) < sx) { //x
					if (abs(y.f * cos(M_PI * angle / 180.0) + z.f * sin(M_PI * angle / 180.0) + dy) < sy) { //y
						if (y.f * sin(M_PI * angle / 180.0) - z.f * cos(M_PI * angle / 180.0) + dz > sz) { //z

							float px, py, pz;
							px = -x.f + dx;
							py = y.f * cos(M_PI * angle / 180.0) + z.f * sin(M_PI * angle / 180.0) + dy;
							pz = y.f * sin(M_PI * angle / 180.0) - z.f * cos(M_PI * angle / 180.0) + dz;
							cloud->points[n] = pcl::PointXYZ(px, py, pz);
							n++;

						}
					}
				}
			}
		}
	}
	else {
		//std::cout << "dejouner" << std::endl;
		cloud->width = pCloud.width;
		cloud->height = pCloud.height;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width* cloud->height);
		for (i = 0; i < pCloud.height; i++) {
			for (j = 0; j < pCloud.width; j++) {
				//std::for_each(std::begin(cloud), std::end(cloud),[](int i) {
				arrayPosition = i * pCloud.row_step + j * pCloud.point_step;
				arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
				arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
				arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
				x.i = 0;  y.i = 0;	z.i = 0;
				x.i = pCloud.data[arrayPosX + 3] << 24 | pCloud.data[arrayPosX + 2] << 16 | pCloud.data[arrayPosX + 1] << 8 | pCloud.data[arrayPosX + 0] << 0;
				y.i = pCloud.data[arrayPosY + 3] << 24 | pCloud.data[arrayPosY + 2] << 16 | pCloud.data[arrayPosY + 1] << 8 | pCloud.data[arrayPosY + 0] << 0;
				z.i = pCloud.data[arrayPosZ + 3] << 24 | pCloud.data[arrayPosZ + 2] << 16 | pCloud.data[arrayPosZ + 1] << 8 | pCloud.data[arrayPosZ + 0] << 0;

				float px, py, pz;
				px = -x.f + dx;
				py = y.f * cos(M_PI * angle / 180.0) + z.f * sin(M_PI * angle / 180.0) + dy;
				pz = y.f * sin(M_PI * angle / 180.0) - z.f * cos(M_PI * angle / 180.0) + dz;

				cloud->points[pCloud.width * i + j] = pcl::PointXYZ(px, py, pz);
				/*
				points.push_back(-x.f + dx);
				points.push_back(y.f * cos(M_PI * angle / 180.0) + z.f * sin(M_PI * angle / 180.0) + dy);
				points.push_back(y.f * sin(M_PI * angle / 180.0) - z.f * cos(M_PI * angle / 180.0) + dz);
				*/
				//}
				n++;
			}
		}
	}



	// Read a PCD file from disk.
	//pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\y4236\\Documents\\tryws\\src\\pclpkg\\pcl\\test\\rops_cloud.pcd", *cloud);
	double b = clock();

	// Octree compressor object.
	// Check /usr/include/pcl-<version>/pcl/compression/compression_profiles.h for more profiles.
	// The second parameter enables the output of information.
	bool bo;
	if (cnt % 10 == 0)bo = true;
	if (cnt % 10 != 0)bo = false;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octreeCompression(pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, bo);
	// Stringstream that will hold the compressed cloud.
	std::stringstream compressedData;

	// Compress the cloud (you would save the stream to disk).
	octreeCompression.encodePointCloud(cloud, compressedData);
	double c = clock();

	// Decompress the cloud.
	octreeCompression.decodePointCloud(compressedData, decompressedCloud);
	double d = clock();

	// Display the decompressed cloud.
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Octree compression"));
	//viewer = new pcl::visualization::PCLVisualizer("Octree compression");
	/*
	viewer->addPointCloud<pcl::PointXYZ>(decompressedCloud, "cloud");
	*/
	std::cout << "     load pcd " << (b - a)
		<< "ms    compression " << (c - b)
		<< "ms    decompression " << (d - c)
		<< "ms"
		<< std::endl;
	/*
	if (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	*/
	//delete cloud;
	//delete(compressedData);
	//delete(decompressedCloud);]
	
	//std::cout << "hello" << std::endl;
	//LastPhase = eROS;

	//float ff;
	//std::cout << decompressedCloud->points[1].x;
	//points.push_back(decompressedCloud->points[1]);
	for (int i = 0; i < n; i++) {

		points.push_back(decompressedCloud->points[i].x);
		points.push_back(decompressedCloud->points[i].y);
		points.push_back(decompressedCloud->points[i].z);

	}
	std::cout << points.size() << std::endl;
}

////////////////////////////////////////////////////////////////////
///////////////////////    PCL    //////////////////////////////////
////////////////////////////////////////////////////////////////////
void Client::pcl_initialize() {
}

void Client::pcl_update() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\y4236\\Documents\\tryws\\src\\pclpkg\\pcl\\test\\rops_cloud.pcd", *cloud) == -1) //* load the file
	{
		//PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		//return (-1);
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
	int cnt = 0;
	for (const auto& point : *cloud) {
		cnt++;
		//std::cout << "    " << point.x
			//<< " " << point.y
			//<< " " << point.z << std::endl;


	}
	
}

void Client::pcl_finalize() {

}

void Client::pcl_compression() {
	double a = clock();
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr decompressedCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\y4236\\Documents\\tryws\\src\\pclpkg\\pcl\\test\\rops_cloud.pcd", *cloud);
	double b = clock();

	// Octree compressor object.
	// Check /usr/include/pcl-<version>/pcl/compression/compression_profiles.h for more profiles.
	// The second parameter enables the output of information.
	bool bo;
	if (cnt % 10 == 0)bo = true;
	if (cnt % 10 != 0)bo = false;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octreeCompression(pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false);
	// Stringstream that will hold the compressed cloud.
	std::stringstream compressedData;

	// Compress the cloud (you would save the stream to disk).
	octreeCompression.encodePointCloud(cloud, compressedData);
	double c = clock();

	// Decompress the cloud.
	octreeCompression.decodePointCloud(compressedData, decompressedCloud);
	double d = clock();

	// Display the decompressed cloud.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Octree compression"));
	viewer->addPointCloud<pcl::PointXYZ>(decompressedCloud, "cloud");

	std::cout << "     load pcd " << (b - a)
		<< "ms    compression " << (c - b)
		<< "ms    decompression " << (d - c)
		<< "ms"
		<< std::endl;
	if (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

////////////////////////////////////////////////////////////////////
//////////////////////  processer  /////////////////////////////////
////////////////////////////////////////////////////////////////////

//add_grid functions are not used any more since they were moved to coppeliasim's child script
void Client::pro_add_grid_initialize() {
	simxGetObjectHandle(ClientID, "Reference#0", &reference_handle, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "Joint_RAab#0", &right_tip_handle, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "Joint_LAab#0", &left_tip_handle, simx_opmode_blocking);
	simxGetObjectPosition(ClientID, right_tip_handle, -1, right_position, simx_opmode_streaming);
	simxGetObjectPosition(ClientID, left_tip_handle, -1, left_position, simx_opmode_streaming);
}

void Client::pro_add_grid_update() {
	//get tip position
	simxGetObjectPosition(ClientID, right_tip_handle, -1, right_position, simx_opmode_buffer);
	simxGetObjectPosition(ClientID, left_tip_handle, -1, left_position, simx_opmode_buffer);
	//distance of points 
	float step = 0.003;
	//right
	//x
	if (use_grid_right_x) {
		for (int i = 0; i * step < 2 * sx; i++) {
			points.push_back(i * step - sx+dx);//+dx?
			points.push_back(right_position[1]);
			points.push_back(right_position[2]);
		}
	}

	//y
	if (use_grid_right_y) {
		for (int i = 0; i * step < 2 * sy; i++) {
			points.push_back(right_position[0]);//+dx?
			points.push_back(i * step - sy);
			points.push_back(right_position[2]);
		}
	}

	//z
	if (use_grid_right_z) {
		for (int i = 0; i * step + sz < 0.2; i++) {
			points.push_back(right_position[0]);//+dx?
			points.push_back(right_position[1]);
			points.push_back(i * step + sz);
		}
	}
	//left
    //x
	if (use_grid_left_x) {
		for (int i = 0; i * step < 2 * sx; i++) {
			points.push_back(i * step - sx+dx);//+dx?
			points.push_back(left_position[1]);
			points.push_back(left_position[2]);
		}
	}

	//y
	if (use_grid_left_y) {
		for (int i = 0; i * step < 2 * sy; i++) {
			points.push_back(left_position[0]);//+dx?
			points.push_back(i * step - sy);
			points.push_back(left_position[2]);
		}
	}

	//z
	if (use_grid_left_z) {
		for (int i = 0; i * step + sz < 0.2; i++) {
			points.push_back(left_position[0]);//+dx?
			points.push_back(left_position[1]);
			points.push_back(i * step + sz);
		}
	}
}


////////////////////////////////////////////////////////////////////
/////////////////////  CoppeliaSim  ////////////////////////////////
////////////////////////////////////////////////////////////////////
void Client::cop_initialize() {
	std::cout << "" << std::endl;
	std::cout << "(Client.cop_initialize)" << std::endl;
	std::cout << "     connecting to coppeliasim..." << std::endl;
	std::cout << "     please start the simulation " << std::endl;
	while (ClientID == -1) { ClientID = simxStart(simIP, PortNumber, true, true, 2000, 5); }
	std::cout << "     !!  connected  !!" << std::endl;
}

void Client::cop_update() {
	bits.clear();
	bits.push_back(use_grid_right_x);
	bits.push_back(use_grid_right_y);
	bits.push_back(use_grid_right_z);
	bits.push_back(use_grid_left_x);
	bits.push_back(use_grid_left_y);
	bits.push_back(use_grid_left_z);


	final_data_size = (int)points.size();

		
	simxCallScriptFunction(ClientID, "PointClouds", sim_scripttype_childscript, "MyOutputData", (int)bits.size(), &bits[0], (int)points.size(), &points[0], 0, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);

		//int result_output = simxCallScriptFunction(ClientID, "RealSense_Depth", sim_scripttype_childscript, "MyOutputData",0,NULL, data_length_ros, data_ros, 0, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);

		//int result_input = simxCallScriptFunction(ClientID, "RealSense_Depth", sim_scripttype_childscript, "MyInputData", 0, NULL, data_length_ros, data_ros, 0, NULL, 0, NULL, 0, NULL, NULL,NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);

		//int result_red = simxCallScriptFunction(ClientID, "Point_cloud", sim_scripttype_childscript, "MyResetPointcloud_red", 0, NULL, data_length_ros, data_ros, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);

		//int result_red = simxCallScriptFunction(ClientID, "Point_cloud", sim_scripttype_childscript, "MyResetPointcloud_red_NoData", 0, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);


	//LastPhase = eNONE;
}

void Client::cop_finalize() {
	simxFinish(ClientID);
}

////////////////////////////////////////////////////////////////////
/////////////////////////  GUI  ////////////////////////////////////
////////////////////////////////////////////////////////////////////
void Client::gui_initialize() {
	cvui::init(WINDOW1_NAME);
	frame = cv::Mat(cv::Size(800, 400), CV_8UC3);
}

void Client::gui_update() {

	cnt++;

	frame = cv::Scalar(49, 52, 49);
	//size
	cvui::printf(frame, 50, 20, "original data size : %d", original_data_size);
	cvui::printf(frame, 50, 50, "output data size  : %d", points.size() / 3);
	int x, y;
	//coordinate transformation
	x = 290; y = 100;
	cvui::text(frame, x, y, "dx");
	cvui::text(frame, x + 100, y, "dy");
	cvui::text(frame, x + 200, y, "dz");
	cvui::text(frame, x + 290, y, "angle");
	x -= 240;  y += 25;
	cvui::text(frame, x, y, "transformation");
	x += 200; y += -5;
	cvui::counter(frame, x, y, &dx, 0.00100);
	cvui::counter(frame, x + 100, y, &dy, 0.00100);
	cvui::counter(frame, x + 200, y, &dz, 0.00100);
	cvui::counter(frame, x + 300, y, &angle, 0.100);

	//PassThrough Filter
	x = 20; y = 187;
	cvui::checkbox(frame, 20, 187, "passthrough filter", &passthrough_filter_enabled);
	if (passthrough_filter_enabled) {
		x += 270; y -= 27;
		cvui::text(frame, x, y, "sx");
		cvui::text(frame, x + 100, y, "sy");
		cvui::text(frame, x + 200, y, "sz");
		x -= 40; y += 20;
		cvui::counter(frame, x, y, &sx, 0.00100);
		cvui::counter(frame, x + 100, y, &sy, 0.00100);
		cvui::counter(frame, x + 200, y, &sz, 0.00100);
	}

	//grid lines
	x = 50; y = 250;
	cvui::text(frame, x, y, "grid");
	x += 100; y -= 10;
	cvui::text(frame, x, y, "right");
	x += 130;
	cvui::checkbox(frame, x, y, "x", &use_grid_right_x);
	cvui::checkbox(frame, x + 100, y, "y", &use_grid_right_y);
	cvui::checkbox(frame, x + 200, y, "z", &use_grid_right_z);
	x -= 130; y += 20;
	cvui::text(frame, x, y, "left");
	x += 130;
	cvui::checkbox(frame, x, y, "x", &use_grid_left_x);
	cvui::checkbox(frame, x + 100, y, "y", &use_grid_left_y);
	cvui::checkbox(frame, x + 200, y, "z", &use_grid_left_z);

	//show window
	cvui::imshow(WINDOW1_NAME, frame);
	cv::waitKey(1);
}

void Client::gui_finalize() {

}

////////////////////////////////////////////////////////////////////
//////////////////////  parameters  ////////////////////////////////
////////////////////////////////////////////////////////////////////
void Client::load_parameters() {
	FILE* fp = NULL;
	char parameter_name[10];
	double	value = 0.0;
	fp = fopen(file_path, "r");
	if (fp == NULL) {
		std::cout << "" << std::endl;
		std::cout << "(Client.load_parameters)" << std::endl;
		std::cout << "     parameters.txt was not found" << std::endl;
		std::cout << "     check the path in client.h " << std::endl;
		std::cout << "     this program continues with default parameters " << std::endl;
		//return -1;
	}
	else {
		while (fscanf(fp, "%s %lf", &parameter_name, &value) != EOF)
		{
			if (strcmp(parameter_name, "dx") == 0) dx = value;
			if (strcmp(parameter_name, "dy") == 0) dy = value;
			if (strcmp(parameter_name, "dz") == 0) dz = value;
			if (strcmp(parameter_name, "angle") == 0) angle = value;
			if (strcmp(parameter_name, "sx") == 0) sx = value;
			if (strcmp(parameter_name, "sy") == 0) sy = value;
			if (strcmp(parameter_name, "sz") == 0) sz = value;
		}
		fclose(fp);
	}
}

void Client::save_parameters() {
	FILE* fp = NULL;
	fp = fopen(file_path, "w");
	if (fp == NULL) {
		std::cout << "" << std::endl;
		std::cout << "(Client.save_parameters)" << std::endl;
		std::cout << "     parameters.txt was not found" << std::endl;
		std::cout << "     saving parameters failed " << std::endl;
		std::cout << "     check the path in client.h " << std::endl;
	}
	fprintf(fp, "dx %lf\n", dx);
	fprintf(fp, "dy %lf\n", dy);
	fprintf(fp, "dz %lf\n", dz);
	fprintf(fp, "angle %lf\n", angle);

	fprintf(fp, "sx %lf\n", sx);
	fprintf(fp, "sy %lf\n", sy);
	fprintf(fp, "sz %lf\n", sz);

	fclose(fp);
}


