//ros_interface.cpp
#include <ros_interface.h>
#include <iostream>
#include <fstream>
//#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <numeric>

#include "ros_param.h"
#include "development_commands.h"

ROSInterface::ROSInterface() 
{
	std::cerr << "ros_interface: constructing" << std::endl;


	transformation_parameters_[0] = 1.515*0.;
	transformation_parameters_[1] = -0.197*0.;
	transformation_parameters_[2] = 0.554;
	transformation_parameters_[3] = 28.87;
	threshold_[0] = 0.15;
	threshold_[1] = 0.15;
	threshold_[2] = 0.089;

	rate = ROSParam::getIntParam("MAIN_rate");
	delay = ROSParam::getDoubleParam("ROS_delay");
	use_color_ = ROSParam::getIntParam("ROS_use_color");

	ROSInterface::load_parameters(ROSParam::getStringParam("ROS_param_txt"));

	//ros_sub_pointcloud = nh.subscribe("/camera/depth/color/points", 1, &ROSInterface::ros_CB, this);
	ros_sub_pointcloud = nh_.subscribe("/camera/depth/color/points", 2, &ROSInterface::ros_CB, this);
	//ros_sub_pointcloud = nh.subscribe("/tdmaker_for_pc/pointcloud/XYZ_p", 1, &ROSInterface::ros_CB, this);

	std::cerr << "ros_interface: constructed" << std::endl;

}

void ROSInterface::ros_CB(const sensor_msgs::PointCloud2& pCloud) {
	sensor_msgs::PointCloud2* tmp = new sensor_msgs::PointCloud2;
	*tmp = pCloud;
	queue_pc_.push(tmp);
	//EL(queue_pc_.size())
}

void ROSInterface::update(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	//座標変換あり
	//_set_cloud(number_of_points, points, color, option_color);
	//座標変換なし
	_set_cloud2(number_of_points, points, color);
}

/* TO DO
threshold, trans parameter for filter
*/
void  ROSInterface::_set_cloud(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {

	//std::cerr << queue_pc_.size() << std::endl;
	points.clear();
	color.clear();
	Rdist = 5.0;
	Ldist = 5.0;
	if (queue_pc_.size() > 0 && queue_pc_.front() != nullptr) {

		original_points_size_ = queue_pc_.front()->width * queue_pc_.front()->height;

		int arrayPosition, arrayPosX, arrayPosY, arrayPosZ, arrayPosRGB;
		unsigned int i, j;
		union { int i; float f; } x;
		union { int i; float f; } y;
		union { int i; float f; } z;



		if (passthrough_filter_enabled_) { //passthrough filter 有効時
			float xmax = -2.0;
			float xmin = 2.0;
			float zmax = -2.0;
			float zmin = 2.0;
			//std::cerr << queue_pc_.front()->width << std::endl;
			for (i = 0; i < queue_pc_.front()->height; i++) {
				for (j = 0; j < (queue_pc_.front()->width / 20) * 20; j++) {
					arrayPosition = i * queue_pc_.front()->row_step + j * queue_pc_.front()->point_step;
					arrayPosX = arrayPosition + queue_pc_.front()->fields[0].offset; // X has an offset of 0
					arrayPosY = arrayPosition + queue_pc_.front()->fields[1].offset; // Y has an offset of 4
					arrayPosZ = arrayPosition + queue_pc_.front()->fields[2].offset; // Z has an offset of 8
					arrayPosRGB = arrayPosition + queue_pc_.front()->fields[3].offset;//rgb 16
					x.i = 0;  y.i = 0;	z.i = 0;
					x.i = queue_pc_.front()->data[arrayPosX + 3] << 24 | queue_pc_.front()->data[arrayPosX + 2] << 16 | queue_pc_.front()->data[arrayPosX + 1] << 8 | queue_pc_.front()->data[arrayPosX + 0] << 0;
					y.i = queue_pc_.front()->data[arrayPosY + 3] << 24 | queue_pc_.front()->data[arrayPosY + 2] << 16 | queue_pc_.front()->data[arrayPosY + 1] << 8 | queue_pc_.front()->data[arrayPosY + 0] << 0;
					z.i = queue_pc_.front()->data[arrayPosZ + 3] << 24 | queue_pc_.front()->data[arrayPosZ + 2] << 16 | queue_pc_.front()->data[arrayPosZ + 1] << 8 | queue_pc_.front()->data[arrayPosZ + 0] << 0;

					float X = -x.f + transformation_parameters_[0];
					float Y = y.f * cos(M_PI * transformation_parameters_[3] / 180.0) + z.f * sin(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[1];
					float Z = y.f * sin(M_PI * transformation_parameters_[3] / 180.0) - z.f * cos(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[2];

					//passthrough filter
					if (abs(x.f) < threshold_[0] && Y < threshold_[1] && Z > threshold_[2]) {
						points.push_back(-X);
						points.push_back(-Y);
						points.push_back(Z);

						//_update_dist(X, Y, Z);


						if (use_color_) {
							color.push_back(queue_pc_.front()->data[arrayPosRGB + 2]);//r
							color.push_back(queue_pc_.front()->data[arrayPosRGB + 1]);//g
							if (queue_pc_.front()->data[arrayPosRGB + 0] >= 0) {
								color.push_back(queue_pc_.front()->data[arrayPosRGB + 0]);
							}
							else {//青系協調
								color.push_back(255);
								xmax = std::max(xmax, -x.f);
								xmin = std::min(xmin, -x.f);
								zmax = std::max(zmax, float(y.f * sin(M_PI * transformation_parameters_[3] / 180.0) - z.f * cos(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[2]));
								zmin = std::min(zmin, float(y.f * sin(M_PI * transformation_parameters_[3] / 180.0) - z.f * cos(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[2]));
							}
						}
						else {

							color.push_back(points_color_[0]);
							color.push_back(points_color_[1]);
							color.push_back(points_color_[2]);

						}
					}
				}
			}
			//std::cerr << "  xwidth  " << xmax - xmin << std::endl;
			//std::cerr << "  zmax " << zmax << std::endl;
			//std::cerr << std::endl;

		}
		else {//passthrough filter 無効時
			for (i = 0; i < queue_pc_.front()->height; i++) {
				for (j = 0; j < queue_pc_.front()->width; j++) {
					arrayPosition = i * queue_pc_.front()->row_step + j * queue_pc_.front()->point_step;
					arrayPosX = arrayPosition + queue_pc_.front()->fields[0].offset; // X has an offset of 0
					arrayPosY = arrayPosition + queue_pc_.front()->fields[1].offset; // Y has an offset of 4
					arrayPosZ = arrayPosition + queue_pc_.front()->fields[2].offset; // Z has an offset of 8
					arrayPosRGB = arrayPosition + queue_pc_.front()->fields[3].offset;
					x.i = 0;  y.i = 0;	z.i = 0;
					x.i = queue_pc_.front()->data[arrayPosX + 3] << 24 | queue_pc_.front()->data[arrayPosX + 2] << 16 | queue_pc_.front()->data[arrayPosX + 1] << 8 | queue_pc_.front()->data[arrayPosX + 0] << 0;
					y.i = queue_pc_.front()->data[arrayPosY + 3] << 24 | queue_pc_.front()->data[arrayPosY + 2] << 16 | queue_pc_.front()->data[arrayPosY + 1] << 8 | queue_pc_.front()->data[arrayPosY + 0] << 0;
					z.i = queue_pc_.front()->data[arrayPosZ + 3] << 24 | queue_pc_.front()->data[arrayPosZ + 2] << 16 | queue_pc_.front()->data[arrayPosZ + 1] << 8 | queue_pc_.front()->data[arrayPosZ + 0] << 0;

					float X = -x.f + transformation_parameters_[0];
					float Y = y.f * cos(M_PI * transformation_parameters_[3] / 180.0) + z.f * sin(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[1];
					float Z = y.f * sin(M_PI * transformation_parameters_[3] / 180.0) - z.f * cos(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[2];

					points.push_back(-X);
					points.push_back(-Y);
					points.push_back(Z);

					//_update_dist(X,Y,Z);

					if (use_color_) {
						color.push_back(queue_pc_.front()->data[arrayPosRGB + 2]);
						color.push_back(queue_pc_.front()->data[arrayPosRGB + 1]);
						color.push_back(queue_pc_.front()->data[arrayPosRGB + 0]);
					}
					else {
						color.push_back(points_color_[0]);
						color.push_back(points_color_[1]);
						color.push_back(points_color_[2]);
					}
				}
			}
		}
		//std::cerr << color_[0] << "   "			<< color_[1] << "   "			<< color_[2] << "   " << std::endl;

		//
		while (queue_pc_.size() > rate * delay + 0.01) {
			delete queue_pc_.front();
			queue_pc_.pop();
		}
		number_of_points = (int)points.size() / 3;
		//points = points_;
		//color = color_;

		//std::cerr << Ldist << " " << Rdist << std::endl;


	}
	//std::cerr << "ros ppoints          "<<*ppoints <<"    size "<< number_of_points << std::endl;
}

void  ROSInterface::_set_cloud2(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {

	//std::cerr << queue_pc_.size() << std::endl;
	points.clear();
	color.clear();
	Rdist = 5.0;
	Ldist = 5.0;
	if (queue_pc_.size() > 0 && queue_pc_.front() != nullptr) {

		original_points_size_ = queue_pc_.front()->width * queue_pc_.front()->height;

		int arrayPosition, arrayPosX, arrayPosY, arrayPosZ, arrayPosRGB;
		unsigned int i, j;
		union { int i; float f; } x;
		union { int i; float f; } y;
		union { int i; float f; } z;
		
		for (i = 0; i < queue_pc_.front()->height; i++) {
			for (j = 0; j < queue_pc_.front()->width; j++) {
				arrayPosition = i * queue_pc_.front()->row_step + j * queue_pc_.front()->point_step;
				arrayPosX = arrayPosition + queue_pc_.front()->fields[0].offset; // X has an offset of 0
				arrayPosY = arrayPosition + queue_pc_.front()->fields[1].offset; // Y has an offset of 4
				arrayPosZ = arrayPosition + queue_pc_.front()->fields[2].offset; // Z has an offset of 8
				arrayPosRGB = arrayPosition + queue_pc_.front()->fields[3].offset;
				x.i = 0;  y.i = 0;	z.i = 0;
				x.i = queue_pc_.front()->data[arrayPosX + 3] << 24 | queue_pc_.front()->data[arrayPosX + 2] << 16 | queue_pc_.front()->data[arrayPosX + 1] << 8 | queue_pc_.front()->data[arrayPosX + 0] << 0;
				y.i = queue_pc_.front()->data[arrayPosY + 3] << 24 | queue_pc_.front()->data[arrayPosY + 2] << 16 | queue_pc_.front()->data[arrayPosY + 1] << 8 | queue_pc_.front()->data[arrayPosY + 0] << 0;
				z.i = queue_pc_.front()->data[arrayPosZ + 3] << 24 | queue_pc_.front()->data[arrayPosZ + 2] << 16 | queue_pc_.front()->data[arrayPosZ + 1] << 8 | queue_pc_.front()->data[arrayPosZ + 0] << 0;

				float X = x.f;
				float Y = y.f;
				float Z = z.f;

				//if (abs(x.f) < threshold_[0] && abs(Y) < threshold_[1]){// && abs(Z) < threshold_[2]) {
				if(
					   threshold_[0]<x.f && x.f<threshold_[1]
					&& threshold_[2] < y.f && y.f < threshold_[3]
					&& threshold_[4] < z.f && z.f < threshold_[5]
					){


					points.push_back(X);
					points.push_back(Y);
					points.push_back(Z);

					//_update_dist(X,Y,Z);

					if (use_color_) {
						color.push_back(queue_pc_.front()->data[arrayPosRGB + 2]);
						color.push_back(queue_pc_.front()->data[arrayPosRGB + 1]);
						color.push_back(queue_pc_.front()->data[arrayPosRGB + 0]);
					}
					else {
						color.push_back(points_color_[0]);
						color.push_back(points_color_[1]);
						color.push_back(points_color_[2]);
					}
				}
			}
		}
	}
	//std::cerr << color_[0] << "   "			<< color_[1] << "   "			<< color_[2] << "   " << std::endl;

	//
	while (queue_pc_.size() > (int)(rate * delay + 0.01)+1) {
		delete queue_pc_.front();
		queue_pc_.pop();
	}
	number_of_points = (int)points.size() / 3;
	//EL(points.size())
	//points = points_;
	//color = color_;

	//std::cerr << Ldist << " " << Rdist << std::endl;


	
	//std::cerr << "ros ppoints          "<<*ppoints <<"    size "<< number_of_points << std::endl;
}

void ROSInterface::load_parameters(std::string file_path) {

	std::ifstream ifs(file_path);
	std::stringstream ss;
	std::string line;
	//std::getline(ifs, line);

	std::string parameter_name;
	std::string value;
	while (getline(ifs, line)) {
		std::istringstream i_stream(line);
		std::getline(i_stream, parameter_name, ' ');
		std::getline(i_stream, value);
		//std::cerr << parameter_name << " " << value << std::endl;

		if (parameter_name == "trans_x")   transformation_parameters_[0] = stof(value);
		if (parameter_name == "trans_y")   transformation_parameters_[1] = stof(value);
		if (parameter_name == "trans_z")   transformation_parameters_[2] = stof(value);
		if (parameter_name == "trans_ax")   transformation_parameters_[3] = stof(value);
		if (parameter_name == "trans_ay")   transformation_parameters_[4] = stof(value);
		if (parameter_name == "trans_az")   transformation_parameters_[5] = stof(value);

		if (parameter_name == "thresh_min_x")   threshold_[0] = stof(value);
		if (parameter_name == "thresh_max_x")   threshold_[1] = stof(value);
		if (parameter_name == "thresh_min_y")   threshold_[2] = stof(value);
		if (parameter_name == "thresh_max_y")   threshold_[3] = stof(value);
		if (parameter_name == "thresh_min_z")   threshold_[4] = stof(value);
		if (parameter_name == "thresh_max_z")   threshold_[5] = stof(value);

	}
	params_read_ = true;
}

void ROSInterface::save_parameters(std::string file_path) {
	if (!params_read_) return;

	std::ofstream ofs(file_path);
	ofs << std::fixed << std::setprecision(6);


	for (int i = 0; i < 6; i++) if (abs(transformation_parameters_[i]) < 0.00001) transformation_parameters_[i] = 0.0;
	ofs << "trans_x " << transformation_parameters_[0] << std::endl;
	ofs << "trans_y " << transformation_parameters_[1] << std::endl;
	ofs << "trans_z " << transformation_parameters_[2] << std::endl;
	ofs << "trans_ax " << transformation_parameters_[3] << std::endl;
	ofs << "trans_ay " << transformation_parameters_[4] << std::endl;
	ofs << "trans_az " << transformation_parameters_[5] << std::endl;

	for (int i = 0; i < 6; i++) if (abs(threshold_[i]) < 0.00001) threshold_[i] = 0.0;
	ofs << "thresh_min_x " << threshold_[0] << std::endl;
	ofs << "thresh_max_x " << threshold_[1] << std::endl;
	ofs << "thresh_min_y " << threshold_[2] << std::endl;
	ofs << "thresh_max_y " << threshold_[3] << std::endl;
	ofs << "thresh_min_z " << threshold_[4] << std::endl;
	ofs << "thresh_max_z " << threshold_[5] << std::endl;
	ofs.close();

}

ROSInterface::~ROSInterface() {
	ROSInterface::save_parameters(ROSParam::getStringParam("ROS_param_txt"));
	//std::cerr << "qqqqq" << std::endl;
}


void ROSInterface::_update_dist(double x, double y, double z) {
	double dist = std::sqrt((LeftPose[0] - x) * (LeftPose[0] - x) + (LeftPose[1] - y) * (LeftPose[1] - y) + (LeftPose[2] - z) * (LeftPose[2] - z));
	Ldist = std::min(Ldist, dist);
	dist = std::sqrt((RightPose[0] - x) * (RightPose[0] - x) + (RightPose[1] - y) * (RightPose[1] - y) + (RightPose[2] - z) * (RightPose[2] - z));
	Rdist = std::min(Rdist, dist);
}



//compressed image をopencvに表示
/*
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
	try
	{
		std::cerr << msg->data[0] << std::endl;
		cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);//convert compressed image data to cv::Mat
		std::cerr << " image    "<<image.size() << std::endl;

		cv::imshow("view", image);

		cv::waitKey(10);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert to image!");
	}
}
#include <ctime>
*/

//opencv test
/*
	cv::namedWindow("view");
	cv::startWindowThread();
	ros::Subscriber sub = nh.subscribe("/camera/depth/image_rect_raw/compressed", 1, imageCallback);
	ros::spin();
	cv::destroyWindow("view");
*/

//rosCB 不採用
/*
void ROSInterface::ros_CB(const sensor_msgs::PointCloud2 pCloud) {

	original_points_size_ = pCloud.width * pCloud.height;

	//ros_pointcloud = pCloud;
	//std::cerr<<"asfasgfasdfg"<<std::endl;
	//std::cerr << pCloud.fields[1].offset << "    ros CB" << std::endl;
	int arrayPosition, arrayPosX, arrayPosY, arrayPosZ;
	unsigned int i, j;
	union { int i; float f; } x;
	union { int i; float f; } y;
	union { int i; float f; } z;
	points_.clear();
	if (passthrough_filter_enabled_) { //passthrough filter 有効時
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
				if (abs(x.f) < threshold_[0]) { //x
					if (abs(y.f * cos(M_PI * transformation_parameters_[3] / 180.0) + z.f * sin(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[1]) < threshold_[1]) { //y
						if (y.f * sin(M_PI * transformation_parameters_[3] / 180.0) - z.f * cos(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[2] > threshold_[2]) { //z
							points_.push_back(-x.f + transformation_parameters_[0]);
							points_.push_back(y.f * cos(M_PI * transformation_parameters_[3] / 180.0) + z.f * sin(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[1]);
							points_.push_back(y.f * sin(M_PI * transformation_parameters_[3] / 180.0) - z.f * cos(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[2]);
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

				points_.push_back(-x.f + transformation_parameters_[0]);
				points_.push_back(y.f * cos(M_PI * transformation_parameters_[3] / 180.0) + z.f * sin(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[1]);
				points_.push_back(y.f * sin(M_PI * transformation_parameters_[3] / 180.0) - z.f * cos(M_PI * transformation_parameters_[3] / 180.0) + transformation_parameters_[2]);
			}
		}
	}
	//LastPhase = eROS;
}
*/


// ROSを介さずにRealSenseをあつかう
/*
int main()
{
	//Contruct a pipeline which abstracts the device
	rs2::pipeline pipe;

	//Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;

	//Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	//Instruct pipeline to start streaming with the requested configuration
	pipe.start(cfg);

	// Camera warmup - dropping several first frames to let auto-exposure stabilize
	rs2::frameset frames;
	for (int i = 0; i < 30; i++)
	{
		//Wait for all configured streams to produce a frame
		frames = pipe.wait_for_frames();
	}

	//Get each frame
	//rs2::frame color_frame = frames.get_color_frame();
	while (1) {
		frames = pipe.wait_for_frames();

		rs2::frame ir_frame = frames.first(RS2_STREAM_INFRARED);
		rs2::frame depth_frame = frames.get_depth_frame();

		auto tmp = (void*)ir_frame.get_data();
		std::cerr << tmp << std::endl;

		// Creating OpenCV Matrix from a color image
		//Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
		Mat ir(Size(640, 480), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);

		// Display in a GUI
		//namedWindow("Display Image", WINDOW_AUTOSIZE);
		//imshow("Display Image", color);

		// Apply Histogram Equalization
		equalizeHist(ir, ir);
		applyColorMap(ir, ir, COLORMAP_JET);

		// Display the image in GUI
		namedWindow("Display Image ", WINDOW_AUTOSIZE);
		imshow("Display Image", ir);

		waitKey(1);
	}
	return 0;
}
*/