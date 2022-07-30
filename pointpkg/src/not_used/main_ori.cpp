#include <iostream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pointer.h>
#include <config.h>

#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

#define WINDOW1_NAME "Window 1"



extern "C" {
#include "extApi.h"
}


//for coppeliasim
int ClientID;

double angle = ANGLE;
double dx = dX;
double dy = dY;
double dz = dZ;

//CallBack Function 3Dpoint
void CB_PixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud){

	int arrayPosition, arrayPosX, arrayPosY, arrayPosZ;
	unsigned int i, j;
	union { int i; float f; } x;
	union { int i; float f; } y;
	union { int i; float f; } z;
	int count = 0;
	int count_red = 0;
	int judge = 0;
	//float position[3*POINT_NUMBER] = {};
	//float position_red[3*POINT_NUMBER] = {};

	//std::cout << "test 2" << std::endl;

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
			//std::cout << "test 4" << std::endl;
			if (abs(x.f) < 0.15 && abs(y.f) < 0.2 && abs(z.f) < 0.7) {
			//if (abs(x.f-0.05) < 0.005 && abs(y.f-0.05) < 0.005 && abs(z.f) < 0.7) {
			//if (j == 10000) {
				if (y.f * sin(PI * ANGLE / 180) - z.f * cos(PI * ANGLE / 180) + dZ > 0.1) { //red
					position_red[3 * (count_red % POINT_NUMBER)] = -x.f + dX;
					position_red[3 * (count_red % POINT_NUMBER) + 1] = y.f * cos(PI * ANGLE / 180) + z.f * sin(PI * ANGLE / 180) + dY;
					position_red[3 * (count_red % POINT_NUMBER) + 2] = +y.f * sin(PI * ANGLE / 180) - z.f * cos(PI * ANGLE / 180) + dZ;
					if (position_red[3 * (count_red % POINT_NUMBER) + 2] < 0.25) {
						count_red++;
					}
				}
				else { //white
					position[3 * (count % POINT_NUMBER)] = -x.f + dX;
					position[3 * (count % POINT_NUMBER) + 1] = y.f * cos(PI * ANGLE / 180) + z.f * sin(PI * ANGLE / 180) + dY;
					position[3 * (count % POINT_NUMBER) + 2] = +y.f * sin(PI * ANGLE / 180) - z.f * cos(PI * ANGLE / 180) + dZ;
					//count++;
					judge++;
				}
				//std::cout << x.f << "    " << y.f << "   " << z.f << std::endl;
			}
			
			//std::cout << "test 5" << std::endl;
			if (j == pCloud.width - 1) {
				std::cout << count << std::endl;
				std::cout << count_red << std::endl;
				//std::cout << "test 3   " <<count<<"    "<<count_red<< std::endl;
				if (count_red > 0) {
					int result_red = simxCallScriptFunction(ClientID, "Point_cloud", sim_scripttype_childscript, "MyResetPointcloud_red", 0, NULL, 3 * count_red, position_red, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
				}
				else {
					int result_red = simxCallScriptFunction(ClientID, "Point_cloud", sim_scripttype_childscript, "MyResetPointcloud_red_NoData", 0, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
				}

					int result_ = simxCallScriptFunction(ClientID, "Point_cloud", sim_scripttype_childscript, "MyResetPointcloud", 0, NULL, 3 * count, position, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
					//std::cout << count << std::endl;
					//std::cout << count_red << std::endl;
				
				count = 0;
				count_red = 0;
				judge = 0;
			}
			
		}
	}
}





void CB_CameraCalibration(const sensor_msgs::PointCloud2 pCloud) {

	int arrayPosition, arrayPosX, arrayPosY, arrayPosZ;
	unsigned int i, j;
	union { int i; float f; } x;
	union { int i; float f; } y;
	union { int i; float f; } z;
	int count = 1;
	float sum_x, sum_y, sum_z, sum_yz, sum_yy, sum_1;
	float SUM_Y, SUM_Z;
	sum_x = 0.0;  sum_y = 0.0; sum_z = 0.0; sum_yz = 0.0;  sum_yy = 0.0, sum_1 = 0.0;  SUM_Y = 0.0;  SUM_Z = 0.0;
	float sum_zz = 0.0;


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
			
			//std::cout << "                 " << angle << "   " << x.f << "   " << y.f << "    " << z.f << std::endl;

			

			if (abs(x.f) < 0.15) {
				if (abs(-y.f * cos(PI * angle / 180) - z.f * sin(PI * angle / 180) + dy) < 0.145 + 0.05/count ) {//
					if (y.f * sin(PI * angle / 180) - z.f * cos(PI * angle / 180) + dz > 0.088 - 0.05/count && y.f * sin(PI * angle / 180) - z.f * cos(PI * angle / 180) + dz < 0.96 + 0.05 / count) {//
						sum_x += x.f;
						sum_y += y.f;
						sum_z += z.f;
						sum_yz += y.f * z.f;
						sum_yy += y.f * y.f;
						sum_1 += 1;
						SUM_Y += -y.f * cos(PI * angle / 180) - z.f * sin(PI * angle / 180) + dy;
						SUM_Z += y.f * sin(PI * angle / 180) - z.f * cos(PI * angle / 180) + dz;

						sum_zz += z.f * z.f;
					}
				} 
			}
			
			if (j == pCloud.width - 1) {
				//std::cout << " angle = " << angle;
				angle = atan((sum_1 * sum_yz - sum_y * sum_z) / (sum_yy * sum_1 - sum_y * sum_y)) * 180 / PI;
				//std::cout << " angle = " << angle;
				//angle = 2.4;
				dx = -sum_x / sum_1;
				dy += -SUM_Y / sum_1;
				dz += 0.092 - SUM_Z / sum_1;
				std::cout << " angle = " << angle << "    dx = " << dx << "    dy = " << dy << "     dz = " << dz << std::endl;
				//std::cout << tan(2.4*PI/180) << std::endl;
				//std::cout << "•ªŽU" << sum_zz-2*sum_ << "    " << dx << "    y = " << y.f << "     z = " << z.f << std::endl;
			}	

			count++;
		}
	}
}



///////     main function
int main(int argc, char* argv[]) {
/////////////////////// coppeliasim /////////////////////////
	simxChar* simIP = SIM_IP;
	int PortNumber = SIM_PORTNUMBER;
	ClientID = -1;
	std::cout << "" << std::endl;
	std::cout << " connecting to coppeliasim..." << std::endl;
	std::cout << " please start the simulation " << std::endl;
	while (ClientID == -1) { ClientID = simxStart(simIP, PortNumber, true, true, 2000, 5); }
	std::cout << "!!  connected  !!" << std::endl;
	std::cout << "" << std::endl;


/////////////////////// ros /////////////////////////////////

	ros::init(argc, argv, "pointer");
	ros::NodeHandle nh;
	ros::Subscriber	sub_pointcloud_pose_detection;
	ros::Subscriber	sub_pointcloud_msgs;

	if (MODE == 0) {
		sub_pointcloud_pose_detection = nh.subscribe("/camera/depth/color/points", 1, &CB_CameraCalibration);
		std::cout << "  CAMERA POSE DETECTION MODE  " << std::endl;
	}
	else if(MODE == 1){
		sub_pointcloud_msgs = nh.subscribe("/camera/depth/color/points", 1, &CB_PixelTo3DPoint);
		std::cout << "  POINT CLOUD MODE  " << std::endl;
	}
	else if (MODE == 2) {
		std::cout << "  VIRTUAL REALSENSE MODE  " << std::endl;
	}

/////////////////////   initialize    /////////////////////////
	int count = 0;
	//Pointer pointer;
	//pointer.initialize(ClientID);


////////////////////    main loop    ///////////////////////////
	ros::Rate rate(ROS_RATE);
	
	//while (ros::ok() && simxGetConnectionId(ClientID) != -1) {
	while (ros::ok()) {
		if (MODE == 2) {
				//std::cout << "myao---" << std::endl;
				//pointer.loop(ClientID);
		}

		ros::spinOnce();
		rate.sleep();
		count++;
	}

/////////////////////    finalize     ////////////////////////
	std::cout << "    Program ended    " << std::endl;
	simxFinish(ClientID);
   return 0;
}

