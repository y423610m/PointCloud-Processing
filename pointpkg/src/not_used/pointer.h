#pragma once
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>

extern "C" {
#include "extApi.h"
}


//get depth data array from coppeliasim, transform it into #D positions, then output pointcloud

class Pointer {
private:
	int count = 0;
	int count_pc = 0;

	
	float dx = 1.5;
	float dy = -0.294;
	float dz = 0.5445;
	float angle = 50.0 * 3.1415 / 180;
	float angle_camera = 31.3 * 3.1415 / 180;
	float dist_min = 0.25;
	float dist_max = 0.8;
	float position[3];

	simxFloat* input;
	simxInt number;



	int Rtt, Rtthelp, Ltt, Ltthelp;
	float position_Rtt[3];
	float position_Ltt[3];
	float position_Rtthelp[3];
	float position_Ltthelp[3];
	float zR, zL;

public:
	void initialize(int ClientID);
	void loop(int ClientID);
};