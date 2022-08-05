//coppeliasim_interface.h
#pragma once
#ifndef COP_FUNC_MAIN
#define COP_FUNC_MAIN 0
#endif

#ifndef COP_FUNC_APPEARED
#define COP_FUNC_APPEARED 1
#endif

#ifndef COP_FUNC_DISAPPEARED
#define COP_FUNC_DISAPPEARED 2
#endif

#include <point_cloud_shower.h>

extern "C" {
#include "extApi.h"
}

#include "labview_interface.h"
#include <memory>

#include <string>
#include <queue>
#include <thread>
#include <vector>
#include <mutex>

class CoppeliaSimInterface{
private:


	int ClientID = -1;
	int PortNumber = 19998;
	std::string simIP = "127.0.0.1";
	int counter = 0;
	bool grids_enabled_[6] = { true, true, true, true, true, true };
	int initialized_ = 0;

	////////-----for common use--------////////
	//_showCloud()ï¿óÒèàóùóp
	std::vector<std::thread> threads_;
	std::mutex mtx_;

	int robot_type_ = 0;
	
	//load read parameters
	bool params_read_ = false;


	////////-----for Cobotta------////////////////
	bool CobottaInitialized = false;
	void _initCobotta();
	void _updateCobotta();
	int ReferenceHandle;
	int RemoverHandle = -1;
	int RTipHandle = -1;
	int LTipHandle = -1;
	int LTipReferenceHandle = -1;
	float R_remover_pose[7];// init pcd data tip pose
	float L_remover_pose[7];
	float R_tip_pose[7];// real time robot arm tip pose
	float L_tip_pose[7];

	////////-----for MSRobot------////////////////
	bool MSRobotInitialized = false;
	void _initMSRobot();
	void _updateMSRobot();
	void _setTipPoseMSRobot();
	void _getTipPoseMSRobot();

	//_tip_position
	int ParallelLink_Tip = -1;
	int RemoverTip = -1;

	//set tip pose
	int ArmAxisJoint1=-1;
	int ArmAxisJoint2=-1;
	int ArmAxisJoint3=-1;
	int ArmAxisJoint4=-1;
	int ParallelLink_TipBase = -1;
	std::vector<float> poseParallelLinkTip;
	std::vector<float> poseRemoverTip;
	//_set_tool_color
	int ToolHandle;
	//communicate with LabView
	std::unique_ptr<LabViewInterface> labview_interface_;






	void _connect();
	void _load_parameters(std::string file_path);
	void _save_parameters(std::string file_path);
	void _showCloud(int number_of_points, std::vector<float>& points, std::vector<int>& color, int option_function);
	void _getPointsFromSim(int& number_of_points, std::vector<float>& points, std::vector<int>& color);
	void _set_tool_color();


public:
	CoppeliaSimInterface();
	~CoppeliaSimInterface();
	void update(int& number_of_points, std::vector<float>& points, std::vector<int>& color, int option_function);
	bool check_loop();

	bool* get_grids_enabled(int n) { return &grids_enabled_[n]; }
	std::vector<float> getLeftTipPose();//m, rad
	std::vector<float> getRemoverTipPose();//m, rad
	//float* get_R_tip_position(){ return R_tip_position; }
	//float* get_L_tip_position(){ return L_tip_position; }
	//float* get_R_tip_orientation(){ return R_tip_orientation; }
	//float* get_L_tip_orientation(){ return L_tip_orientation; }
};

