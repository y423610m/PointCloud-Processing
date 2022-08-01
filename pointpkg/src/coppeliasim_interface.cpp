//coppeliasim_interface.cpp
#include <coppeliasim_interface.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <windows.h>


//for Eigen
//#include <pcl/common/transforms.h>
#include "Eigen/Core"
#include <Eigen/Geometry>

#include "ros_param.h"
#include "development_commands.h"

CoppeliaSimInterface::CoppeliaSimInterface() {
	std::cerr << "coppeliasim_interface: constructing" << std::endl;
	CoppeliaSimInterface::_load_parameters(ROSParam::getStringParam("SIM_param_txt"));

	simIP = ROSParam::getStringParam("SIM_simIP");
	PortNumber = ROSParam::getIntParam("SIM_PortNumber");

	CoppeliaSimInterface::_connect();

	robot_type_ = ROSParam::getIntParam("COMMON_robot_type");
	if(robot_type_==1) CoppeliaSimInterface::_initCobotta();
	else if(robot_type_ == 2)CoppeliaSimInterface::_initMSRobot();

	std::cerr << "coppeliasim_interface: constructed" << std::endl;
}

CoppeliaSimInterface::~CoppeliaSimInterface() {
	simxFinish(ClientID);
	CoppeliaSimInterface::_save_parameters(ROSParam::getStringParam("SIM_param_txt"));
	labview_interface_->~LabViewInterface();
}

void CoppeliaSimInterface::_load_parameters(std::string file_path) {
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

		if (parameter_name == "simIP")   simIP = value;
		if (parameter_name == "PortNumber")   PortNumber = stoi(value);
		if (parameter_name == "grid_rx")   grids_enabled_[0] = stoi(value);
		if (parameter_name == "grid_ry")   grids_enabled_[1] = stoi(value);
		if (parameter_name == "grid_rz")   grids_enabled_[2] = stoi(value);
		if (parameter_name == "grid_lx")   grids_enabled_[3] = stoi(value);
		if (parameter_name == "grid_ly")   grids_enabled_[4] = stoi(value);
		if (parameter_name == "grid_lz")   grids_enabled_[5] = stoi(value);
	}
	params_read_ = true;
}

void CoppeliaSimInterface::_save_parameters(std::string file_path) {
	if (!params_read_) return;
	std::ofstream ofs(file_path);
	ofs << std::fixed << std::setprecision(6);


	ofs << "simIP " << simIP << std::endl;
	ofs << "PortNumber " << PortNumber << std::endl;

	ofs << "grid_rx " << grids_enabled_[0] << std::endl;
	ofs << "grid_ry " << grids_enabled_[1] << std::endl;
	ofs << "grid_rz " << grids_enabled_[2] << std::endl;
	ofs << "grid_lx " << grids_enabled_[3] << std::endl;
	ofs << "grid_ly " << grids_enabled_[4] << std::endl;
	ofs << "grid_lz " << grids_enabled_[5] << std::endl;
	ofs.close();
}

void CoppeliaSimInterface::_connect() {
	std::cerr << "" << std::endl;
	std::cerr << "(CoppeliaSimInterface.initialize())" << std::endl;
	std::cerr << "     connecting to coppeliasim..." << std::endl;
	std::cerr << "     please start the simulation " << std::endl;
	while (ClientID == -1) {
		std::cerr << "     ClientID  " << ClientID << std::endl;
		ClientID = simxStart(&simIP[0], PortNumber, true, true, 2000, 5);
	}
	std::cerr << "     !!  connected  !!" << std::endl;
}

void CoppeliaSimInterface::update(int& number_of_points, std::vector<float>& points, std::vector<int>& color, int option_function = COP_FUNC_MAIN) {
	//並列処理join
	while(!threads_.empty()){
		threads_.back().join();
		threads_.pop_back();
	}

	//<=thread使っても実行時間あまり変化なし
	if(robot_type_==1) CoppeliaSimInterface::_updateCobotta();
	else if(robot_type_ == 2) CoppeliaSimInterface::_updateMSRobot();


	threads_.push_back(std::thread([&]() {
		std::lock_guard<std::mutex> lock(mtx_);
		_showCloud(number_of_points, points, color, option_function);
	}));

	//_showCloud(number_of_points, points, color, option_function);

	//_getPointsFromSim(number_of_points, points, color);
	//_set_tool_color();

	//std::cerr << "cop update" << std::endl;
	counter++;
}

void CoppeliaSimInterface::_showCloud(int number_of_points, std::vector<float>& points, std::vector<int>& color, int option_function = COP_FUNC_MAIN) {
	int n = 3 * number_of_points;
	int n_c = 3 * number_of_points;
	if (points.size() == 0) {
		n = 0;
		//std::cerr << "cop points nullptr " << std::endl;
	}
	if (color.size() == 0) {
		n_c = 0;
		//std::cerr << "cop color nullptr " << std::endl;
	}

	if (n > 0) {
		if (option_function == COP_FUNC_MAIN) {

			//double a = clock();
			simxCallScriptFunction(ClientID, "Point_Cloud_Main", sim_scripttype_childscript, "ShowPointCloud", color.size(), &color[0], points.size(), &points[0], 0, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
			//std::cerr<< "coppeliasim   "<<double(clock()-a)/1000 <<std::endl;

		}
		else if (option_function == COP_FUNC_APPEARED) {
			//simxCallScriptFunction(ClientID, "PointClouds", sim_scripttype_childscript, "ShowAppearedPointCloud", 0, NULL, n, &points[0], 0, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
		}
	}
}

void CoppeliaSimInterface::_initCobotta() {


	CobottaInitialized = true;
	std::cerr << "CoppeliaSimInterface initialized for Cobotta" << std::endl;
}

void CoppeliaSimInterface::_updateCobotta() {
	if (!CobottaInitialized) {
		std::cerr << "CoppeliaSimInterface  for Cobotta  not initialized" << std::endl;
		return;
	}




}

void CoppeliaSimInterface::_initMSRobot() {

	if (!ROSParam::getIntParam("SIM_LABView_connect")) {
		cerr << "don't connect to LabView for MSRobot" << endl;
		return;
	}

	labview_interface_.reset(new LabViewInterface(ROSParam::getStringParam("SIM_LabView_IP").c_str(), ROSParam::getIntParam("SIM_LabView_Port")));

	//simxGetObjectHandle(ClientID, "ToolColor", &ToolHandle, simx_opmode_blocking);

	//for pcl::remover, get Tip Pose
	simxGetObjectHandle(ClientID, "ParallelLink_Tip", &ParallelLink_Tip, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "RemoverTip", &RemoverTip, simx_opmode_blocking);
	poseParallelLinkTip = std::vector<float>(7);
	poseRemoverTip = std::vector<float>(7);
	//simxGetObjectPosition(ClientID, RemoverTip, -1, &poseRemoverTip[0], simx_opmode_oneshot);
	//simxGetObjectOrientation(ClientID, RemoverTip, -1, &poseRemoverTip[3], simx_opmode_oneshot);

	//set tip pose
	simxGetObjectHandle(ClientID, "ParallelLink_Tip_Reference", &ReferenceHandle, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "Arm_Axis_Joint1", &ArmAxisJoint1, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "Arm_Axis_Joint2", &ArmAxisJoint2, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "Arm_Axis_Joint3", &ArmAxisJoint3, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "Arm_Axis_Joint4", &ArmAxisJoint4, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "ParallelLink_TipBase", &ParallelLink_TipBase, simx_opmode_blocking);

	MSRobotInitialized = true;

	std::cerr << "CoppeliaSimInterface initialized for MSRobot" << std::endl;
}

void CoppeliaSimInterface::_updateMSRobot() {
	if (!MSRobotInitialized) {
		//std::cerr << "CoppeliaSimInterface  for MSRobot  not initialized" << std::endl;
		return;
	}



	//LabViewから実機姿勢取得，CoppeliaSimへ指示
	labview_interface_->update();
	_setTipPoseMSRobot();

	//絶対座標に対するツール姿勢取得
	_getTipPoseMSRobot();



}

void CoppeliaSimInterface::_getTipPoseMSRobot() {

	simxGetObjectPosition(ClientID, ParallelLink_Tip, -1, &poseParallelLinkTip[0], simx_opmode_oneshot);
	simxGetObjectOrientation(ClientID, ParallelLink_Tip, -1, &poseParallelLinkTip[3], simx_opmode_oneshot);

	simxGetObjectPosition(ClientID, RemoverTip, -1, &poseRemoverTip[0], simx_opmode_oneshot);
	simxGetObjectOrientation(ClientID, RemoverTip, -1, &poseRemoverTip[3], simx_opmode_oneshot);

}

void CoppeliaSimInterface::_getPointsFromSim(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	int n_points, n_color;
	int* color_;
	float* points_;
	simxCallScriptFunction(ClientID, "tool", sim_scripttype_childscript, "getPoints", 0, NULL, 0, NULL, 0, NULL, 0, NULL, &n_color, &color_, &n_points, &points_, NULL, NULL, NULL, NULL, simx_opmode_blocking);
	std::cerr << "coppeliasim" << std::endl;
	//std::cerr << points[0] << std::endl;
	int CNT = 0;
	points.clear();
	color.clear();
	for (int i = 0; i < n_points; i++) { 
		points.push_back(points_[i]);
		color.push_back(color_[i%3]);
	}
	//number_of_points = 1;// n_points / 3;
	number_of_points = n_points / 3;

	std::cerr << points.size() << std::endl;
}

void CoppeliaSimInterface::_set_tool_color() {
	float position[3] = { 0., 0., 0. };
	simxSetObjectPosition(ClientID, ToolHandle, -1, position, simx_opmode_oneshot);
}

void CoppeliaSimInterface::_setTipPoseMSRobot() {
	std::vector<float> vf = labview_interface_->get_tip_pose();
	// xyzoat, rot1, rot2, mm, rot3

	//---vf 6,7,8,9 アーム角度指令
	simxSetJointPosition(ClientID, ArmAxisJoint1, vf[6], simx_opmode_oneshot);
	simxSetJointPosition(ClientID, ArmAxisJoint2, vf[7], simx_opmode_oneshot);
	simxSetJointPosition(ClientID, ArmAxisJoint3, vf[8], simx_opmode_oneshot);
	simxSetJointPosition(ClientID, ArmAxisJoint4, vf[9], simx_opmode_oneshot);

	if (vf[3] == 0.0 && vf[4] == 0.0){
		std::cerr << "labview_interface::_set_tip_pose: " << "TCP 0.0 error" << std::endl;
		return;
	}

	//---vf 0,1,2 鉗子先端並進
	simxSetObjectPosition(ClientID, LTipHandle, ParallelLink_TipBase, &vf[0], simx_opmode_oneshot);


	//---vf 3,4,5
	Eigen::Quaterniond q(
		  Eigen::AngleAxisd(vf[3], Eigen::Vector3d::UnitZ()) 
		* Eigen::AngleAxisd(vf[4], Eigen::Vector3d::UnitY()) 
		* Eigen::AngleAxisd(vf[5], Eigen::Vector3d::UnitZ()) 
	); 

	float Q[4] = { q.x(), q.y(), q.z(), q.w() };
	simxSetObjectOrientation(ClientID, LTipHandle, ParallelLink_TipBase, Q, simx_opmode_oneshot);

}

bool CoppeliaSimInterface::check_loop() {
	if (simxGetConnectionId(ClientID) == -1) return false;
	return true;
}

std::vector<float> CoppeliaSimInterface::getLeftTipPose() {
	if(MSRobotInitialized) return poseParallelLinkTip;

	std::cerr << "CoppeliaSimInterface::getLeftTipPose(): " << "Nothing was initialized"<< std::endl;
	return poseParallelLinkTip;
}

std::vector<float> CoppeliaSimInterface::getRemoverTipPose() {
	//std::cerr << RemoverTip<<poseRemoverTip[0] << poseRemoverTip[1] << poseRemoverTip[2] << std::endl;
	if (MSRobotInitialized) return poseRemoverTip;

	std::cerr << "CoppeliaSimInterface::getRemoverTipPose: " << "Nothing was initialized" << std::endl;
	return poseRemoverTip;
}
