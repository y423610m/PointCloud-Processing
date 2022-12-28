#pragma once

#include <ros/ros.h>

#include<string>

class ROSParam {
	static ros::NodeHandle* pnh;
	static std::string namespace_;
	static std::string nodename_;
	~ROSParam() {
		std::cerr << "~ROSParam" << std::endl;
	}

public:
	static void init();
	static int getIntParam(std::string param, bool addNameSpace = false, bool addNodeName = true);
	static double getDoubleParam(std::string param, bool addNameSpace = false, bool addNodeName = true);
	static std::string getStringParam(std::string param, bool addNameSpace = false, bool addNodeName = true);
};