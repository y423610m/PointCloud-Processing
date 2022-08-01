#include <ros/ros.h>

#include<string>

class ROSParam {
	static ros::NodeHandle* pnh;
	static std::string namespace_;
	static std::string nodename_;

public:
	static void init();
	static int getIntParam(std::string param, bool addNameSpace = false, bool addNodeName = true);
	static float getFloatParam(std::string param, bool addNameSpace = false, bool addNodeName = true);
	static std::string getStringParam(std::string param, bool addNameSpace = false, bool addNodeName = true);
};