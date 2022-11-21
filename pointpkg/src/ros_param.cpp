#include "ros_param.h"
#include "development_commands.h"

ros::NodeHandle* ROSParam::pnh;
std::string ROSParam::namespace_;
std::string ROSParam::nodename_;

void ROSParam::init() {
	pnh = new ros::NodeHandle("~");
	namespace_ = ros::this_node::getNamespace();
	nodename_ = ros::this_node::getName();
	PL("ROS Param init")
	EL(namespace_)
	EL(nodename_)
}

int ROSParam::getIntParam(std::string param, bool addNameSpace, bool addNodeName) {
	if (addNameSpace) param = namespace_ + "/" + param;
	if (addNodeName) param = nodename_ + "/" + param;

	int val = 998244353;
	pnh->getParam(param, val);
	if (val == 998244353) {
		ES(param) PL("Not found")
		val = 0;
	}
	PS(param) PL(val)
	return  val;
}

double ROSParam::getDoubleParam(std::string param, bool addNameSpace, bool addNodeName) {
	if (addNameSpace) param = namespace_ + "/" + param;
	if (addNodeName) param = nodename_ + "/" + param;

	float val = -998244353.;
	pnh->getParam(param, val);
	if (val == -998244353.) {
		ES(param) PL("Not found")
		val = 0.;
	}
	PS(param) PL(val)
	return  val;
}

std::string ROSParam::getStringParam(std::string param, bool addNameSpace, bool addNodeName) {
	if (addNameSpace) param = namespace_ + "/" + param;
	if (addNodeName) param = nodename_ + "/" + param;

	std::string val = "-";
	pnh->getParam(param, val);
	if (val == "-") {
		ES(param) PL("Not found")
		val = "";
	}
	PS(param) PL(val)
	return  val;
}