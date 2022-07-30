#pragma once
#include <ros/ros.h>
#include <client.h>



class Manager {
private:
	int count;
	Client client;

public:
	Manager(ros::NodeHandle& nh);
	void loop();
	~Manager();

};
