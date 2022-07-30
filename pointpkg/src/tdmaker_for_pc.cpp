//tdmaker_for_pc
#include <iostream>
#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

std::queue<sensor_msgs::PointCloud2*> qPC;
float delay = 5.0;
int fps = 10;
sensor_msgs::PointCloud2 PC;
sensor_msgs::PointCloud2* tmp;


void pc_CB(const sensor_msgs::PointCloud2 pCloud) {
	tmp = new sensor_msgs::PointCloud2;
	*tmp = pCloud;
	qPC.push(tmp);
	std::cout << qPC.front()->height << std::endl;

}


int main(int argc, char* argv[]) {

	ros::init(argc, argv, "tdmaker_for_pc");
	ros::NodeHandle nh;

	ros::Subscriber	ros_sub_pointcloud = nh.subscribe("/camera/depth/color/points", 1, &pc_CB);
	ros::Publisher ros_pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/tdmaker_for_pc/pointcloud/XYZ", 1);
	ros::Publisher ros_pub_pointcloud_p = nh.advertise<sensor_msgs::PointCloud2>("/tdmaker_for_pc/pointcloud/XYZ_p", 1);
	ros::spinOnce();


	//std::queue<sensor_msgs::PointCloud2> qqPC;
	
	ros::Rate rate(fps);
	while (ros::ok()) {
		

		while (qPC.size() > fps * delay + 1) {
			delete qPC.front();
			qPC.pop();
		}
		if (tmp != nullptr) {

			std::cout << qPC.front()->height << std::endl;
			ros_pub_pointcloud_p.publish(*qPC.front());
		}




		/*
		qqPC.push(PC);
		while (qqPC.size() > fps * delay + 1) {
			qqPC.pop();
		}
		ros_pub_pointcloud.publish(qqPC.front());
		*/

		ros::spinOnce();
		rate.sleep();

	}

	return 0;
}