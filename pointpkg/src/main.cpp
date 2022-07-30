#include <iostream>
#include <string>
#include <math.h>
#include <time.h>

#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>

//#include <pointer.h>
//#include <config.h>
//#include <not_used/manager.h>
//#include <not_used/client.h>
#include <Params.h>

#include "manager.h"
#include <gui.h>


extern "C" {
#include "extApi.h"
} 



int main(int argc, char** argv) {
	
	ros::init(argc, argv, "pointer");
	ros::NodeHandle nh;
	const int FPS = 100;
	ros::Rate rate(FPS);

	std::unique_ptr<Manager> manager(new Manager(nh, FPS));
	std::unique_ptr<GUI> gui(new GUI(manager.get()));


	while (manager->check_loop()){// && gui->check_loop()) {

		manager->update();
		gui->update();

		ros::spinOnce();
		rate.sleep();
	}
	
    // Manager manager(nh);
    // manager.loop();
     
	std::cout << "program ended successfully" << std::endl;

   return 0;
}



///////////////////////////////////////
