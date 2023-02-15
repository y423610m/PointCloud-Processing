/*

git push https://github.com/y423610m/PointCloud-Processing.git develop
 
*/




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

#include "manager.h"
#include "gui.h"
#include "ros_param.h"
#include "development_commands.h"


#include "ros_param.h"


int main(int argc, char** argv) {

	//try {

		ros::init(argc, argv, "pointer");
		ROSParam::init();

		std::cout << std::fixed << std::setprecision(3);
		std::cerr << std::fixed << std::setprecision(3);

		ros::Rate rate(ROSParam::getIntParam("MAIN_rate"));

		std::unique_ptr<Manager> manager(new Manager());
		std::unique_ptr<GUI> gui(new GUI(manager.get()));


		bool showTime = false;
		showTime = ROSParam::getIntParam("MAIN_ShowTime");
		auto time_last = clock();
		auto time_now = time_last;
		while (manager->check_loop()) {// && gui->check_loop()) {

			manager->update();
			gui->update();

			ros::spinOnce();
			rate.sleep();
			if (showTime) {
				time_now = clock();
				ES("MAIN") EL(time_now - time_last);
				time_last = time_now;
			}
		}

		// Manager manager(nh);
		// manager.loop();

	//}
	//catch (std::exception& e) {
	//	PS("~RS") EL(e.what())
	//}
     
	std::cerr << "program ended successfully" << std::endl;

   return 0;
}


