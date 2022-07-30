//manager
#include <iostream>
#include <windows.h>
#include <manager.h>


Manager::Manager(ros::NodeHandle &nh) {
	std::cout << "" << std::endl;
	std::cout << "(Manager.Manager)" << std::endl;
	std::cout << "     manager constructer" << std::endl;
	//client.ros_initialize(nh);
	client.ros_initialize_PCL(nh);
	client.pcl_initialize();
	client.gui_initialize();
	client.cop_initialize();
	client.load_parameters();
}

void Manager::loop() {
	std::cout << "" << std::endl;
	std::cout << "(Manager.loop)" << std::endl;
	std::cout << "     manager loop" << std::endl;

	ros::Rate rate(5);
	while (ros::ok()) {
		if (GetKeyState(VK_ESCAPE) & 0x8000) {	break;	}


		//std::cout << "" << std::endl;


				
		client.gui_update();

		//client.pcl_update();
		//client.pcl_compression();
		client.cop_update();






		ros::spinOnce();
		rate.sleep();
		count++;
	}
}

Manager::~Manager() {
	std::cout << "" << std::endl;
	std::cout << "(Manager.~Manager)" << std::endl;
	std::cout << "     manager destructer" << std::endl;
	client.pcl_finalize();
	client.cop_finalize();
	client.gui_finalize();
	client.save_parameters();
}