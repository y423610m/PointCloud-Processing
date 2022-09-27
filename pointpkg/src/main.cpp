/*

track param robottype

coppeliasim getHandle



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





int main(int argc, char** argv) {
	
	ros::init(argc, argv, "pointer");
	ROSParam::init();

	ros::Rate rate(ROSParam::getIntParam("MAIN_rate"));

	std::unique_ptr<Manager> manager(new Manager());
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
/* プログラム改善点
--PCLクラスのテンプレート化--
現状はint pcd_type_で処理変更
理想はpcl::PointXYZ, pcl::PointXYZRGBA各々にtemplate<> PCL<pcl::PointXYZRGBA>::hoge()に変更

--PCL::_detect_contact_with_particlefilter()用のメンバ変数多すぎ--
ローカルor別クラスにまとめる

--manager内でnumber_of_points変数必要?--
std::vector<int> points.size(), color.size()/3などで良さげ




*/
