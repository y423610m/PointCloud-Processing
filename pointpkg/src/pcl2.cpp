#include "pcl2.h"


/*
TO DO
convertArrayToPCD cloud_yolo‘Î‰ž
*/


//cvt vector to PCD
template<>
void PCL2<pcl::PointXYZRGB>::_convert_array_to_pcd(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	cloud_main_->clear();
	if (points.size() == 0) return;

	cloud_main_->resize(points.size() / 3);
	for (int i = 0; i < points.size() / 3; i++) {
		(*cloud_main_)[i].x = points[3 * i + 0];
		(*cloud_main_)[i].y = points[3 * i + 1];
		(*cloud_main_)[i].z = points[3 * i + 2];
		(*cloud_main_)[i].r = color[3 * i + 0];
		(*cloud_main_)[i].g = color[3 * i + 1];
		(*cloud_main_)[i].b = color[3 * i + 2];
	}
}



template<>
void PCL2<pcl::PointXYZ>::_convert_array_to_pcd(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	cloud_main_->clear();
	if (points.size() == 0) return;

	cloud_main_->resize(points.size() / 3);
	for (int i = 0; i < points.size() / 3; i++) {
		(*cloud_main_)[i].x = points[3 * i + 0];
		(*cloud_main_)[i].y = points[3 * i + 1];
		(*cloud_main_)[i].z = points[3 * i + 2];
	}
}



//cvt PCD to vector
template<>
void PCL2<pcl::PointXYZRGB>::_convert_pcd_to_array(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	points.clear();
	color.clear();
	if (cloud_main_->size() == 0) return;

	points.resize(cloud_main_->size() * 3);
	color.resize(cloud_main_->size() * 3);

	for (int i = 0; i < cloud_main_->size(); i++) {
		points[3 * i + 0] = (*cloud_main_)[i].x;
		points[3 * i + 1] = (*cloud_main_)[i].y;
		points[3 * i + 2] = (*cloud_main_)[i].z;
		color[3 * i + 0] = (*cloud_main_)[i].r;
		color[3 * i + 1] = (*cloud_main_)[i].g;
		color[3 * i + 2] = (*cloud_main_)[i].b;
	}
	number_of_points = points.size() / 3;
}



template<>
void PCL2<pcl::PointXYZ>::_convert_pcd_to_array(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	points.clear();
	color.clear();
	if (cloud_main_->size() == 0) return;

	points.resize(cloud_main_->size() * 3);
	color.resize(cloud_main_->size() * 3);

	for (int i = 0; i < cloud_main_->size(); i++) {
		points[3 * i + 0] = (*cloud_main_)[i].x;
		points[3 * i + 1] = (*cloud_main_)[i].y;
		points[3 * i + 2] = (*cloud_main_)[i].z;
		color[3 * i + 0] = 255;
		color[3 * i + 1] = 255;
		color[3 * i + 2] = 255;
	}
	number_of_points = points.size() / 3;
}