

#include <iostream>
#include <windows.h>
#include <numeric>
#include <cmath>
#include <string>
#include <fstream>
#include <algorithm>

#include "ros_param.h"
#include "development_commands.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

template<typename PointTypeT = pcl::PointXYZRGBA>
class ContactDetector2 {
private:

	using CloudPtr = typename pcl::PointCloud<PointTypeT>::Ptr;

	// not transformed pointcloud .'. roe pcd data
	CloudPtr cloud_target_init_;
	CloudPtr cloud_target_transformed_;
	CloudPtr cloud_mask_init_;
	CloudPtr cloud_mask_transformed_;
	CloudPtr cloud_tip_init_;
	CloudPtr cloud_tip_transformed_;

	pcl::octree::OctreePointCloudChangeDetector<PointTypeT> octree;
	CloudPtr cloud_tmp;
	std::vector<int> newPointIdxVector;
	pcl::PointIndices::Ptr inliers;
public:
	ContactDetector2(std::string pcd_target, std::string pcd_mask, std::string pcd_tip);

	void transform_init(float* pos, float* ori, bool inverse = false);
	void transform_init(const Eigen::Affine3f& T);
	void transform_rotated(float* pos, float* ori, bool inverse = false);
	void remove_and_detect(CloudPtr& cloud_xyzrgba);

	Eigen::Affine3f init_for_particlefilter();

	CloudPtr get_cloud_target() { return cloud_target_transformed_; }
	CloudPtr get_cloud_mask() { return cloud_mask_transformed_; }
	CloudPtr get_cloud_tip() { return cloud_tip_transformed_; }
};

#ifndef CD2DEF

// arg:トラッキングしたい対象の点群，　削除用点群，　トラッキングしたい対象の点群の先端位置（1点のみ）
template<typename PointTypeT>
ContactDetector2<PointTypeT>::ContactDetector2(std::string pcd_target, std::string pcd_mask = "", std::string pcd_tip = "") :
	octree(pcl::octree::OctreePointCloudChangeDetector<PointTypeT>(0.005))
	, cloud_tmp(new pcl::PointCloud<PointTypeT>())
	, inliers(new pcl::PointIndices())
{
	//TO DO 点群密度を下げる
	//CloudPtr cloud_tmp(new pcl::PointCloud<PointTypeT>);
	//pcl::ApproximateVoxelGrid<PointTypeT> grid;
	//grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
	//grid.setInputCloud(cloud);
	//grid.filter(result);

	//３つの点群を読み込む
	cloud_target_init_.reset(new pcl::PointCloud<PointTypeT>);
	if (pcl::io::loadPCDFile(pcd_target, *cloud_target_init_) == -1) {
		std::cerr << "remover::remover; pcd target file not found" << std::endl;
		exit(-1);
	}
	cloud_mask_init_.reset(new pcl::PointCloud<PointTypeT>);
	if (pcd_mask != "" && pcl::io::loadPCDFile(pcd_mask, *cloud_mask_init_) == -1) {
		std::cerr << "remover::remover; pcd mask file not found" << std::endl;
		exit(-1);
	}
	cloud_tip_init_.reset(new pcl::PointCloud<PointTypeT>);
	if (pcd_tip != "" && pcl::io::loadPCDFile(pcd_tip, *cloud_tip_init_) == -1) {
		std::cerr << "remover::remover; pcd tip file not found" << std::endl;
		exit(-1);
	}
	std::cerr << "ContactDetector2<PointTypeT>::ContactDetector2 succsessfully read pcd" << std::endl;
	cloud_target_transformed_ = cloud_target_init_;
	cloud_mask_transformed_ = cloud_mask_init_;
	cloud_tip_transformed_ = cloud_tip_init_;
	//gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
}

//読み込んだデータを姿勢変換する  init->transformed
template<typename PointTypeT>
void ContactDetector2<PointTypeT>::transform_init(float* pos, float* ori, bool inverse) {
	//std::cerr << "ContactDetector2<PointTypeT>::transform_init(): " << " pos-z " << pos[2] << std::endl;
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << pos[0], pos[1], pos[2];
	transform.rotate(Eigen::AngleAxisf(ori[0], Eigen::Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(ori[1], Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(ori[2], Eigen::Vector3f::UnitZ()));

	if (inverse == true) transform = transform.inverse();

	cloud_target_transformed_.reset(new pcl::PointCloud<PointTypeT>());
	pcl::transformPointCloud(*cloud_target_init_, *cloud_target_transformed_, transform);
	cloud_mask_transformed_.reset(new pcl::PointCloud<PointTypeT>());
	pcl::transformPointCloud(*cloud_mask_init_, *cloud_mask_transformed_, transform);
	cloud_tip_transformed_.reset(new pcl::PointCloud<PointTypeT>());
	pcl::transformPointCloud(*cloud_tip_init_, *cloud_tip_transformed_, transform);
}

//読み込んだデータを姿勢変換する  init->transformed
template<typename PointTypeT>
void ContactDetector2<PointTypeT>::transform_init(const Eigen::Affine3f& transform) {
	//PL("aaa")
	//PL(cloud_target_init_->size())
	cloud_target_transformed_.reset(new pcl::PointCloud<PointTypeT>());
	pcl::transformPointCloud(*cloud_target_init_, *cloud_target_transformed_, transform);
	cloud_mask_transformed_.reset(new pcl::PointCloud<PointTypeT>());
	pcl::transformPointCloud(*cloud_mask_init_, *cloud_mask_transformed_, transform);
	cloud_tip_transformed_.reset(new pcl::PointCloud<PointTypeT>());
	pcl::transformPointCloud(*cloud_tip_init_, *cloud_tip_transformed_, transform);
}

//姿勢変換済みのデータをさらに姿勢変換する  transformed->transformed
template<typename PointTypeT>
void ContactDetector2<PointTypeT>::transform_rotated(float* pos, float* ori, bool inverse) {
	//std::cerr << "rotated" << std::endl;
	//for (int i = 0; i < 6; i++) std::cerr << pos[i] << " ";
	//std::cerr << endl;
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << pos[0], pos[1], pos[2];
	//std::cerr << "transform rotated target" << transform.matrix() << std::endl;
	transform.rotate(Eigen::AngleAxisf(ori[0], Eigen::Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(ori[1], Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(ori[2], Eigen::Vector3f::UnitZ()));
	//std::cerr << "transform rotated target" << transform.matrix() << std::endl;
	if (inverse == true) transform = transform.inverse();

	//pcl::PointCloud<PointTypeT> cloud_tmp;
	//CloudPtr cloud_tmp(new pcl::PointCloud<PointTypeT>());
	cloud_tmp = cloud_target_transformed_;
	pcl::transformPointCloud(*cloud_tmp, *cloud_target_transformed_, transform);
	cloud_tmp = cloud_mask_transformed_;
	pcl::transformPointCloud(*cloud_tmp, *cloud_mask_transformed_, transform);
	cloud_tmp = cloud_tip_transformed_;
	pcl::transformPointCloud(*cloud_tmp, *cloud_tip_transformed_, transform);
}

//cloud_xyzrgbaからpcd_maskを取り除き，pcd_tip周辺の点数を数える
template<typename PointTypeT>
void ContactDetector2<PointTypeT>::remove_and_detect(CloudPtr& cloud_xyzrgba) {
	//return;

	//cloud_ と resultの一致検出
	//pcl::octree::OctreePointCloudChangeDetector<PointTypeT> octree(0.005);
	octree.setInputCloud(cloud_mask_transformed_);
	octree.addPointsFromInputCloud();
	octree.switchBuffers();
	octree.setInputCloud(cloud_xyzrgba);
	octree.addPointsFromInputCloud();


	//std::vector<int> newPointIdxVector;
	newPointIdxVector.clear();
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	inliers->indices.clear();
	for (int i = 0; i < newPointIdxVector.size(); i++) {
		inliers->indices.push_back(newPointIdxVector[i]);
	}
	CloudPtr consensus(new pcl::PointCloud<PointTypeT>);
	CloudPtr different(new pcl::PointCloud<PointTypeT>);

	pcl::ExtractIndices<PointTypeT> extract;
	//extract.setKeepOrganized(true);
	extract.setInputCloud(cloud_xyzrgba);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*different);
	extract.setNegative(true);//red
	extract.filter(*consensus);

	//std::cerr << cloud_xyzrgba->size() << " " << different->size() << " " << consensus->size() <<" "<< newPointIdxVector.size()<< std::endl;

	cloud_xyzrgba = different;
	//*cloud_xyzrgba = *cloud_mask_transformed_;
	//std::cerr << cloud_xyzrgba->size() << std::endl;


	//cloud_xyzrgba = cloud_mask_transformed_;

	pcl::PointXYZRGBA searchPoint = (*cloud_tip_transformed_)[0];
	searchPoint.r = 255;
	searchPoint.g = 0;
	searchPoint.b = 0;

	if (cloud_xyzrgba->size() > 0) {
		pcl::KdTreeFLANN<PointTypeT> kdtree;
		kdtree.setInputCloud(cloud_xyzrgba);



		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquareDistance;

		float R = 0.01;
		int num = 200;
		int n = kdtree.radiusSearch(searchPoint, 0.020, pointIdxRadiusSearch, pointRadiusSquareDistance);
		//std::cerr << "number of nearest point 0.020 " << n << std::endl;
		n = kdtree.radiusSearch(searchPoint, 0.015, pointIdxRadiusSearch, pointRadiusSquareDistance);
		//std::cerr << "number of nearest point 0.015 " << n << std::endl;
		n = kdtree.radiusSearch(searchPoint, 0.010, pointIdxRadiusSearch, pointRadiusSquareDistance);
		//std::cerr << "number of nearest point 0.010 " << n << std::endl;
		n = kdtree.radiusSearch(searchPoint, 0.005, pointIdxRadiusSearch, pointRadiusSquareDistance);
		//std::cerr << "number of nearest point 0.005 " << n << std::endl;



		//if (n > num) for (int i = 0; i < int(std::log2(n-num)); i++) std::cerr << "##";
		//std::cerr << std::endl;
	}

	cloud_xyzrgba->push_back(searchPoint);


	for (auto& point : *consensus) {
		//point.r = 0;// -cloud.r;
		//point.g = 0;// -cloud.g;
		//point.b = 255;// -cloud.b;
		cloud_xyzrgba->push_back(point);
	}


	for (auto p : *cloud_target_transformed_) {
		p.r = 255;
		cloud_xyzrgba->push_back(p);
	}

}

//粒子フィルタにおける重心計算を補正するため
template<typename PointTypeT>
Eigen::Affine3f ContactDetector2<PointTypeT>::init_for_particlefilter() {
	//重心計算
	Eigen::Vector4f c;
	pcl::compute3DCentroid<PointTypeT>(*cloud_target_init_, c);
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);

	//target, mask,tipに対して，重心が原点に来るように平行移動
	cloud_target_transformed_.reset(new pcl::PointCloud<PointTypeT>());
	pcl::transformPointCloud<PointTypeT>(*cloud_target_init_, *cloud_target_transformed_, trans.inverse());
	//gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
	*cloud_target_init_ = *cloud_target_transformed_;

	cloud_mask_transformed_.reset(new pcl::PointCloud<PointTypeT>());
	pcl::transformPointCloud<PointTypeT>(*cloud_mask_init_, *cloud_mask_transformed_, trans.inverse());
	*cloud_mask_init_ = *cloud_mask_transformed_;

	cloud_tip_transformed_.reset(new pcl::PointCloud<PointTypeT>());
	pcl::transformPointCloud<PointTypeT>(*cloud_tip_init_, *cloud_tip_transformed_, trans.inverse());
	*cloud_tip_init_ = *cloud_tip_transformed_;

	return  trans;
}

#define CD2DEF 1
#endif //CD2DEF