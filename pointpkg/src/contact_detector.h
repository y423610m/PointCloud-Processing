

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

	using Cloud = typename pcl::PointCloud<PointTypeT>;
	using CloudPtr = typename pcl::PointCloud<PointTypeT>::Ptr;
	using CloudConstPtr = typename Cloud::ConstPtr;

	// not transformed pointcloud .'. roe pcd data
	CloudPtr cloud_target_init_;
	CloudPtr cloud_target_transformed_;
	CloudPtr cloud_mask_init_;
	CloudPtr cloud_mask_transformed_;
	CloudPtr cloud_tip_init_;
	CloudPtr cloud_tip_transformed_;
	CloudPtr cloud_sub_;

	pcl::octree::OctreePointCloudChangeDetector<PointTypeT> octree;
	std::vector<int> newPointIdxVector;
	pcl::PointIndices::Ptr inliers;


	//pcl::ApproximateVoxelGrid<PointTypeT> grid_;
	//void _gridSampleApprox(const CloudConstPtr& cloud, Cloud& result, double leaf_size) {
	//	//pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
	//	if (!grid_init_) {
	//		grid_.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
	//		grid_init_ = true;
	//	}
	//	grid_.setInputCloud(cloud);
	//	grid_.filter(result);
	//}


public:
	bool downSampleTargetCloud_ = false;
	// arg:トラッキングしたい対象の点群，　削除用点群，　トラッキングしたい対象の点群の先端位置（1点のみ）
	ContactDetector2(std::string pcd_target, std::string pcd_mask = "", std::string pcd_tip = "", std::array<int, 3> rgb = {255,0,0}) :
		cloud_target_init_(new Cloud())
		, cloud_mask_init_(new Cloud())
		, cloud_tip_init_(new Cloud())
		, cloud_target_transformed_(new Cloud())
		, cloud_mask_transformed_(new Cloud())
		, cloud_tip_transformed_(new Cloud())
		, cloud_sub_(new Cloud())
		, octree(pcl::octree::OctreePointCloudChangeDetector<PointTypeT>(0.005))
		, inliers(new pcl::PointIndices())
		, consensus_(new Cloud())
		, different_(new Cloud())
	{

		downSampleTargetCloud_ = ROSParam::getIntParam("CD_DownSampleTargetCloud");
		showConsensus_ = ROSParam::getIntParam("CD_ShowConsensus");
		showTargetTool_ = ROSParam::getIntParam("CD_ShowTargetTool");

		//TO DO 点群密度を下げる
		pcl::ApproximateVoxelGrid<PointTypeT> grid;
		double leaf_size = 0.005;
		grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
		//grid.setInputCloud(cloud);
		//grid.filter(result);

		//３つの点群を読み込む
		if (pcl::io::loadPCDFile(pcd_target, *cloud_sub_) == -1) {
			std::cerr << "remover::remover; pcd target file not found" << std::endl;
			exit(-1);
		}
		if (downSampleTargetCloud_) {
			grid.setInputCloud(cloud_sub_);
			grid.filter(*cloud_target_init_);
		}
		else *cloud_target_init_ = *cloud_sub_;
		//target着色
		for (auto& p : *cloud_target_init_) {
			p.r = rgb[0];
			p.g = rgb[1];
			p.b = rgb[2];
		}
		*cloud_target_transformed_ = *cloud_target_init_;

		if (pcd_mask != "" && pcl::io::loadPCDFile(pcd_mask, *cloud_sub_) == -1) {
			std::cerr << "remover::remover; pcd mask file not found" << std::endl;
			exit(-1);
		}
		grid.setInputCloud(cloud_sub_);
		grid.filter(*cloud_mask_init_);
		*cloud_mask_transformed_ = *cloud_mask_init_;

		if (pcd_tip != "" && pcl::io::loadPCDFile(pcd_tip, *cloud_sub_) == -1) {
			std::cerr << "remover::remover; pcd tip file not found" << std::endl;
			exit(-1);
		}
		grid.setInputCloud(cloud_sub_);
		grid.filter(*cloud_tip_init_);
		*cloud_tip_transformed_ = *cloud_tip_init_;

		std::cerr << "ContactDetector2<PointTypeT>::ContactDetector2 succsessfully read pcd" << std::endl;
		cloud_sub_->reserve(1e4);
		//gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
	}

	//読み込んだデータを姿勢変換する  init->transformed
	void transform_init(float* pos, float* ori, bool inverse) {
		//std::cerr << "ContactDetector2<PointTypeT>::transform_init(): " << " pos-z " << pos[2] << std::endl;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << pos[0], pos[1], pos[2];
		transform.rotate(Eigen::AngleAxisf(ori[0], Eigen::Vector3f::UnitX()));
		transform.rotate(Eigen::AngleAxisf(ori[1], Eigen::Vector3f::UnitY()));
		transform.rotate(Eigen::AngleAxisf(ori[2], Eigen::Vector3f::UnitZ()));

		if (inverse) transform = transform.inverse();

		if (cloud_target_transformed_->size()) cloud_target_transformed_->clear();
		pcl::transformPointCloud(*cloud_target_init_, *cloud_target_transformed_, transform);
		if (cloud_mask_transformed_->size()) cloud_mask_transformed_->clear();
		pcl::transformPointCloud(*cloud_mask_init_, *cloud_mask_transformed_, transform);
		if (cloud_tip_transformed_->size()) cloud_tip_transformed_->clear();
		pcl::transformPointCloud(*cloud_tip_init_, *cloud_tip_transformed_, transform);
	}

	//読み込んだデータを姿勢変換する  init->transformed
	void transform_init(const Eigen::Affine3f& transform) {
		//PL("aaa")
		//PL(cloud_target_init_->size())
		if (cloud_target_transformed_->size()) cloud_target_transformed_->clear();
		pcl::transformPointCloud(*cloud_target_init_, *cloud_target_transformed_, transform);
		if (cloud_mask_transformed_->size()) cloud_mask_transformed_->clear();
		pcl::transformPointCloud(*cloud_mask_init_, *cloud_mask_transformed_, transform);
		if (cloud_tip_transformed_->size()) cloud_tip_transformed_->clear();
		pcl::transformPointCloud(*cloud_tip_init_, *cloud_tip_transformed_, transform);
	}

	//姿勢変換済みのデータをさらに姿勢変換する  transformed->transformed
	void transform_rotated(float* pos, float* ori, bool inverse) {
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

		//pcl::PointCloud<PointTypeT> cloud_sub_;
		//CloudPtr cloud_sub_(new pcl::PointCloud<PointTypeT>());
		*cloud_sub_ = *cloud_target_transformed_;
		pcl::transformPointCloud(*cloud_sub_, *cloud_target_transformed_, transform);
		*cloud_sub_ = *cloud_mask_transformed_;
		pcl::transformPointCloud(*cloud_sub_, *cloud_mask_transformed_, transform);
		*cloud_sub_ = *cloud_tip_transformed_;
		pcl::transformPointCloud(*cloud_sub_, *cloud_tip_transformed_, transform);
	}

	//cloud_xyzrgbaからpcd_maskを取り除き，pcd_tip周辺の点数を数える
	CloudPtr consensus_;
	CloudPtr different_;
	bool showConsensus_ = true;
	bool showTargetTool_ = true;
	void remove_and_detect(CloudPtr& cloud) {
		//cloud_ と resultの一致検出
		//pcl::octree::OctreePointCloudChangeDetector<PointTypeT> octree(0.005);
		octree.setInputCloud(cloud_mask_transformed_);
		octree.addPointsFromInputCloud();
		octree.switchBuffers();
		octree.setInputCloud(cloud);
		octree.addPointsFromInputCloud();
		//std::vector<int> newPointIdxVector;
		if(newPointIdxVector.size()) newPointIdxVector.clear();
		octree.getPointIndicesFromNewVoxels(newPointIdxVector);
		octree.switchBuffers();

		//pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		inliers->indices.clear();
		for (const auto& v : newPointIdxVector) inliers->indices.push_back(v);
		//for (int i = 0; i < newPointIdxVector.size(); i++) {
		//	inliers->indices.push_back(newPointIdxVector[i]);
		//}


		pcl::ExtractIndices<PointTypeT> extract;
		//extract.setKeepOrganized(true);
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*different_);
		extract.setNegative(true);//red
		extract.filter(*consensus_);


		*cloud = *different_;//!!!!!!!!!!!!!!!!!!


		//*cloud = *cloud_mask_transformed_;




		//ツール先端近傍点数を数える
		PointTypeT searchPoint = (*cloud_tip_transformed_)[0];
		searchPoint.r = 255;
		searchPoint.g = 0;
		searchPoint.b = 0;

		if (cloud->size() > 0) {
			pcl::KdTreeFLANN<PointTypeT> kdtree;
			kdtree.setInputCloud(cloud);

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquareDistance;

			//double dist = sqrt();

			float R = 0.01;
			int num = 200;
			int n = kdtree.radiusSearch(searchPoint, 0.020, pointIdxRadiusSearch, pointRadiusSquareDistance);
			//std::cerr << "number of nearest point 0.020 " << n << std::endl;
			//PS(n);
			n = kdtree.radiusSearch(searchPoint, 0.015, pointIdxRadiusSearch, pointRadiusSquareDistance);
			//std::cerr << "number of nearest point 0.015 " << n << std::endl;
			//PS(n);
			n = kdtree.radiusSearch(searchPoint, 0.010, pointIdxRadiusSearch, pointRadiusSquareDistance);
			//std::cerr << "number of nearest point 0.010 " << n << std::endl;
			//PS(n);
			n = kdtree.radiusSearch(searchPoint, 0.005, pointIdxRadiusSearch, pointRadiusSquareDistance);
			//std::cerr << "number of nearest point 0.005 " << n << std::endl;
			//PL(n);

			//if (n > num) {
			//	for (int i = 0; i < int(3 * std::log2(n - num)); i++) std::cerr << "#";
			//	std::cerr << std::endl;
			//}
			/*
			メモ
			r=0.015で400超えたら接触
			*/


		}

		cloud->push_back(searchPoint);
		//発見したターゲットを再び戻す？
		if (showConsensus_) for (const auto& point : *consensus_) {
			cloud->emplace_back(point);
			//cloud->back().r = 255;
			//cloud->back().g = 0;
			//cloud->back().b = 0;
		}

		//target予測位置表示
		if(showTargetTool_) for (const auto& p : *cloud_target_transformed_) {
			cloud->emplace_back(p);
		}
	}

	//粒子フィルタにおける重心計算を補正するため
	Eigen::Affine3f init_for_particlefilter() {
		//重心計算
		Eigen::Vector4f c;
		pcl::compute3DCentroid<PointTypeT>(*cloud_target_init_, c);
		Eigen::Affine3f trans = Eigen::Affine3f::Identity();
		trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);

		//target, mask,tipに対して，重心が原点に来るように平行移動
		pcl::transformPointCloud<PointTypeT>(*cloud_target_init_, *cloud_target_transformed_, trans.inverse());
		*cloud_target_init_ = *cloud_target_transformed_;

		pcl::transformPointCloud<PointTypeT>(*cloud_mask_init_, *cloud_mask_transformed_, trans.inverse());
		*cloud_mask_init_ = *cloud_mask_transformed_;

		pcl::transformPointCloud<PointTypeT>(*cloud_tip_init_, *cloud_tip_transformed_, trans.inverse());
		*cloud_tip_init_ = *cloud_tip_transformed_;

		//gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

		return  trans;
	}

	CloudPtr get_cloud_target_init() { return cloud_target_init_; }
	CloudPtr get_cloud_target_transformed() { return cloud_target_transformed_; }
	//CloudPtr get_cloud_mask() { return cloud_mask_transformed_; }
	//CloudPtr get_cloud_tip() { return cloud_tip_transformed_; }
};

