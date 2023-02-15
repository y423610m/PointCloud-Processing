//PCL
#pragma once



#define PCL_ID_PCL 0
#define PCL_ID_ReadPCD 1
#define PCL_ID_WritePCD 2
#define PCL_ID_Transform 3
#define PCL_ID_PassThrough 4
#define PCL_ID_ParticleFilter 5
#define PCL_ID_DetectWithSim 6
#define PCL_ID_Compress 7
#define PCL_ID_DetectChange 8
#define PCL_ID_RemovePlane 9
#define PCL_ID_Recognite 10
#define PCL_ID_DrawResult 11
#define PCL_ID_LoadParam 12
#define PCL_ID_SaveParam 13
#define PCL_ID_SetROSParam 14
#define PCL_ID_CvtVecToPCD 15
#define PCL_ID_CvtPCDToVec 16
#define PCL_ID_DetectWithYOLO 17
#define PCL_ID_CalibrationHorizon 18
#define PCL_ID_CalibrationVertical 19

// <31


#include <vector>
#include <string>
#include <time.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

//compression
#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/compression/octree_pointcloud_compression.h>
//#include <pcl/visualization/cloud_viewer.h>


//detect changes
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

//segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <string>

//tracking
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
//#include <pcl/tracking/impl/nearest_pair_point_cloud_coherence.hpp>

#include <boost/format.hpp>

#include <mutex>
#include <thread>

#define PCL_NO_PRECOMPILE
#define PCL_TRACKING_NORMAL_SUPPORTED
//#include <pcl/tracking/particle_filter.h>

//recognite
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>





#include <iostream>
#include <windows.h>
#include <numeric>
#include <cmath>
#include <string>
#include <fstream>
#include <algorithm>

#include "ros_param.h"
#include "development_commands.h"
#include "coppeliasim_interface.h"
#include "contact_detector.h"
#include "array3d_extention.h"

using namespace pcl::tracking;
using namespace std::chrono_literals;


template<typename PointTypeT = pcl::PointXYZRGBA>
class PCL2 {
private:
	using Cloud = pcl::PointCloud<PointTypeT>;
	using CloudPtr = typename Cloud::Ptr;
	using CloudConstPtr = typename Cloud::ConstPtr;
	//以下の書き方だと，typenameの曖昧性でerror https://qiita.com/Seo-4d696b75/items/818f6abb273f9cdffaa4
	//typedef Cloud::Ptr CloudPtr;
	//typedef Cloud::ConstPtr CloudConstPtr;


	//点群格納用データ（vector）
	std::vector<float> points_;
	std::vector<int> color_;
	//点群データ（pcd）
	CloudPtr cloud_main_;
	CloudPtr cloud_sub_;
	CoppeliaSimInterface* coppeliasim_interface_;
	bool enable_[32];
	bool initialized_[32];
	std::unique_ptr<ContactDetector2<PointTypeT>> contact_detector_;



	//folder内のpcdをcloud_mianに追加 1
	//read(), write()用
	int cnt_read_ = 0;
	std::string folder_read_;
	void _read_pcd_file(std::string folder, int& number_of_points, std::vector<int>& color) {
		if (!enable_[PCL_ID_ReadPCD]) return;
		if (folder == "") return;
		std::string file;
		if (folder.find(".pcd") != std::string::npos) file = folder;
		else {
			file = folder + std::to_string(cnt_read_);
			//file += std::to_string(cnt_read_);
			file += ".pcd";
			//std::cerr << "qwert" << std::endl;
		}


		if(cloud_sub_->size()) cloud_sub_->clear();
		if (pcl::io::loadPCDFile<PointTypeT>(file, *cloud_sub_) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file test_pcd.pcd \n");
			EL(file);
			cnt_read_ = 0;
		}
		else {

			for (const auto& p : *cloud_sub_) { cloud_main_->push_back(p); }
			//cloud_main_ = cloud_tmp;
			number_of_points = cloud_main_->width * cloud_main_->height;
			cnt_read_++;
			//std::cerr << "read" << endl;
		}
	}



	//cloud_mianをpcdとしてfolder内に保存 2
	int cnt_write_ = 0;
	std::string folder_write_;
	void _write_pcd_file(std::string folder) {
		if (!enable_[PCL_ID_WritePCD]) return;
		if (cloud_main_->size() == 0) return;

		if (folder.back() != '/') folder += '/';


		pcl::io::savePCDFileASCII(folder + std::to_string(cnt_write_) + ".pcd", *cloud_main_);
		cnt_write_++;

		//std::cerr << "Saved " << cloud_main_->size() << " data points to test_pcd.pcd." << std::endl;
	}



	//cloud_xyzrgbaを姿勢変換 3
	//注意 cloud_subは関数内で一時的に使うだけで，座標変換後の点群データはcloud_mainに格納されている．
	double transformation_parameters_[6];//x,y,z,anglex,y,z
	void _transform(CloudPtr& cloud_main, CloudPtr& cloud_sub) {
		if (!enable_[PCL_ID_Transform]) return;
		if (cloud_main->size() == 0) return;

		Eigen::Affine3f transform;

		transform = Eigen::Affine3f::Identity();
		transform.translation() << transformation_parameters_[0], transformation_parameters_[1], transformation_parameters_[2];
		//if (cloud_sub->size()) cloud_sub->clear();
		//pcl::transformPointCloud(*cloud_main, *cloud_sub, transform);
		//cloud_main.swap(cloud_sub);


		//transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(transformation_parameters_[5], Eigen::Vector3f::UnitZ()));
		transform.rotate(Eigen::AngleAxisf(transformation_parameters_[4], Eigen::Vector3f::UnitY()));
		transform.rotate(Eigen::AngleAxisf(transformation_parameters_[3], Eigen::Vector3f::UnitX()));
		if (cloud_sub->size()) cloud_sub->clear();
		pcl::transformPointCloud(*cloud_main, *cloud_sub, transform);
		cloud_main.swap(cloud_sub);



	}



	//Filter along a specified dimension 4
	//_filter_path_through()
	//pcl::PassThrough<pcl::PointXYZRGBA> pass;
	double threshold_[6] = { -0.3, 0.0, -0.5, 0.0, 0.0, 0.3 };	
	void _filterPassThrough() {
		if (!enable_[PCL_ID_PassThrough]) return;
		if (cloud_main_->size() == 0) return;
		//PointCloud::Ptr cloud_tmp(new PointCloud());
		//pcl::PassThrough<pcl::PointXYZRGBA> passX;
		////passX.setKeepOrganized(false);
		//passX.setInputCloud(cloud_main_);
		//passX.setFilterFieldName("x");
		//passX.setFilterLimits(threshold_[0], threshold_[1]);
		//passX.filter(*cloud_tmp);

		if (cloud_sub_->size()) cloud_sub_->clear();
		for (auto p : *cloud_main_) if (threshold_[0] <= p.x && p.x <= threshold_[1] && threshold_[2] <= p.y && p.y <= threshold_[3] && threshold_[4] <= p.z && p.z <= threshold_[5])
			cloud_sub_->push_back(p);

		cloud_main_.swap(cloud_sub_);

	}



	//track()用
	bool grid_init_ = false;
	pcl::ApproximateVoxelGrid<PointTypeT> grid_;
	void _gridSampleApprox(const CloudConstPtr& cloud, Cloud& result, double leaf_size) {
		//pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
		if (!grid_init_) {
			grid_.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
			grid_init_ = true;
		}
		grid_.setInputCloud(cloud);
		grid_.filter(result);
	}
	bool particle_filter_initialized_ = false;
	typedef ParticleXYZRPY ParticleT;
	//typedef ParticleFilterTracker<PointTypeT, ParticleT> ParticleFilter;
	using ParticleFilter = typename ParticleFilterTracker<PointTypeT, ParticleT>;
	std::mutex mtx_;
	typename ParticleFilter::Ptr tracker_;
	CloudPtr cloud_downsampled_;
	CloudPtr target_cloud;
	typename KLDAdaptiveParticleFilterOMPTracker<PointTypeT, ParticleT>::Ptr tracker;
	typename ApproxNearestPairPointCloudCoherence<PointTypeT>::Ptr coherence;
	typename DistanceCoherence<PointTypeT>::Ptr distance_coherence;
	typename pcl::search::Octree<PointTypeT>::Ptr search;
	typename HSVColorCoherence<PointTypeT>::Ptr hsv_coherence;
	typename NearestPairPointCloudCoherence<PointTypeT>::Ptr nearest_pair_coherence;
	double downsampling_grid_size_ = 0.002;
	//除去
	CloudPtr cloud_target_remover_;
	CloudPtr cloud_target_remover_ref_;
	CloudPtr cloud_target_remover_ref_downsampled_;
	//R探索
	CloudPtr Rcenter_;
	CloudPtr Rcenter_ref_;
	void _detect_contact_with_particlefilter() {
		//https://pcl.readthedocs.io/projects/tutorials/en/latest/tracking.html
		if (!enable_[PCL_ID_ParticleFilter]) return;

		if (cloud_main_->size() == 0) return;

		if (!initialized_[PCL_ID_ParticleFilter]) {

			tracker.reset(new KLDAdaptiveParticleFilterOMPTracker<PointTypeT, ParticleT>(8));
			coherence.reset(new ApproxNearestPairPointCloudCoherence<PointTypeT>());
			distance_coherence.reset(new DistanceCoherence<PointTypeT>());
			hsv_coherence.reset(new HSVColorCoherence<PointTypeT>());
			nearest_pair_coherence.reset(new NearestPairPointCloudCoherence<PointTypeT>());
			search.reset(new pcl::search::Octree<PointTypeT>(0.01));
			cloud_downsampled_.reset(new Cloud());
			//この下３つ使ってない？
			cloud_target_remover_ref_.reset(new Cloud);
			cloud_target_remover_ref_downsampled_.reset(new Cloud);
			Rcenter_ref_.reset(new Cloud);

			ParticleT bin_size;
			bin_size.x = 0.01f;
			bin_size.y = 0.01f;
			bin_size.z = 0.01f;
			bin_size.roll = 0.01f;
			bin_size.pitch = 0.01f;
			bin_size.yaw = 0.01f;


			std::vector<double> default_step_covariance;
			std::vector<double> initial_noise_covariance;
			std::vector<double> default_initial_mean;
			int MaximumParticleNum;
			double Delta;
			double Epsilon;
			int IterationNum;
			int ParticleNum;
			double ResampleLikelihoodThr;

			if (ROSParam::getIntParam("COMMON_robot_type") == 1) {
				PL("particle filter for Cobotta Tool")
				default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
				for (int i = 0; i < 3; i++) default_step_covariance[i] *= 0.1;
				for (int i = 3; i < 6; i++) default_step_covariance[i] *= 40.0;
				initial_noise_covariance = std::vector<double>(6, 0.0001); // 1 * 6 vector
				//std::vector<double> default_initial_mean = std::vector<double>{ -0.15 * 1., -0.35 * 1., 0.22 * 1., 0, 0, 0. };// 1 * 6 vector xyzrpy
				//std::vector<double> default_initial_mean = std::vector<double>{ -0.16 * 1., -0.26 * 1., 0.22 * 0., 0., 0., 0. };// 1 * 6 vector xyzrpy
				default_initial_mean = std::vector<double>{ -0.15 * 1., -0.30 * 1., 0.27 * 1., 0., 0, 1.5 };// 1 * 6 vector xyzrpy best for MSRobot
				default_initial_mean = std::vector<double>(6, 0.);
				default_initial_mean[1] = -0.1;
				MaximumParticleNum = 500;
				Delta = 0.99;
				Epsilon = 0.15;
				IterationNum = 2;
				ParticleNum = 50;//300;
				ResampleLikelihoodThr = 0.0;



				//Setup coherence object for tracking
				////////////////////pcl::tracking::NearestPairPointCloudCoherence
				hsv_coherence->setHWeight(1.0);
				hsv_coherence->setSWeight(0.0);
				hsv_coherence->setVWeight(0.0);
				coherence->addPointCoherence(hsv_coherence);

				//distance_coherence->setWeight(0.005);
				//coherence->addPointCoherence(nearest_pair_coherence);
				//coherence->addPointCoherence(distance_coherence);

				coherence->setSearchMethod(search);
				coherence->setMaximumDistance(0.01);
			}
			else {//MSRobot
				default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
				for (int i = 0; i < 3; i++) default_step_covariance[i] *= 0.15;//*0.1
				for (int i = 3; i < 6; i++) default_step_covariance[i] *= 40.0;//40
				initial_noise_covariance = std::vector<double>(6, 0.00001); // 1 * 6 vector
				//std::vector<double> default_initial_mean = std::vector<double>{ -0.15 * 1., -0.35 * 1., 0.22 * 1., 0, 0, 0. };// 1 * 6 vector xyzrpy
				//std::vector<double> default_initial_mean = std::vector<double>{ -0.16 * 1., -0.26 * 1., 0.22 * 0., 0., 0., 0. };// 1 * 6 vector xyzrpy
				default_initial_mean = std::vector<double>{ -0.15 * 1., -0.30 * 1., 0.27 * 1., 0, 0, 0. };// 1 * 6 vector xyzrpy best for MSRobot
				default_initial_mean = std::vector<double>(6, 0.);
				MaximumParticleNum = 500;//1000
				Delta = 0.99;//0.99
				Epsilon = 0.15;//0.15
				IterationNum = 3;//3
				ParticleNum = 50;//300;
				ResampleLikelihoodThr = 0.0;



				//Setup coherence object for tracking
				////////////////////pcl::tracking::NearestPairPointCloudCoherence
				//H:色　S: V:
				hsv_coherence->setHWeight(1.0);
				hsv_coherence->setSWeight(0.1);
				hsv_coherence->setVWeight(0.1);
				//distance_coherence->setWeight(0.005);


				coherence->addPointCoherence(hsv_coherence);
				//coherence->addPointCoherence(nearest_pair_coherence);
				//coherence->addPointCoherence(distance_coherence);
				coherence->setSearchMethod(search);
				coherence->setMaximumDistance(0.02);//0.01
			}

			//Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
			tracker->setMaximumParticleNum(MaximumParticleNum);
			tracker->setDelta(Delta);
			tracker->setEpsilon(Epsilon);
			tracker->setBinSize(bin_size);

			tracker_ = tracker;
			tracker_->setTrans(Eigen::Affine3f::Identity());
			tracker_->setStepNoiseCovariance(default_step_covariance);
			tracker_->setInitialNoiseCovariance(initial_noise_covariance);
			tracker_->setInitialNoiseMean(default_initial_mean);
			tracker_->setIterationNum(IterationNum);
			tracker_->setParticleNum(ParticleNum);
			tracker_->setResampleLikelihoodThr(ResampleLikelihoodThr);
			tracker_->setUseNormal(false);
			tracker_->setCloudCoherence(coherence);

			Eigen::Affine3f trans = contact_detector_->init_for_particlefilter();
			tracker_->setTrans(trans);
			tracker_->setReferenceCloud(contact_detector_->get_cloud_target_init());


			initialized_[PCL_ID_ParticleFilter] = true;
			//１ループ目は初期化のみで関数抜ける
			PL("Particle FIlter initialized")
			return;
		}


		//std::lock_guard<std::mutex> lock(mtx_);
		if (cloud_sub_->size()) cloud_sub_->clear();
		_gridSampleApprox(cloud_main_, *cloud_sub_, downsampling_grid_size_);
		//*cloud_main_ = *cloud_downsampled_;


		//Track the object
		tracker_->setInputCloud(cloud_sub_);
		tracker_->compute();


		ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
		//std::cerr << tracker_->getParticles() << std::endl;
		if (tracker_->getParticles() || false)
		{


			//標準偏差から消失予測

			//double sumx = 0., sumy = 0., sumz = 0.;
			//double vx = 0., vy = 0., vz = 0.;
			//for (auto p : *particles) {
			//	sumx += p.x; sumy += p.y; sumz += p.z;
			//	vx += p.x * p.x; vy += p.y * p.y; vz += p.z * p.z;
			//}
			//double size = particles->size();
			//vx = vx / size; sumx = sumx / size; sumx = sumx * sumx;
			//vy = vy / size; sumy = sumy / size; sumy = sumy * sumy;
			//vz = vz / size; sumz = sumz / size; sumz = sumz * sumz;
			//std::cerr << "x  二乗和  " << vx << "  平均2乗  " << sumx <<"  分散  "<< vx- sumx << std::endl;//0.001
			//std::cerr << "y  二乗和  " << vy << "  平均2乗  " << sumy << "  分散  " << vy - sumy << std::endl;
			//std::cerr << "z  二乗和  " << vz << "  平均2乗  " << sumz << "  分散  " << vz - sumz << std::endl;
			//std::cerr << ""<<std::endl;


			ParticleXYZRPY result = tracker_->getResult();
			Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);

			contact_detector_->transform_init(transformation);

			contact_detector_->remove_and_detect(cloud_main_);

			//EL(particles->size())
			for (const auto& particle : *particles)
			{
				continue;
				cloud_main_->push_back(PointTypeT());
				PointTypeT& point = cloud_main_->back();
				point.x = particle.x;
				point.y = particle.y;
				point.z = particle.z;
				point.r = 255;
				point.g = 255;
				point.r = (std::min)(255, (int)(100000 * particle.weight));
				//point.g = (std::min)(255, (int)(100000 * particle.weight));
			}

		}

		//*cloud_main_ = *(tracker_->getReferenceCloud());
	}



	//CoppeliaSimのロボットモデルの姿勢を取得し，接触判定する 6
	void _detect_contact_with_coppeliasim() {
		if (!enable_[PCL_ID_DetectWithSim]) return;
		if (cloud_main_->size() == 0) return;

		////pcd_target, pcd_mask, pcd_tipを絶対座標の原点に戻す
		std::vector<float> RemTP = coppeliasim_interface_->getRemoverTipPose();
		contact_detector_->transform_init(&RemTP[0], &RemTP[3], true);
		//coppeliasim内のロボットモデルと位置姿勢を一致させる．
		std::vector<float> LeftTP = coppeliasim_interface_->getLeftTipPose();
		contact_detector_->transform_rotated(&LeftTP[0], &LeftTP[3], false);

		contact_detector_->remove_and_detect(cloud_main_);
	}



	//octreeによるデータ圧縮 7
	std::stringstream compressedData_;
	pcl::io::OctreePointCloudCompression<PointTypeT> octreeCompression_;
	void _compress() {

		if (!enable[PCL_ID_Compress]) return;
		if (!initialized[PCL_ID_Compress]) {
			octreeCompression_ = pcl::io::OctreePointCloudCompression<PointTypeT>(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR, false);
			initialized_[PCL_ID_Compress] = true;
		}

		double a = clock();
		if (cloud_main_->size() > 0) {
			octreeCompression_.encodePointCloud(cloud_main_, compressedData_);
			octreeCompression_.decodePointCloud(compressedData_, cloud_main_);
			//frame_++;
		}
		std::cerr << "time for compression   " << double(clock() - a) / 1000 << std::endl;
	}



	//点群の変化差分抽出 8
	std::vector<float> appeared_points_;
	std::vector<float> disappeared_points_;
	CloudPtr cloud_prev_;
	float changeRes_ = 0.02;
	typename pcl::octree::OctreePointCloudChangeDetector<PointTypeT>::Ptr octreeChangeDetector_;
	std::vector<int> newPointIdxVector_;
	void _detect_change(std::vector<int>& color) {
		if (cloud_main_->size() == 0) return;
		if (!initialized[PCL_ID_DetectChange]) {
			octreeChangeDetector_ = new pcl::octree::OctreePointCloudChangeDetector<PointTypeT>(changeRes_);
			initialized_[PCL_ID_DetectChange] = true;
		}
		//pcl::octree::OctreePointCloudChangeDetector<PointTypeT> octreeChangeDetector_(changeRes);
		////////////////////    appeared
		octreeChangeDetector_->setInputCloud(cloud_prev_);
		octreeChangeDetector_->addPointsFromInputCloud();
		octreeChangeDetector_->switchBuffers();
		octreeChangeDetector_->setInputCloud(cloud_main_);
		octreeChangeDetector_->addPointsFromInputCloud();
		newPointIdxVector_.clear();
		octreeChangeDetector_->getPointIndicesFromNewVoxels(newPointIdxVector_);
		appeared_points_.clear();
		for (std::size_t i = 0; i < newPointIdxVector_.size(); ++i) {
			color[3 * newPointIdxVector_[i] + 0] = 0;
			color[3 * newPointIdxVector_[i] + 1] = 255 - color[3 * newPointIdxVector_[i] + 1];
			color[3 * newPointIdxVector_[i] + 2] = 255 - color[3 * newPointIdxVector_[i] + 2];
			appeared_points_.push_back((*cloud_main_)[newPointIdxVector_[i]].x);
			appeared_points_.push_back((*cloud_main_)[newPointIdxVector_[i]].y);
			appeared_points_.push_back((*cloud_main_)[newPointIdxVector_[i]].z);
		}
		////////////////////  disappeared
		octreeChangeDetector_->setInputCloud(cloud_main_);
		octreeChangeDetector_->addPointsFromInputCloud();
		octreeChangeDetector_->switchBuffers();
		// Add points from cloudB to octreeChangeDetector_
		octreeChangeDetector_->setInputCloud(cloud_prev_);
		octreeChangeDetector_->addPointsFromInputCloud();
		newPointIdxVector_.clear();
		// Get vector of point indices from octreeChangeDetector_ voxels which did not exist in previous buffer
		octreeChangeDetector_->getPointIndicesFromNewVoxels(newPointIdxVector_);
		disappeared_points_.clear();
		for (std::size_t i = 0; i < newPointIdxVector_.size(); ++i) {
			//color[3 * newPointIdxVector_[i] + 0] = 255 - color[3 * newPointIdxVector_[i] + 0];
			//color[3 * newPointIdxVector_[i] + 1] = 255 - color[3 * newPointIdxVector_[i] + 1];
			//color[3 * newPointIdxVector_[i] + 2] = 255 - color[3 * newPointIdxVector_[i] + 2];
		}
		*cloud_prev_ = *cloud_main_;
	}



	//最大の平面除去 9
	void _remove_plane() {
		if (!enable_[PCL_ID_RemovePlane]) return;
		if (!initialized_[PCL_ID_RemovePlane]) {
			initialized_[PCL_ID_RemovePlane] = true;
		}

		for(int i=0;i<1;i++) {
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			// Create the segmentation object
			pcl::SACSegmentation<PointTypeT> seg;
			// Optional
			seg.setOptimizeCoefficients(true);
			// Mandatory
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.002);

			seg.setInputCloud(cloud_main_);
			seg.segment(*inliers, *coefficients);

			std::cerr << inliers->indices.size() << std::endl;
			if (inliers->indices.size() > 0.3 * cloud_main_->size()) {
				CloudPtr cloud_main_filtered(new pcl::PointCloud<PointTypeT>);
				pcl::ExtractIndices<PointTypeT> extract;
				extract.setInputCloud(cloud_main_);
				extract.setIndices(inliers);
				extract.setNegative(true);
				extract.filter(*cloud_main_filtered);
				cloud_main_->resize(0);
				cloud_main_ = cloud_main_filtered;
			}
			else break;
		}
	}



	//cloud_main_を描画．要コンストラクタの初期化 10
	//viewerのwrapper
	/*
	         void
         setBackgroundColor (const double &r, const double &g, const double &b, int viewport = 0);
	*/
	pcl::visualization::CloudViewer* cViewer_;
	bool showCoordinate_ = true;
	void drawResult(){
		if (!enable_[PCL_ID_DrawResult]) return;
		if (!initialized_[PCL_ID_DrawResult]) {
			cViewer_ = new pcl::visualization::CloudViewer("PCL2::DrawResult");
			showCoordinate_ = ROSParam::getIntParam("PCL_ShowCoordinate");
			if (!showCoordinate_) {
				//cViewer_->removeVisualizationCallable("reference");
				auto f = [&](pcl::visualization::PCLVisualizer& vis)->void {
					vis.removeAllCoordinateSystems();
					array<double, 3> rgb;
					rgb[0] = ROSParam::getDoubleParam("PCL_BG_R");
					rgb[1] = ROSParam::getDoubleParam("PCL_BG_G");
					rgb[2] = ROSParam::getDoubleParam("PCL_BG_B");
					vis.setBackgroundColor(rgb[0], rgb[1], rgb[2]);
				};
				cViewer_->runOnVisualizationThreadOnce(f);
			}
			initialized_[PCL_ID_DrawResult] = true;
		}
		cViewer_->showCloud(cloud_main_);
	}



	//recognite 用 11
	//typedef pcl::PointXYZRGBA PointType;
	typedef pcl::Normal NormalType;
	typedef pcl::ReferenceFrame RFType;
	typedef pcl::SHOT352 DescriptorType;
	bool show_keypoints_;
	bool show_correspondences_;
	bool use_cloud_resolution_;
	bool use_hough_;
	float model_ss_;
	float scene_ss_;
	float rf_rad_;
	float descr_rad_;
	float cg_size_;
	float cg_thresh_;
	CloudPtr model;
	CloudPtr model_keypoints;
	CloudPtr scene;
	CloudPtr scene_keypoints;
	pcl::PointCloud<NormalType>::Ptr model_normals;
	pcl::PointCloud<NormalType>::Ptr scene_normals;
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors;
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors;
	pcl::visualization::PCLVisualizer::Ptr recViewer_;

	double _computeCloudResolution(const CloudConstPtr& cloud)
	{
		double res = 0.0;
		int n_points = 0;
		int nres;
		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		pcl::search::KdTree<PointTypeT> tree;
		tree.setInputCloud(cloud);

		for (std::size_t i = 0; i < cloud->size(); ++i)
		{
			if (!std::isfinite((*cloud)[i].x))
			{
				continue;
			}
			//Considering the second neighbor since the first is the point itself.
			nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
			if (nres == 2)
			{
				res += sqrt(sqr_distances[1]);
				++n_points;
			}
		}
		if (n_points != 0)
		{
			res /= n_points;
		}
		return res;
	}
	void _recognite() {
		//https://pcl.readthedocs.io/projects/tutorials/en/latest/correspondence_grouping.html#correspondence-grouping
		if (!enable_[PCL_ID_Recognite]) return;
		if (!initialized_[PCL_ID_Recognite]) {
			show_keypoints_ = (true);
			show_correspondences_ = (true);
			use_cloud_resolution_ = (false);
			use_hough_ = (false);


			model.reset(new Cloud);
			model_keypoints.reset(new Cloud);
			scene.reset(new Cloud);
			scene_keypoints.reset(new Cloud);
			model_normals = pcl::PointCloud<NormalType>::Ptr(new pcl::PointCloud<NormalType>());
			scene_normals = pcl::PointCloud<NormalType>::Ptr(new pcl::PointCloud<NormalType>());
			model_descriptors = pcl::PointCloud<DescriptorType>::Ptr(new pcl::PointCloud<DescriptorType>());
			scene_descriptors = pcl::PointCloud<DescriptorType>::Ptr(new pcl::PointCloud<DescriptorType>());

			if (cloud_main_->size() < 10) return;

			//if (pcl::io::loadPCDFile("src/pointpkg/pcd/MS_xyzrgba_common/tool_target.pcd", *model) < 0) {
			if (pcl::io::loadPCDFile(ROSParam::getStringParam("PCL_pcd_target"), *model) < 0) {
				std::cerr << "Error loading model cloud." << std::endl;
			}
			model_ss_ = (0.005f);
			scene_ss_ = (0.005f);
			rf_rad_ = (0.005f);
			descr_rad_ = (0.01f);
			cg_size_ = (0.005f);
			cg_thresh_ = (0.05f);


			//if (pcl::io::loadPCDFile("milk_color.pcd", *model) < 0){
			//	std::cerr << "Error loading model cloud." << std::endl;
			//	return;
			//}
			//if (pcl::io::loadPCDFile("milk_cartoon_all_small_clorox.pcd", *scene) < 0)
			//{
			//	std::cerr << "Error loading model cloud." << std::endl;
			//	return;
			//}
			//model_ss_ = (0.01f);
			//scene_ss_ = (0.03f);
			//rf_rad_ = (0.015f);
			//descr_rad_ = (0.02f);
			//cg_size_ = (0.01f);
			//cg_thresh_ = (5.f);



			//
			//  Set up resolution invariance
			//
			if (use_cloud_resolution_)
			{
				float resolution = static_cast<float> (_computeCloudResolution(model));
				if (resolution != 0.0f)
				{
					model_ss_ *= resolution;
					scene_ss_ *= resolution;
					rf_rad_ *= resolution;
					descr_rad_ *= resolution;
					cg_size_ *= resolution;
				}

				std::cerr << "Model resolution:       " << resolution << std::endl;
				std::cerr << "Model sampling size:    " << model_ss_ << std::endl;
				std::cerr << "Scene sampling size:    " << scene_ss_ << std::endl;
				std::cerr << "LRF support radius:     " << rf_rad_ << std::endl;
				std::cerr << "SHOT descriptor radius: " << descr_rad_ << std::endl;
				std::cerr << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
			}

			pcl::UniformSampling<PointTypeT> uniform_sampling;
			uniform_sampling.setInputCloud(model);
			uniform_sampling.setRadiusSearch(model_ss_);
			uniform_sampling.filter(*model_keypoints);
			std::cerr << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

			recViewer_.reset(new pcl::visualization::PCLVisualizer("recViewer"));

			std::cerr << scene->size() << "   aaa" << std::endl;
			initialized_[PCL_ID_Recognite] = true;
		}
		else {
			//viewer.removeAllPointClouds();
		}
		*scene = *cloud_main_;







		//  Compute Normals
		pcl::NormalEstimationOMP<PointTypeT, NormalType> norm_est;
		norm_est.setKSearch(100);
		norm_est.setInputCloud(model);
		norm_est.compute(*model_normals);

		norm_est.setInputCloud(scene);
		norm_est.compute(*scene_normals);




		//  Downsample Clouds to Extract keypoints


		pcl::UniformSampling<PointTypeT> uniform_sampling;
		uniform_sampling.setInputCloud(scene);
		uniform_sampling.setRadiusSearch(scene_ss_);
		uniform_sampling.filter(*scene_keypoints);
		std::cerr << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

		//return;
		//
	//  Compute Descriptor for keypoints
	//
		pcl::SHOTEstimationOMP<PointTypeT, NormalType, DescriptorType> descr_est;
		descr_est.setRadiusSearch(descr_rad_);

		descr_est.setInputCloud(model_keypoints);
		descr_est.setInputNormals(model_normals);
		descr_est.setSearchSurface(model);
		descr_est.compute(*model_descriptors);

		descr_est.setInputCloud(scene_keypoints);
		descr_est.setInputNormals(scene_normals);
		descr_est.setSearchSurface(scene);
		descr_est.compute(*scene_descriptors);

		std::cerr << scene->size() << std::endl;

		//return;


	//  Find Model-Scene Correspondences with KdTree

		pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

		pcl::KdTreeFLANN<DescriptorType> match_search;
		match_search.setInputCloud(model_descriptors);

		//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
		for (std::size_t i = 0; i < scene_descriptors->size(); ++i)
		{
			std::vector<int> neigh_indices(1);
			std::vector<float> neigh_sqr_dists(1);
			if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
			{
				continue;
			}
			int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
			if (found_neighs == 1 && neigh_sqr_dists[0] < 0.1f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			{
				pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
				model_scene_corrs->push_back(corr);
			}
		}
		std::cerr << "Correspondences found: " << model_scene_corrs->size() << std::endl;


		//
		//  Actual Clustering
		//
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
		std::vector<pcl::Correspondences> clustered_corrs;

		//  Using Hough3D
		if (use_hough_)
		{
			//
			//  Compute (Keypoints) Reference Frames only for Hough
			//
			pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
			pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

			pcl::BOARDLocalReferenceFrameEstimation<PointTypeT, NormalType, RFType> rf_est;
			rf_est.setFindHoles(true);
			rf_est.setRadiusSearch(rf_rad_);

			rf_est.setInputCloud(model_keypoints);
			rf_est.setInputNormals(model_normals);
			rf_est.setSearchSurface(model);
			rf_est.compute(*model_rf);

			rf_est.setInputCloud(scene_keypoints);
			rf_est.setInputNormals(scene_normals);
			rf_est.setSearchSurface(scene);
			rf_est.compute(*scene_rf);

			//  Clustering
			pcl::Hough3DGrouping<PointTypeT, PointTypeT, RFType, RFType> clusterer;
			clusterer.setHoughBinSize(cg_size_);
			clusterer.setHoughThreshold(cg_thresh_);
			clusterer.setUseInterpolation(true);
			clusterer.setUseDistanceWeight(false);

			clusterer.setInputCloud(model_keypoints);
			clusterer.setInputRf(model_rf);
			clusterer.setSceneCloud(scene_keypoints);
			clusterer.setSceneRf(scene_rf);
			clusterer.setModelSceneCorrespondences(model_scene_corrs);

			//clusterer.cluster (clustered_corrs);
			clusterer.recognize(rototranslations, clustered_corrs);
		}
		else // Using GeometricConsistency
		{
			pcl::GeometricConsistencyGrouping<PointTypeT, PointTypeT> gc_clusterer;
			gc_clusterer.setGCSize(cg_size_);
			gc_clusterer.setGCThreshold(cg_thresh_);

			gc_clusterer.setInputCloud(model_keypoints);
			gc_clusterer.setSceneCloud(scene_keypoints);
			gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

			//gc_clusterer.cluster (clustered_corrs);
			gc_clusterer.recognize(rototranslations, clustered_corrs);
		}

		//
		//  Output results
		//
		std::cerr << "Model instances found: " << rototranslations.size() << std::endl;
		for (std::size_t i = 0; i < rototranslations.size(); ++i)
		{
			std::cerr << "\n    Instance " << i + 1 << ":" << std::endl;
			std::cerr << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

			// Print the rotation matrix and translation vector
			Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
			Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

			printf("\n");
			printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
			printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
			printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
			printf("\n");
			printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
		}

		//
	//  Visualization
	//
		//viewer.addPointCloud(scene, "scene_cloud");

		CloudPtr off_scene_model(new Cloud);
		CloudPtr off_scene_model_keypoints(new Cloud);

		if (show_correspondences_ || show_keypoints_)
		{
			//  We are translating the model so that it doesn't end in the middle of the scene representation
			pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
			pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

			pcl::visualization::PointCloudColorHandlerCustom<PointTypeT> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
			recViewer_->addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
			//cViewer_->showCloud(off_scene_model);
		}

		if (false&&show_keypoints_)
		{
			pcl::visualization::PointCloudColorHandlerCustom<PointTypeT> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
			recViewer_->addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
			recViewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

			pcl::visualization::PointCloudColorHandlerCustom<PointTypeT> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
			recViewer_->addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
			recViewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
		}
		// for 0 loop
		for (std::size_t i = 0; i < rototranslations.size(); ++i)
		{
			float r00 = abs(rototranslations[i].block<3, 3>(0, 0)(0, 0) - 1.00);
			float eps = 0.0001;
			if (r00 < eps) continue;
			CloudPtr rotated_model(new Cloud);
			pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);

			std::stringstream ss_cloud;
			ss_cloud << "instance" << i;

			pcl::visualization::PointCloudColorHandlerCustom<PointTypeT> rotated_model_color_handler(rotated_model, 255, 0, 0);
			recViewer_->addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

			if (show_correspondences_)
			{
				for (std::size_t j = 0; j < clustered_corrs[i].size(); ++j)
				{
					std::stringstream ss_line;
					ss_line << "correspondence_line" << i << "_" << j;
					PointTypeT& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
					PointTypeT& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);

					//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
					recViewer_->addLine<PointTypeT, PointTypeT>(model_point, scene_point, 0, 255, 0, ss_line.str());
				}
			}
		}


		//return;

		if (!recViewer_->wasStopped())
		{
			std::cerr << "viewer loop" << std::endl;
			recViewer_->spinOnce();
		}
	}



	//txtからパラメータ読み込み．ほとんどtransform用12
	bool params_read_ = false;
	void _load_parameters(std::string file_path) {

		ifstream ifs(file_path);
		std::stringstream ss;
		std::string line;
		//std::getline(ifs, line);

		std::string parameter_name;
		std::string value;
		while (getline(ifs, line)) {
			std::istringstream i_stream(line);
			std::getline(i_stream, parameter_name, ' ');
			std::getline(i_stream, value);
			//std::cerr << parameter_name << " " << value << std::endl;

			if (parameter_name == "trans_x")   transformation_parameters_[0] = stof(value);
			if (parameter_name == "trans_y")   transformation_parameters_[1] = stof(value);
			if (parameter_name == "trans_z")   transformation_parameters_[2] = stof(value);
			if (parameter_name == "trans_ax")   transformation_parameters_[3] = stof(value);
			if (parameter_name == "trans_ay")   transformation_parameters_[4] = stof(value);
			if (parameter_name == "trans_az")   transformation_parameters_[5] = stof(value);

			if (parameter_name == "thresh_min_x")   threshold_[0] = stof(value);
			if (parameter_name == "thresh_max_x")   threshold_[1] = stof(value);
			if (parameter_name == "thresh_min_y")   threshold_[2] = stof(value);
			if (parameter_name == "thresh_max_y")   threshold_[3] = stof(value);
			if (parameter_name == "thresh_min_z")   threshold_[4] = stof(value);
			if (parameter_name == "thresh_max_z")   threshold_[5] = stof(value);

		}

		std::cerr << "pcl; parameters loaded" << std::endl;

		params_read_ = true;
	}



	//txtにパラメータ保存．ほとんどtransform用13
	void _save_parameters(std::string file_path) {
		if (!params_read_) return;

		PL("PCL saving parameters")

		ofstream ofs(file_path);
		ofs << std::fixed << std::setprecision(6);


		for (int i = 0; i < 6; i++) if (abs(transformation_parameters_[i]) < 0.00001) transformation_parameters_[i] = 0.0;
		ofs << "trans_x " << transformation_parameters_[0] << endl;
		ofs << "trans_y " << transformation_parameters_[1] << endl;
		ofs << "trans_z " << transformation_parameters_[2] << endl;
		ofs << "trans_ax " << transformation_parameters_[3] << endl;
		ofs << "trans_ay " << transformation_parameters_[4] << endl;
		ofs << "trans_az " << transformation_parameters_[5] << endl;

		for (int i = 0; i < 6; i++) if (abs(threshold_[i]) < 0.00001) threshold_[i] = 0.0;
		ofs << "thresh_min_x " << threshold_[0] << endl;
		ofs << "thresh_max_x " << threshold_[1] << endl;
		ofs << "thresh_min_y " << threshold_[2] << endl;
		ofs << "thresh_max_y " << threshold_[3] << endl;
		ofs << "thresh_min_z " << threshold_[4] << endl;
		ofs << "thresh_max_z " << threshold_[5] << endl;
		ofs.close();

	}



	//ROSParam14
	void _setROSParam() {
		folder_read_ = ROSParam::getStringParam("PCL_pcd_read");
		folder_write_ = ROSParam::getStringParam("PCL_pcd_write");
		this->_load_parameters(ROSParam::getStringParam("PCL_param_txt"));

		enable_[PCL_ID_ReadPCD] = ROSParam::getIntParam("PCL_enable_ReadPCD");
		enable_[PCL_ID_WritePCD] = ROSParam::getIntParam("PCL_enable_WritePCD");
		enable_[PCL_ID_Transform] = ROSParam::getIntParam("PCL_enable_Transform");
		enable_[PCL_ID_PassThrough] = ROSParam::getIntParam("PCL_enable_PassThrough");
		enable_[PCL_ID_ParticleFilter] = ROSParam::getIntParam("PCL_enable_ParticleFilter");
		enable_[PCL_ID_DetectWithSim] = ROSParam::getIntParam("PCL_enable_DetectWithSim");
		enable_[PCL_ID_RemovePlane] = ROSParam::getIntParam("PCL_enable_RemovePlane");
		enable_[PCL_ID_Recognite] = ROSParam::getIntParam("PCL_enable_Recognite");
		enable_[PCL_ID_DrawResult] = ROSParam::getIntParam("PCL_enable_DrawResult");
		enable_[PCL_ID_DetectWithYOLO] = ROSParam::getIntParam("PCL_enable_DetectWithYOLO");
		enable_[PCL_ID_CalibrationHorizon] = ROSParam::getIntParam("PCL_enable_CalibrationHorizon");
		enable_[PCL_ID_CalibrationVertical] = ROSParam::getIntParam("PCL_enable_CalibrationVertical");
		
		//EL(enable_)
	}



	//vectorからpcdへ変換(cloud) 15
	//YOLO用に色が範囲外だったらcloud_yolo_に追加
	void _convert_array_to_pcd(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
		cloud_main_->clear();
		for (int i = 0; i < 2; i++)if (cloud_yolo_[i]->size() != 0) cloud_yolo_[i]->clear();
		if (points.size() == 0) return;

		int sz = int(points.size()) / 3;

		//cloud_main_->resize(sz);

		const int mask = 255;
		int cnt = 0;

		for (int i = 0; i < sz; i++) {
			//0<=r<=255ならツール無関係の点
			if (color[3 * i]<256) {
				cnt++;
				if (cnt % 3 == 0) continue;
				cloud_main_->emplace_back(PointTypeT());
				auto& p = cloud_main_->back();
				p.x = points[3 * i + 0];
				p.y = points[3 * i + 1];
				p.z = points[3 * i + 2];
				p.r = color[3 * i + 0];
				p.g = color[3 * i + 1];
				p.b = color[3 * i + 2];
				p.a = 255;
			}
			//ツール 1(Yellow&Green)
			else if(color[3 * i] < 512) {
				cloud_yolo_[0]->emplace_back(PointTypeT());
				auto& p = cloud_yolo_[0]->back();
				p.x = points[3 * i + 0];
				p.y = points[3 * i + 1];
				p.z = points[3 * i + 2];
				p.r = color[3 * i + 0] & mask;
				p.g = color[3 * i + 1] & mask;
				p.b = color[3 * i + 2] & mask;
				p.a = 255;
			}
			//ツール 2(Yellow&Green)
			else{//r<1024
				cloud_yolo_[1]->emplace_back(PointTypeT());
				auto& p = cloud_yolo_[1]->back();
				p.x = points[3 * i + 0];
				p.y = points[3 * i + 1];
				p.z = points[3 * i + 2];
				p.r = color[3 * i + 0] & mask;
				p.g = color[3 * i + 1] & mask;
				p.b = color[3 * i + 2] & mask;
				p.a = 255;
			}
		}
	}



	//pcdからvectorへ変換 16
	void _convert_pcd_to_array(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
		points.clear();
		color.clear();
		if (cloud_main_->size() == 0) return;

		points.resize(cloud_main_->size() * 3);
		color.resize(cloud_main_->size() * 3);

		int sz = cloud_main_->size();
		for (int i = 0; i < sz;i++) {
			points[3 * i + 0] = (*cloud_main_)[i].x;
			points[3 * i + 1] = (*cloud_main_)[i].y;
			points[3 * i + 2] = (*cloud_main_)[i].z;
			color[3 * i + 0] = (*cloud_main_)[i].r;
			color[3 * i + 1] = (*cloud_main_)[i].g;
			color[3 * i + 2] = (*cloud_main_)[i].b;
		}
		number_of_points = points.size() / 3;
	}



	//RGBデータからYOLOでツール位置を推定したうえで接触判定をする 17
	std::array<CloudPtr, 2> cloud_yolo_;//鉗子，剪刀生データ格納用(マーカーx2,先端位置)
	CloudPtr cloud_yolo_sub_;//transform用
	std::array<std::unique_ptr<ContactDetector2<PointTypeT>>, 2> contact_detector_yolo_;
	struct LastToolPose {
		LastToolPose() = default;
		bool found_ = false;
		std::array<std::array<double, 3>,3> positions_;//(tip,M1,M2) x (x,y,z)
		void update(CloudPtr& cloud) {
			//ES("update") ES(cloud->size())
			if (found_) {
				double w = 2.0;//前回のデータに対して，何倍の影響力を持たせるか
				for (int i = 0; i < 3; i++) {
					(*cloud)[i].x = ((*cloud)[i].x * w + positions_[i][0]) / 3;
					(*cloud)[i].y = ((*cloud)[i].y * w + positions_[i][1]) / 3;
					(*cloud)[i].z = ((*cloud)[i].z * w + positions_[i][2]) / 3;
				}
			}
			found_ = true;
			for (int i = 0; i < 3; i++) {//tip,M1,M2
				positions_[i][0] = (*cloud)[i].x;
				positions_[i][1] = (*cloud)[i].y;
				positions_[i][2] = (*cloud)[i].z;
			}
			//ES("escape")
		}
		array<queue<array<double, 3>>,3> que;
		int len = 3;
		void update2(CloudPtr& cloud) {
			if (!found_) {
				for (int i = 0; i < 3; i++) {
					while (!que[i].empty()) que[i].pop();
					for (int j = 0; j < 3; j++) positions_[i][j] = 0.0;
				}
			}
			
			for (int i = 0; i < 3; i++) {
				array<double, 3> arr;
				arr[0] = (*cloud)[i].x;
				arr[1] = (*cloud)[i].y;
				arr[2] = (*cloud)[i].z;
				que[i].push(arr);
				positions_[i] = positions_[i] + arr;
				while (que[i].size() > len) {
					positions_[i] = positions_[i] - que[i].front();
que[i].pop();
				}
				(*cloud)[i].x = positions_[i][0] / que[i].size();
				(*cloud)[i].y = positions_[i][1] / que[i].size();
				(*cloud)[i].z = positions_[i][2] / que[i].size();
			}

		}
	};
	std::array<LastToolPose, 2> lastToolPoses_;
	bool calibrationYOLO_ = false;
	int calibrationStep_ = 0;
	int calibrationLoopCnt_ = 0;
	bool showTipPosition_[2];
	bool showMarkersMeanPosition_[2];
	bool calibrateO_ = false;
	bool calibrateRotZ_ = false;
	int cnt = 0;
	bool show200Frames_ = false;
	void _detect_contact_with_YOLO() {
		auto time = clock();
		//PL("a")
		if (!enable_[PCL_ID_DetectWithYOLO]) return;
		if (!initialized_[PCL_ID_DetectWithYOLO]) {
			//ROSParamのパラメータ名
			std::string rgb_param_names[2] = { "PCL_YOLO_RGB_YG", "PCL_YOLO_RGB_BP" };
			for (int i = 0; i < 2; i++) {
				//rrrgggbbbの形式(ex:255255255, 255000255, 255255)
				int rgb = ROSParam::getIntParam(rgb_param_names[i]);
				int b = rgb % 1000; rgb /= 1000;
				int g = rgb % 1000; rgb /= 1000;
				int r = rgb % 1000; rgb /= 1000;
				PS("pcl 1195") ES(r) ES(g) EL(b)
					std::array<int, 3> arr_rgb; arr_rgb[0] = r, arr_rgb[1] = g, arr_rgb[2] = b;
				contact_detector_yolo_[i].reset(new ContactDetector2<PointTypeT>(
					ROSParam::getStringParam("PCL_pcd_target")
					, ROSParam::getStringParam("PCL_pcd_mask")
					, ROSParam::getStringParam("PCL_pcd_tip")
					, arr_rgb
					));
			}
			calibrateO_ = ROSParam::getIntParam("PCL_CalibrateO");
			calibrateRotZ_ = ROSParam::getIntParam("PCL_CalibrateRotZ");
			show200Frames_ = ROSParam::getIntParam("PCL_Show200Frames");
			if (calibrateO_ & calibrateRotZ_) calibrateO_ = false;
			for (int i = 0; i < 2; i++) showMarkersMeanPosition_[i] = ROSParam::getIntParam("PCL_ShowMarkersMeanPosition" + to_string(i));
			calibrationYOLO_ = ROSParam::getIntParam("PCL_CalibrationYOLO");
			showTipPosition_[0] = ROSParam::getIntParam("PCL_ShowTipPosition0");
			showTipPosition_[1] = ROSParam::getIntParam("PCL_ShowTipPosition1");
			initialized_[PCL_ID_DetectWithYOLO] = true;
		}

		for (int i = 0; i < 2; i++) {//right, left
			if (cloud_yolo_[i]->size() != 3) {
				lastToolPoses_[i].found_ = false;
				continue;
			}

			//並べ替え Tip, M1(Yellow), M2(Green)の順で
			sort(cloud_yolo_[i]->begin(), cloud_yolo_[i]->end(), [](PointTypeT& a, PointTypeT& b) {
				return a.r > b.r;
				});
			_transform(cloud_yolo_[i], cloud_yolo_sub_);

			lastToolPoses_[i].update2(cloud_yolo_[i]);

			//PS("pcl yolo") PL(clock() - time)


			/*
			RealSenseInterfaceでツール上のマーカー（Yellow, Green）を検出．
			マーカー及びツール先端予測位置を点群データに追加．その際，色データに256(==0b100000000 == 1<<8)足しておく．

			PCL2::convertArrayToPCDで，255を超える色データがあった場合，color&255としてcloud_yolo_に追加．
			実用上，接触判定には位置データが必要であり，色データは不要．

			*/

			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			//Green->Yellowの方向ベクトル
			Eigen::Vector3f GY((*cloud_yolo_[i])[1].x - (*cloud_yolo_[i])[2].x, (*cloud_yolo_[i])[1].y - (*cloud_yolo_[i])[2].y, (*cloud_yolo_[i])[1].z - (*cloud_yolo_[i])[2].z);
			//ツール初期姿勢（y方向正）からの姿勢変化クォータニオン
			Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitY(), GY);
			transform *= q.matrix();

			/*
			ツールが上向きになるように回転させたい
			z軸(0,0,1)をtransformさせたときに，z座標が最も高くなる軸周り回転計算．
			二分探索．単調性を持たなければいけないので，abs(l-r)=r-l<M_PIを満たさないと多分壊れる．
			*/
			PointTypeT p;
			p.x = 0, p.y = 0, p.z = 1;
			float l = -1.0, r = 1.0, eps = 1e-5, d = 1e-4;
			CloudPtr a(new Cloud()), b(new Cloud());
			a->push_back(p);
			Eigen::Affine3f t;
			auto f = [&](float angle)->float {
				t = Eigen::Affine3f::Identity();
				t.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
				t = transform * t;
				pcl::transformPointCloud(*a, *b, t);
				return (*b)[0].z;
			};
			while (f((l + r) / 2) < 0.0) {
				l += M_PI * 2 / 3;
				r += M_PI * 2 / 3;
			}
			while (r - l > eps) {
				float mid = (l + r) / 2;
				if (f(l) >= f(r)) r = mid;
				else l = mid;
			}
			t = Eigen::Affine3f::Identity();
			t.rotate(Eigen::AngleAxisf(l, Eigen::Vector3f::UnitY()));
			transform = transform * t;
			transform.translation() << (*cloud_yolo_[i])[0].x, (*cloud_yolo_[i])[0].y, (*cloud_yolo_[i])[0].z;
			contact_detector_yolo_[i]->transform_init(transform);

			//PS("pcl yolo") PL(clock() - time)


			//z軸最低点周りを探索する．
			array<float, 3> tip = { (*cloud_yolo_[i])[0].x, (*cloud_yolo_[i])[0].y, (*cloud_yolo_[i])[0].z };
			auto lowest = contact_detector_yolo_[i]->get_lowest_point();
			//EL(tip);
			//EL(lowest);
			if (norm(tip - lowest) < 0.02) {
				swap(tip, lowest);
				//PL("change")
			}
			//EL(tip);
			contact_detector_yolo_[i]->set_tip_position(tip);

			//実際に接触判定は別クラス内で行う
			contact_detector_yolo_[i]->remove_and_detect(cloud_main_);

			//PS("pcl yolo") PL(clock() - time)



			//以下はデバッグ及びキャリブレーション用

			if (showTipPosition_[i]) {
				for (int j = 0; j < 3; j++) {
					//PS(i) PS(j) PS((*cloud_yolo_[i])[j].x) PS((*cloud_yolo_[i])[j].y) PS((*cloud_yolo_[i])[j].z) PL(pow(pow((*cloud_yolo_[i])[j].x * 1000, 2.) + pow((*cloud_yolo_[i])[j].y * 1000, 2.) + pow((*cloud_yolo_[i])[j].z * 1000, 2.),1.0/2));
					PS(j) PS((*cloud_yolo_[i])[j].x * 1000.) PS((*cloud_yolo_[i])[j].y * 1000.) PL((*cloud_yolo_[i])[j].z * 1000.)
				}
				//ES(i) ES((*cloud_yolo_[i])[0].x) ES((*cloud_yolo_[i])[0].y) EL((*cloud_yolo_[i])[0].z)
			}

			if (show200Frames_) {
				if (20 <= cnt && cnt < 220) {
					for (int j = 0; j < 3; j++) {
						PS(j) PS((*cloud_yolo_[i])[j].x*1000.) PS((*cloud_yolo_[i])[j].y * 1000.) PL((*cloud_yolo_[i])[j].z * 1000.)
					}
				}
				cnt++;
			}

			//必要に応じて，マーカーなどをMainの点群に追加しておく．
			if (showMarkersMeanPosition_[i]) for (const auto& point : *cloud_yolo_[i]) {
				//cloud_main_->emplace_back(p);
				//EL(int(cloud_main_->back().a))
				for (double r = 0.0002; r <= 0.0004; r += 0.0001) {
					for (double a = 0; a < M_PI * 2; a += 0.02) {
						for (double b = 0; b < M_PI * 2; b += 0.02) {
							PointTypeT p = point;
							p.x += r * cos(a) * sin(b);
							p.y += r * sin(a) * sin(b);
							p.z += r * cos(b);
							cloud_main_->emplace_back(p);
						}
					}
				}
			}

			if (calibrateO_) {
				transformation_parameters_[0] += -(*cloud_yolo_[i])[0].x * 0.1;
				transformation_parameters_[1] += -(*cloud_yolo_[i])[0].y * 0.1;
				transformation_parameters_[2] += -(*cloud_yolo_[i])[0].z * 0.1;
			}

			if (calibrateRotZ_) {
				//double sig = (*cloud_yolo_[i])[0].x / abs((*cloud_yolo_[i])[0].x) * (*cloud_yolo_[i])[0].y / abs((*cloud_yolo_[i])[0].y);
				transformation_parameters_[5] += -atan2((*cloud_yolo_[i])[0].y, (*cloud_yolo_[i])[0].x) * 0.1;
			}

			
			//no more used calibration function
			if (false && calibrationYOLO_ && i == 0) {
				const auto& tip = (*cloud_yolo_[i])[0];
				ES(tip.x) ES(tip.y) EL(tip.z);
				if (calibrationStep_ == 0) {
					//PL("0:回転をすべて0にする");
					for (int i = 3; i < 6; i++) transformation_parameters_[i] = 0.0;
					calibrationLoopCnt_ = 0;
					calibrationStep_++;
				}
				else if (calibrationStep_ == 1) {
					//PL("1:ツール先端が原点に来るように移動して，HOMEを押す")
					transformation_parameters_[0] -= tip.x;
					transformation_parameters_[1] -= tip.y;
					transformation_parameters_[2] -= tip.z;
					calibrationLoopCnt_ = 0;
					calibrationStep_++;
				}
				else if (calibrationStep_ == 2) {
					PL("2:ツールをx方向に平行移動させた後，y=0になるようにz軸回転．HOMEを押す")
				}
				else if (calibrationStep_ == 3) {
					PL("3:ツールは動かさず，z=0になるようにy軸回転．HOMEを押す")
				}
				else if (calibrationStep_ == 4) {
					PL("4:ツールをy方向に平行移動させた後，z=0になるようにx軸回転．HOMEを押す")
				}
				else {
					calibrationYOLO_ = false;
					continue;
				}

				if ((bool)(GetKeyState(VK_HOME) & 0x8000) && calibrationLoopCnt_ > 30) {
					calibrationLoopCnt_ = 0;
					calibrationStep_++;
					EL(calibrationStep_)
				}
				calibrationLoopCnt_++;
			}
		}
	}



	//水平面キャリブレーション 18
	//最大平面とxy平面を一致させる
	double dist_horizon_ = 0.1;
	void _horizontal_calibration() {
		if (!enable_[PCL_ID_CalibrationHorizon]) return;
		if (!initialized_[PCL_ID_CalibrationHorizon]) {
			initialized_[PCL_ID_CalibrationHorizon] = true;
		}

		for (int i = 0; i < 1; i++) {
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			// Create the segmentation object
			pcl::SACSegmentation<PointTypeT> seg;
			// Optional
			seg.setOptimizeCoefficients(true);
			// Mandatory
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.002);

			seg.setInputCloud(cloud_main_);
			seg.segment(*inliers, *coefficients);

			//ax+by+cz+d = 0
			PS(coefficients->values[0]);
			PS(coefficients->values[1]);
			PS(coefficients->values[2]);
			PL(coefficients->values[3]);

			transformation_parameters_[2] += coefficients->values[3] * 0.1;
			transformation_parameters_[3] += coefficients->values[1] * 0.1;
			transformation_parameters_[4] += coefficients->values[0] * 0.1;
		}
		dist_horizon_ *= 0.99;
		chmax(dist_horizon_, 0.001);
	}



	//水平面キャリブレーション 19
	//最大平面とyz平面を一致させる
	//horizonのキャリブレーション後に使う
	double dist_vertical_ = 0.1;
	void _vertical_calibration() {
		if (!enable_[PCL_ID_CalibrationVertical]) return;
		if (!initialized_[PCL_ID_CalibrationVertical]) {
			initialized_[PCL_ID_CalibrationVertical] = true;
		}

		for (int i = 0; i < 1; i++) {
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			// Create the segmentation object
			pcl::SACSegmentation<PointTypeT> seg;
			// Optional
			seg.setOptimizeCoefficients(true);
			// Mandatory
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.002);

			seg.setInputCloud(cloud_main_);
			seg.segment(*inliers, *coefficients);

			//ax+by+cz+d = 0
			PS(coefficients->values[0]);
			PS(coefficients->values[1]);
			PS(coefficients->values[2]);
			PL(coefficients->values[3]);
			//xz平面が原点通るように
			transformation_parameters_[1] -= coefficients->values[3] * 0.1;
			if (coefficients->values[1] != 0.0) {
				if (coefficients->values[0] / coefficients->values[1] > 0) transformation_parameters_[5] += dist_vertical_;
				else transformation_parameters_[5] -= dist_vertical_;
			}
			//transformation_parameters_[3] -= coefficients->values[2] * 0.1;
			//transformation_parameters_[5] -= coefficients->values[0] * 0.1;
		}
		dist_vertical_ *= 0.8;
		chmax(dist_vertical_, 0.001);
	}


	//適当に点群を編集するとき用 
	float tx = -0.08;
	float ty = -0.13;
	float tz = 0.15;
	float ZZ = -2.0;
	void _edit() {
		//0.0405
		//0.0825
		std::cerr << "edit" << std::endl;
		CloudPtr tmp(new pcl::PointCloud<PointTypeT>);
		for (const auto& p : *cloud_main_) {
			if (p.y < 0.) tmp->push_back(p);
			ZZ = std::max(ZZ, p.z);
		}
		cloud_main_ = tmp;
		std::cerr << ZZ << std::endl;
	}



	



	void _get_tip_pose();

public:
	PCL2();
	~PCL2();
	void setSimPtr(CoppeliaSimInterface* coppeliasim_interface);
	void update(int& number_of_points, std::vector<float>& points, std::vector<int>& color);
	//void update_RGB(int& number_of_points, float** points);

	int get_points_size() { return points_.size() / 3; }
	float* get_points() { return &points_[0]; }
	int get_appeared_points_size() { return appeared_points_.size() / 3; }
	float* get_appeared_points() { return &appeared_points_[0]; }
	int get_disappeared_points_size() { return disappeared_points_.size() / 3; }
	float* get_disappeared_points() { return &disappeared_points_[0]; }
	double* get_transformation_parameters(int n) { return &transformation_parameters_[n]; }
	bool* get_enable_filterPassThrough() { return &(enable_[PCL_ID_PassThrough]); }
	double* get_filterPassThrough_threshold(int i) { return &threshold_[i]; }
};




template<typename PointTypeT>
PCL2<PointTypeT>::PCL2() :
	//enable_(vector<bool>(30))
	//,initialized_(vector<bool>(30))
	//----basic point clouds init---
	cloud_main_(new Cloud())
	, cloud_sub_(new Cloud())
	//detect contact with yolo
	, contact_detector_(new ContactDetector2<PointTypeT>(
		ROSParam::getStringParam("PCL_pcd_target")
		, ROSParam::getStringParam("PCL_pcd_mask")
		, ROSParam::getStringParam("PCL_pcd_tip")
		))
{
	std::cerr << "pcl: constructing" << std::endl;

	for (int i = 0; i < 32; i++) enable_[i] = false;
	for (int i = 0; i < 32; i++) initialized_[i] = false;


	enable_[PCL_ID_PCL] = ROSParam::getIntParam("PCL_enable_PCL");
	if (!enable_[PCL_ID_PCL]) return;

	this->_setROSParam();

	for (int i = 0; i < 2; i++) cloud_yolo_[i].reset(new Cloud);
	cloud_yolo_sub_.reset(new Cloud);
	

	cloud_main_->reserve(1e6);
	cloud_sub_->reserve(1e6);

	initialized_[PCL_ID_PCL] = true;


	std::cerr << "pcl: constructed" << std::endl;

}



template<typename PointTypeT>
PCL2<PointTypeT>::~PCL2() {
	PL("~PCL2");
	this->_save_parameters(ROSParam::getStringParam("PCL_param_txt"));
}

template<typename PointTypeT>
void PCL2<PointTypeT>::setSimPtr(CoppeliaSimInterface* coppeliasim_interface) {
	coppeliasim_interface_ = coppeliasim_interface;
}

template<typename PointTypeT>
void PCL2<PointTypeT>::update(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	if (!enable_[PCL_ID_PCL]) return;
	if (!initialized_[PCL_ID_PCL]) return;

	cloud_main_->clear();

	//vevtor -> pcd 変換
	PCL2<PointTypeT>::_convert_array_to_pcd(number_of_points, points, color);


	//保存済みデータ読み込み
	//PCL2<PointTypeT>::_read_pcd_file("src/pointpkg/pcd/MS_xyzrgba5/1.pcd", number_of_points, color);
	//PCL2<PointTypeT>::_read_pcd_file("src/pointpkg/pcd/MS_xyzrgba5/", number_of_points, color);
	//PCL2<PointTypeT>::_read_pcd_file("src/pointpkg/pcd/MS_xyzrgba_common/tool_blue.pcd", number_of_points, color);
	PCL2<PointTypeT>::_read_pcd_file(folder_read_, number_of_points, color);


	//カメラ->coppeliasimモデル座標変換
	PCL2<PointTypeT>::_transform(cloud_main_, cloud_sub_);

	//範囲外除去
	PCL2<PointTypeT>::_filterPassThrough();

	//最大平面がxy平面と重なるように座標変換
	PCL2<PointTypeT>::_horizontal_calibration();
	PCL2<PointTypeT>::_vertical_calibration();
	PCL2<PointTypeT>::_remove_plane();
	//PCL2<PointTypeT>::_compress();
	//PCL2<PointTypeT>::_detect_change(color);
	//PCL2<PointTypeT>::_edit();
	PCL2<PointTypeT>::_recognite();

	//接触判定　(粒子フィルタ)
	PCL2<PointTypeT>::_detect_contact_with_particlefilter();


	//接触判定　(CoppeliaSim)
	PCL2<PointTypeT>::_detect_contact_with_coppeliasim();


	//接触判定　（YOLO）
	PCL2<PointTypeT>::_detect_contact_with_YOLO();


	//新規window上に点群描画．PCL2<PointTypeT>::PCL()のviewerの初期化必要
	PCL2<PointTypeT>::drawResult();

	//pcdデータ（cloud_main_）保存．  保存場所指定は最後に"/"忘れずに
	PCL2<PointTypeT>::_write_pcd_file(folder_write_);

	//pcd->vector 変換
	PCL2<PointTypeT>::_convert_pcd_to_array(number_of_points, points, color);

	//std::cerr << counter << std::endl;

}








