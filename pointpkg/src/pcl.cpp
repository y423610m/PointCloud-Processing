//pcl.cpp
#include <pcl.h>
#include <iostream>
#include <windows.h>
#include <numeric>
#include <cmath>
#include <string>
#include <fstream>
#include <algorithm>

#include "ros_param.h"
#include "development_commands.h"


PCL::PCL() :

	//----basic point clouds init---
	//cloud_xyz_(new pcl::PointCloud<pcl::PointXYZ>),
	//cloud_xyzrgb_(new pcl::PointCloud<pcl::PointXYZRGB>),
	cloud_xyzrgba_(new pcl::PointCloud<pcl::PointXYZRGBA>)

	//---track()---
	,tracker(new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>(8))
	,coherence(new ApproxNearestPairPointCloudCoherence<RefPointType>)
	,distance_coherence(new DistanceCoherence<RefPointType>())
	,hsv_coherence(new HSVColorCoherence<RefPointType>())
	,nearest_pair_coherence(new NearestPairPointCloudCoherence<RefPointType>)
	,search(new pcl::search::Octree<RefPointType>(0.01)),
	transed_ref(new Cloud),
	transed_ref_downsampled(new Cloud),
	cloud_target_remover_ref_(new Cloud),
	cloud_target_remover_ref_downsampled_(new Cloud),
	Rcenter_ref_(new Cloud)
	//,viewer(new pcl::visualization::CloudViewer("PCL"))
	, contact_detector_(new ContactDetector(ROSParam::getStringParam("PCL_pcd_target")
											, ROSParam::getStringParam("PCL_pcd_mask")
											, ROSParam::getStringParam("PCL_pcd_tip")
											))
{
	std::cerr << "pcl: constructing" << std::endl;


	enable_ |= ROSParam::getIntParam("PCL_enable_PCL");
	if (!enable_) return;

	this->_setROSParam();

	initialized_ |= 1;


	std::cerr << "pcl: constructed" << std::endl;

}

void PCL::_setROSParam() {
	pcd_type_ = ROSParam::getIntParam("PCL_pcd_type");
	folder_read_ = ROSParam::getStringParam("PCL_pcd_read");
	folder_write_ = ROSParam::getStringParam("PCL_pcd_write");
	PCL::_load_parameters(ROSParam::getStringParam("PCL_param_txt"));

	enable_ |= ROSParam::getIntParam("PCL_enable_read_pcd") << PCL_ENABLE_READ_PCD;
	enable_ |= ROSParam::getIntParam("PCL_enable_write_pcd") << PCL_ENABLE_WRITE_PCD;
	enable_ |= ROSParam::getIntParam("PCL_enable_transform") << PCL_ENABLE_TRANSFORM;
	enable_ |= ROSParam::getIntParam("PCL_enable_passthrough") << PCL_ENABLE_PASSTHROUGH;
	enable_ |= ROSParam::getIntParam("PCL_enable_particlefilter") << PCL_ENABLE_PARTICLEFILTER;
	enable_ |= ROSParam::getIntParam("PCL_enable_detect_with_sim") << PCL_ENABLE_DETECT_WITH_SIM;

	EL(enable_)
}

PCL::~PCL() {
	PCL::_save_parameters(ROSParam::getStringParam("PCL_param_txt"));
}

void PCL::init(CoppeliaSimInterface* coppeliasim_interface) {
	coppeliasim_interface_ = coppeliasim_interface;
}


void PCL::update(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	if (!(initialized_ & 1)) return;
	if (!enable_) return;

	cloud_xyzrgba_->clear();


	//vevtor -> pcd 変換
	PCL::_convert_array_to_pcd(number_of_points, points, color);

	//保存済みデータ読み込み
	//PCL::_read_pcd_file("src/pointpkg/pcd/MS_xyzrgba5/1.pcd", number_of_points, color);
	//PCL::_read_pcd_file("src/pointpkg/pcd/MS_xyzrgba5/", number_of_points, color);
	//PCL::_read_pcd_file("src/pointpkg/pcd/MS_xyzrgba_common/tool_blue.pcd", number_of_points, color);
	PCL::_read_pcd_file(folder_read_, number_of_points, color);
	

	//カメラ->coppeliasimモデル座標変換
	PCL::_transform();
		
	//範囲外除去
	PCL::_filterPassThrough();




	//PCL::_remove_plane();  //xyz only
	//PCL::_compress();
	//PCL::_detect_change(color);  //xyz only
	//PCL::_edit();
	//PCL::_recognite(); // to be improved

	//接触判定　(粒子フィルタ)
	PCL::_detect_contact_with_particlefilter();


	//接触判定　(CoppeliaSim)
	PCL::_detect_contact_with_coppeliasim();


	 

	//新規window上に点群描画．PCL::PCL()のviewerの初期化必要
	//PCL::drawResult();

	//pcdデータ（cloud_xyzrgba_）保存．  保存場所指定は最後に"/"忘れずに
	PCL::_write_pcd_file(folder_write_);

	//pcd->vector 変換
	PCL::_convert_pcd_to_array(number_of_points, points, color);

	//std::cerr << counter << std::endl;
	
}

//vectorからpcdへ変換
void PCL::_convert_array_to_pcd(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	if (points.size() == 0) return;

	if (pcd_type_ == PCL_XYZ) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
		//std::cerr << "array to pcd" << std::endl;
		cloud_tmp->width = number_of_points;
		cloud_tmp->height = 1;
		cloud_tmp->is_dense = false;
		cloud_tmp->points.resize(cloud_tmp->width * cloud_tmp->height);

		float px, py, pz;
		for (int i = 0; i < number_of_points; i++) {
			px = points[3 * i + 0];
			py = points[3 * i + 1];
			pz = points[3 * i + 2];
			cloud_tmp->points[i] = pcl::PointXYZ(px, py, pz);
		}
		//std::cerr << "px"<<px << std::endl;
		cloud_xyz_ = cloud_tmp;
		//std::cerr << cloud->size() << std::endl;
	}
	else if(pcd_type_ == PCL_XYZRGB){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
		//std::cerr << "array to pcd" << std::endl;
		cloud_tmp->width = number_of_points;
		cloud_tmp->height = 1;
		cloud_tmp->is_dense = false;
		cloud_tmp->points.resize(cloud_tmp->width * cloud_tmp->height);

		float px, py, pz;
		int r, g, b;
		for (int i = 0; i < number_of_points; i++) {
			px = points[3 * i + 0];
			py = points[3 * i + 1];
			pz = points[3 * i + 2];
			r = color[3 * i + 0];
			g = color[3 * i + 1];
			b = color[3 * i + 2];
			cloud_tmp->points[i] = pcl::PointXYZRGB(px, py, pz,r,g,b);
		}
		//std::cerr << "px"<<px << std::endl;
		cloud_xyzrgb_ = cloud_tmp;
		//std::cerr << cloud->size() << std::endl;
	}
	else if (pcd_type_ == PCL_XYZRGBA) {
		cloud_xyzrgba_->clear();

		float px, py, pz;
		int r, g, b;
		int a;
		for (int i = 0; i < number_of_points; i++) {
			px = points[3 * i + 0];
			py = points[3 * i + 1];
			pz = points[3 * i + 2];
			r = color[3 * i + 0];
			g = color[3 * i + 1];
			b = color[3 * i + 2];
			a = 255;
			cloud_xyzrgba_->push_back(pcl::PointXYZRGBA(px, py, pz, r, g, b, a));
		}
		//std::cerr << "px"<<px << std::endl;
		//cloud_xyzrgba_ = cloud_tmp;
		//std::cerr << cloud->size() << std::endl;
		//std::cerr << "array to pcd  " << cloud_xyzrgba_->size() << std::endl;

	}
}

//pcdからvectorへ変換
void PCL::_convert_pcd_to_array(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	if (cloud_xyzrgba_->size() == 0) return;

	//std::cerr << "pcd to array" << std::endl;
	points.clear();
	color.clear();

	if (pcd_type_ == PCL_XYZ) {

		for (int i = 0; i < cloud_xyz_->size(); i++) {
			points.push_back(cloud_xyz_->points[i].x);
			points.push_back(cloud_xyz_->points[i].y);
			points.push_back(cloud_xyz_->points[i].z);
		}
		number_of_points = points.size() / 3;
		//*color = *color;
		//std::cerr<< points.size()/3 <<std::endl;
	}
	else if(pcd_type_ == PCL_XYZRGB){
		for (int i = 0; i < cloud_xyzrgb_->size(); i++) {
			points.push_back(cloud_xyzrgb_->points[i].x);
			points.push_back(cloud_xyzrgb_->points[i].y);
			points.push_back(cloud_xyzrgb_->points[i].z);
			color.push_back(cloud_xyzrgb_->points[i].r);
			color.push_back(cloud_xyzrgb_->points[i].g);
			color.push_back(cloud_xyzrgb_->points[i].b);
		}

		number_of_points = points.size() / 3;
	}
	else if (pcd_type_ == PCL_XYZRGBA) {
		for(const auto& p: *cloud_xyzrgba_){
			points.push_back(p.x);
			points.push_back(p.y);
			points.push_back(p.z);
			color.push_back(p.r);
			color.push_back(p.g);
			color.push_back(p.b);
		}

		number_of_points = points.size() / 3;
		//std::cerr << "                               pcd to array  " << number_of_points << std::endl;
	}
}



//octreeによるデータ圧縮
void PCL::_compress() {
	double a = clock();
	std::stringstream compressedData_;

	if (pcd_type_ == PCL_XYZ) {
		if (cloud_xyz_->size() > 0) {
			pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octreeCompression_xyz_(pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR, true);
			octreeCompression_xyz_.encodePointCloud(cloud_xyz_, compressedData_);
			octreeCompression_xyz_.decodePointCloud(compressedData_, cloud_xyz_);
			//frame_++;
		}
	}
	else {
		if (cloud_xyzrgb_->size() > 0) {
			pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> octreeCompression_xyzrgb_(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR, false);
			octreeCompression_xyzrgb_.encodePointCloud(cloud_xyzrgb_, compressedData_);
			octreeCompression_xyzrgb_.decodePointCloud(compressedData_, cloud_xyzrgb_);
			//frame_++;
		}
	}
	std::cerr<< "time for compression   "<<double(clock()-a)/1000 <<std::endl;
}

void PCL::_detect_change(std::vector<int>& color) {
	if (pcd_type_ == PCL_XYZ) {
		if (cloud_xyz_->size() > 0 && cloud_previous_xyz_ != nullptr) {
			float resolution = 0.02;
			pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_xyz_(resolution);
			std::vector<int> newPointIdxVector;
			////////////////////    appeared
			octree_xyz_.setInputCloud(cloud_previous_xyz_);
			octree_xyz_.addPointsFromInputCloud();
			octree_xyz_.switchBuffers();
			octree_xyz_.setInputCloud(cloud_xyz_);
			octree_xyz_.addPointsFromInputCloud();
			newPointIdxVector.clear();
			octree_xyz_.getPointIndicesFromNewVoxels(newPointIdxVector);
			appeared_points_.clear();
			for (std::size_t i = 0; i < newPointIdxVector.size(); ++i) {
				color[3 * newPointIdxVector[i] + 0] = 0;
				color[3 * newPointIdxVector[i] + 1] = 255 - color[3 * newPointIdxVector[i] + 1];
				color[3 * newPointIdxVector[i] + 2] = 255 - color[3 * newPointIdxVector[i] + 2];
				appeared_points_.push_back((*cloud_xyz_)[newPointIdxVector[i]].x);
				appeared_points_.push_back((*cloud_xyz_)[newPointIdxVector[i]].y);
				appeared_points_.push_back((*cloud_xyz_)[newPointIdxVector[i]].z);
			}
			////////////////////  disappeared
			octree_xyz_.setInputCloud(cloud_xyz_);
			octree_xyz_.addPointsFromInputCloud();
			octree_xyz_.switchBuffers();
			// Add points from cloudB to octree_xyz_
			octree_xyz_.setInputCloud(cloud_previous_xyz_);
			octree_xyz_.addPointsFromInputCloud();
			newPointIdxVector.clear();
			// Get vector of point indices from octree_xyz_ voxels which did not exist in previous buffer
			octree_xyz_.getPointIndicesFromNewVoxels(newPointIdxVector);
			disappeared_points_.clear();
			for (std::size_t i = 0; i < newPointIdxVector.size(); ++i) {
				//color[3 * newPointIdxVector[i] + 0] = 255 - color[3 * newPointIdxVector[i] + 0];
				//color[3 * newPointIdxVector[i] + 1] = 255 - color[3 * newPointIdxVector[i] + 1];
				//color[3 * newPointIdxVector[i] + 2] = 255 - color[3 * newPointIdxVector[i] + 2];
			}
		}
		cloud_previous_xyz_ = cloud_xyz_;
	}
}

void PCL::_remove_plane() {
	bool loop = true;
	while (loop) {
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.002);

		seg.setInputCloud(cloud_xyzrgba_);
		seg.segment(*inliers, *coefficients);

		std::cerr << inliers->indices.size() << std::endl;
		if(inliers->indices.size() > 0.3*cloud_xyzrgba_->size()) {
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyz_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
			pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
			extract.setInputCloud(cloud_xyzrgba_);
			extract.setIndices(inliers);
			extract.setNegative(true);
			extract.filter(*cloud_xyz_filtered);
			cloud_xyzrgba_->resize(0);
			cloud_xyzrgba_ = cloud_xyz_filtered;

			loop = true;
		}
		else {
			loop = false;

		}
	}
}

void PCL::_detect_contact_with_particlefilter() {
	if ((enable_ & (1 << PCL_ENABLE_PARTICLEFILTER)) == 0) return;

	if (cloud_xyzrgba_->size() == 0) return;

	if (!particle_filter_initialized_) {

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
			default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
			for (int i = 0; i < 3; i++) default_step_covariance[i] *= 0.3;
			for (int i = 3; i < 6; i++) default_step_covariance[i] *= 40.0;
			initial_noise_covariance = std::vector<double>(6, 0.0001); // 1 * 6 vector
			//std::vector<double> default_initial_mean = std::vector<double>{ -0.15 * 1., -0.35 * 1., 0.22 * 1., 0, 0, 0. };// 1 * 6 vector xyzrpy
			//std::vector<double> default_initial_mean = std::vector<double>{ -0.16 * 1., -0.26 * 1., 0.22 * 0., 0., 0., 0. };// 1 * 6 vector xyzrpy
			default_initial_mean = std::vector<double>{ -0.15 * 1., -0.30 * 1., 0.27 * 1., 0, 0, 0. };// 1 * 6 vector xyzrpy best for MSRobot
			MaximumParticleNum = 500;
			Delta = 0.99;
			Epsilon = 0.15;
			IterationNum = 2;
			ParticleNum = 100;//300;
			ResampleLikelihoodThr = 0.0;



			//Setup coherence object for tracking
			////////////////////pcl::tracking::NearestPairPointCloudCoherence
			hsv_coherence->setHWeight(0.2);
			hsv_coherence->setSWeight(1.0);
			hsv_coherence->setVWeight(1.0);
			//distance_coherence->setWeight(0.005);


			coherence->addPointCoherence(hsv_coherence);
			//coherence->addPointCoherence(nearest_pair_coherence);
			//coherence->addPointCoherence(distance_coherence);
			coherence->setSearchMethod(search);
			coherence->setMaximumDistance(0.01);
		}
		else {//MSRobot
			default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
			for (int i = 0; i < 3; i++) default_step_covariance[i] *= 0.1;
			for (int i = 3; i < 6; i++) default_step_covariance[i] *= 40.0;
			initial_noise_covariance = std::vector<double>(6, 0.00001); // 1 * 6 vector
			//std::vector<double> default_initial_mean = std::vector<double>{ -0.15 * 1., -0.35 * 1., 0.22 * 1., 0, 0, 0. };// 1 * 6 vector xyzrpy
			//std::vector<double> default_initial_mean = std::vector<double>{ -0.16 * 1., -0.26 * 1., 0.22 * 0., 0., 0., 0. };// 1 * 6 vector xyzrpy
			default_initial_mean = std::vector<double>{ -0.15 * 1., -0.30 * 1., 0.27 * 1., 0, 0, 0. };// 1 * 6 vector xyzrpy best for MSRobot
			MaximumParticleNum = 1000;
			Delta = 0.99;
			Epsilon = 0.15;
			IterationNum = 3;
			ParticleNum = 100;//300;
			ResampleLikelihoodThr = 0.0;



			//Setup coherence object for tracking
			////////////////////pcl::tracking::NearestPairPointCloudCoherence
			hsv_coherence->setHWeight(1.0);
			hsv_coherence->setSWeight(0.2);
			hsv_coherence->setVWeight(0.2);
			//distance_coherence->setWeight(0.005);


			coherence->addPointCoherence(hsv_coherence);
			//coherence->addPointCoherence(nearest_pair_coherence);
			//coherence->addPointCoherence(distance_coherence);
			coherence->setSearchMethod(search);
			coherence->setMaximumDistance(0.01);
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


		trans = contact_detector_->init_for_particlefilter();
		tracker_->setTrans(trans);
		tracker_->setReferenceCloud(contact_detector_->get_cloud_target());


		particle_filter_initialized_ = true;
		//１ループ目は初期化のみで関数抜ける
		return;
	}

	//std::lock_guard<std::mutex> lock(mtx_);
	cloud_downsampled_.reset(new Cloud);
	gridSampleApprox(cloud_xyzrgba_, *cloud_downsampled_, downsampling_grid_size_);
	cloud_xyzrgba_ = cloud_downsampled_;


	//Track the object
	tracker_->setInputCloud(cloud_xyzrgba_);
	tracker_->compute();
	new_cloud_ = true;


	ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
	//std::cerr << tracker_->getParticles() << std::endl;
	if (tracker_->getParticles() && new_cloud_ || false)
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
		transformation = tracker_->toEigenMatrix(result);
		 
		contact_detector_->transform_init(transformation);

		contact_detector_->remove_and_detect(cloud_xyzrgba_);

		//std::cerr << (*particles)[0].weight << std::endl;

		for (const auto& particle : *particles)
		{
			pcl::PointXYZRGBA point;
			point.x = particle.x;
			point.y = particle.y;
			point.z = particle.z;
			//point.r = 255;
			//point.g = 255;
			point.r = (std::min)(255, (int)(100000 * particle.weight));
			point.g = (std::min)(255, (int)(100000 * particle.weight));
			cloud_xyzrgba_->push_back(point);
		}

	}

	

	new_cloud_ = false;
}

double PCL::_computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr& cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointType> tree;
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

void PCL::_recognite() {
	show_keypoints_=(false);
	show_correspondences_=(true);
	use_cloud_resolution_=(false);
	use_hough_=(false);


	model = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
	model_keypoints = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
	scene = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
	scene_keypoints = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
	model_normals = pcl::PointCloud<NormalType>::Ptr(new pcl::PointCloud<NormalType>());
	scene_normals = pcl::PointCloud<NormalType>::Ptr(new pcl::PointCloud<NormalType>());
	model_descriptors = pcl::PointCloud<DescriptorType>::Ptr(new pcl::PointCloud<DescriptorType>());
	scene_descriptors = pcl::PointCloud<DescriptorType>::Ptr(new pcl::PointCloud<DescriptorType>());

	if (cloud_xyzrgba_->size() < 10) return;

	//if (pcl::io::loadPCDFile("src/pointpkg/pcd/MS_xyzrgba_common/tool_target.pcd", *model) < 0) {
	if (pcl::io::loadPCDFile("src/pointpkg/pcd/MS_xyzrgba_common/model3.pcd", *model) < 0) {
			std::cerr << "Error loading model cloud." << std::endl;
	}
	*scene = *cloud_xyzrgba_;
	model_ss_ = (0.005f);
	scene_ss_ = (0.005f);
	rf_rad_ = (0.01f);
	descr_rad_ = (0.015f);
	cg_size_ = (0.01f);
	cg_thresh_ = (0.5f);


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

	//*scene = *cloud_xyzrgba_;
	std::cerr << scene->size() << "   aaa" << std::endl;


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

	
	//  Compute Normals
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch(100);
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);

	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);


	

//  Downsample Clouds to Extract keypoints


	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(model);
	uniform_sampling.setRadiusSearch(model_ss_);
	uniform_sampling.filter(*model_keypoints);
	std::cerr << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

	uniform_sampling.setInputCloud(scene);
	uniform_sampling.setRadiusSearch(scene_ss_);
	uniform_sampling.filter(*scene_keypoints);
	std::cerr << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

	//return;
	//
//  Compute Descriptor for keypoints
//
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
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

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
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
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
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
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
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
	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	viewer.addPointCloud(scene, "scene_cloud");

	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

	if (show_correspondences_ || show_keypoints_)
	{
		//  We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
		viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}

	if (show_keypoints_)
	{
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
		viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
	}
	// for 0 loop
	for (std::size_t i = 0; i < rototranslations.size(); ++i)
	{
		float r00 = abs(rototranslations[i].block<3, 3>(0, 0)(0, 0)-1.00);
		float eps = 0.0001;
		if (r00 < eps) continue;
		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

		if (show_correspondences_)
		{
			for (std::size_t j = 0; j < clustered_corrs[i].size(); ++j)
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;
				PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
				PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
			}
		}
	}

	return;

	while (!viewer.wasStopped())
	{
		//std::cerr << "viewer loop" << std::endl;
		viewer.spinOnce();
	}
}

//Filter along a specified dimension
void PCL::_filterPassThrough(){
	if ((enable_ & (1 << PCL_ENABLE_PASSTHROUGH)) == 0) return;
	if (cloud_xyzrgba_->size() == 0) return;
	if (!enable_filterPathThrough) return;
	//PL(0)
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>());
	//pcl::PassThrough<pcl::PointXYZRGBA> passX;
	////passX.setKeepOrganized(false);
	//passX.setInputCloud(cloud_xyzrgba_);
	//passX.setFilterFieldName("x");
	//passX.setFilterLimits(threshold_[0], threshold_[1]);
	//passX.filter(*cloud_tmp);
	//PL(1);

	////cloud_xyzrgba_.reset();
	//pcl::PassThrough<pcl::PointXYZRGBA> passY;
	//passY.setInputCloud(cloud_tmp);
	//passY.setFilterFieldName("y");
	//passY.setFilterLimits(threshold_[2], threshold_[3]);
	//passY.filter(*cloud_xyzrgba_);
	//PL(2);

	//
	//pcl::PassThrough<pcl::PointXYZRGBA> passZ;
	//passZ.setInputCloud(cloud_xyzrgba_);
	//passZ.setFilterFieldName("z");
	//passZ.setFilterLimits(threshold_[4], threshold_[5]);
	//passZ.filter(*cloud_tmp);
	//PL(3); 

	//*cloud_xyzrgba_ = *cloud_tmp;



	pcl::PointCloud<pcl::PointXYZRGBA>* cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>());
	for (auto p : *cloud_xyzrgba_) {
		if (threshold_[0] <= p.x && p.x <= threshold_[1] && threshold_[2] <= p.y && p.y <= threshold_[3] && threshold_[4] <= p.z && p.z <= threshold_[5])	cloud_tmp->push_back(p);
	}
	cloud_xyzrgba_->clear();
	for (auto p : *cloud_tmp) cloud_xyzrgba_->push_back(p);
	delete cloud_tmp;



	//PL(cloud_xyzrgba_->size());  

}

void PCL::gridSampleApprox(const CloudConstPtr& cloud, Cloud& result, double leaf_size)
{
	//pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
	grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
	grid.setInputCloud(cloud);
	grid.filter(result);
}

//Draw the current particles
bool PCL::drawParticles(pcl::visualization::PCLVisualizer& viz)
{
	ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
	if (particles && new_cloud_)
	{
		//Set pointCloud with particle's points
		pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		for (const auto& particle : *particles)
		{
			pcl::PointXYZ point;

			point.x = particle.x;
			point.y = particle.y;
			point.z = particle.z;
			particle_cloud->push_back(point);
		}

		//Draw red particles 
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(particle_cloud, 250, 99, 71);

			if (!viz.updatePointCloud(particle_cloud, red_color, "particle cloud"))
				viz.addPointCloud(particle_cloud, red_color, "particle cloud");
		}
		return true;
	}
	else
	{
		return false;
	}
}

//cloud_xyzrgbaを描画．要コンストラクタの初期化
void PCL::drawResult()
{
	viewer->showCloud(cloud_xyzrgba_);
	

}

//適当に点群を編集するとき用
void PCL::_edit() {
	//0.0405
	//0.0825
	std::cerr << "edit" << std::endl;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
	for (const auto& p : *cloud_xyzrgba_) {
		if (p.y < 0.) tmp->push_back(p);
		ZZ = std::max(ZZ, p.z);
	}
	cloud_xyzrgba_ = tmp;
	std::cerr << ZZ << std::endl;
}

//不使用
void PCL::_noise_filter() {

}

//不使用
void PCL::_RadiusSearch() {
	/*
	pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;

	kdtree.setInputCloud(cloud_xyzrgba_);

	pcl::PointXYZRGBA searchPoint;
	//searchPoint

	if(_RadiusSearch_is_first){
	target_cloud.reset(new Cloud());
	if (pcl::io::loadPCDFile(target, *target_cloud) == -1) {
		std::cerr << "pcd file not found" << std::endl;
		exit(-1);
	}
	*/
}

//cloud_xyzrgbaを姿勢変換
void PCL::_transform() {
	if ((enable_ & (1 << PCL_ENABLE_TRANSFORM)) == 0) return;
	if (cloud_xyzrgba_->size() == 0) return;

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << transformation_parameters_[0], transformation_parameters_[1], transformation_parameters_[2];
	//transform.rotate(Eigen::AngleAxisf(transformation_parameters_[3], Eigen::Vector3f::UnitX()));
	//transform.rotate(Eigen::AngleAxisf(transformation_parameters_[4], Eigen::Vector3f::UnitY()));
	//transform.rotate(Eigen::AngleAxisf(transformation_parameters_[5], Eigen::Vector3f::UnitZ()));

	Eigen::Affine3f rotX = Eigen::Affine3f::Identity();
	rotX.rotate(Eigen::AngleAxisf(transformation_parameters_[3], Eigen::Vector3f::UnitX()));

	Eigen::Affine3f rotY = Eigen::Affine3f::Identity();
	rotY.rotate(Eigen::AngleAxisf(transformation_parameters_[4], Eigen::Vector3f::UnitY()));

	Eigen::Affine3f rotZ = Eigen::Affine3f::Identity();
	rotZ.rotate(Eigen::AngleAxisf(transformation_parameters_[5], Eigen::Vector3f::UnitZ()));

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>());

	pcl::transformPointCloud(*cloud_xyzrgba_, *cloud_tmp, rotX);
	pcl::transformPointCloud(*cloud_tmp, *cloud_xyzrgba_, rotY);
	pcl::transformPointCloud(*cloud_xyzrgba_, *cloud_tmp, rotZ);
	pcl::transformPointCloud(*cloud_tmp, *cloud_xyzrgba_, transform);

}


//CoppeliaSimのロボットモデルの姿勢を取得し，接触判定する
void PCL::_detect_contact_with_coppeliasim(){
	if ((enable_ & (1 << PCL_ENABLE_DETECT_WITH_SIM)) == 0) return;

	if (cloud_xyzrgba_->size() == 0) return;
	////pcd_target, pcd_mask, pcd_tipを絶対座標の原点に戻す
	std::vector<float> RemTP = coppeliasim_interface_->getRemoverTipPose();
	contact_detector_->transform_init(&RemTP[0], &RemTP[3], true);
	//coppeliasim内のロボットモデルと位置姿勢を一致させる．
	std::vector<float> LeftTP = coppeliasim_interface_->getLeftTipPose();
	contact_detector_->transform_rotated(&LeftTP[0], &LeftTP[3], false);

	contact_detector_->remove_and_detect(cloud_xyzrgba_);
}



//folder内のpcdをcloud_xyzrgbaに追加
void PCL::_read_pcd_file(std::string folder, int& number_of_points, std::vector<int>& color) {
	if ((enable_ & (1 << PCL_ENABLE_READ_PCD)) == 0) return;
	if (folder == "") return;
	std::string file;
	if (folder.find(".pcd") != std::string::npos) file = folder;
	else {
		file = folder + std::to_string(cnt_read_);
		//file += std::to_string(cnt_read_);
		file += ".pcd";
		//std::cerr << "qwert" << std::endl;
	}

	if (pcd_type_ == PCL_XYZ) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud_tmp) == -1) //* load the file
		{
			//PCL_ERROR("Couldn't read file test_pcd.pcd \n");
			cnt_read_ = 0;
		}
		else {
			cloud_xyz_ = cloud_tmp;
			number_of_points = cloud_xyz_->width * cloud_xyz_->height;
			cnt_read_++;
			color_.clear();
			for (int i = 0; i < cloud_xyz_->width * cloud_xyz_->height; i++) {
				color_.push_back(255);
				color_.push_back(0);
				color_.push_back(0);
			}
			color = color_;
		}
	}
	else if(pcd_type_ == PCL_XYZRGB) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);

		if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file, *cloud_tmp) == -1) //* load the file
		{
			//PCL_ERROR("Couldn't read file test_pcd.pcd \n");
			cnt_read_ = 0;
		}
		else {
			cloud_xyzrgb_ = cloud_tmp;
			number_of_points = cloud_xyzrgb_->width * cloud_xyzrgb_->height;
			cnt_read_++;

			color = color_;
		}
	}
	else if (pcd_type_ == PCL_XYZRGBA) {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
		//cloud_xyzrgba_->clear();
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file, *cloud_tmp) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file test_pcd.pcd \n");
			EL(file)
			cnt_read_ = 0;
		}
		else {
			
			for (const auto& p : *cloud_tmp) {	cloud_xyzrgba_->push_back(p);}
			//cloud_xyzrgba_ = cloud_tmp;
			number_of_points = cloud_xyzrgba_->width * cloud_xyzrgba_->height;
			cnt_read_++;
			//std::cerr << "read" << endl;
		}
	}


}

//cloud_xyzrgbaをpcdとしてfolder内に保存
void PCL::_write_pcd_file(std::string folder) {
	if ((enable_ & (1 << PCL_ENABLE_WRITE_PCD)) == 0) return;

	if (folder.back() != '/') folder += '/';

	if (cloud_xyzrgba_->size() == 0) return;
	
	if (pcd_type_ == PCL_XYZ && cloud_xyz_->size() > 0) {
		pcl::io::savePCDFileASCII(folder + std::to_string(cnt_write_) + ".pcd", *cloud_xyz_);
		cnt_write_++;
	}
	if (pcd_type_ == PCL_XYZRGB && cloud_xyzrgb_->size() > 0) {
		pcl::io::savePCDFileASCII(folder + std::to_string(cnt_write_) + ".pcd", *cloud_xyzrgb_);
		cnt_write_++;
	}
	if (pcd_type_ == PCL_XYZRGBA && cloud_xyzrgba_->size() > 0) {
		pcl::io::savePCDFileASCII(folder + std::to_string(cnt_write_) + ".pcd", *cloud_xyzrgba_);
		cnt_write_++;
	}
	//std::cerr << "Saved " << cloud_xyz_->size() << " data points to test_pcd.pcd." << std::endl;
}

//txtからパラメータ読み込み．ほとんどtransform用
void PCL::_load_parameters(std::string file_path) {

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

		if ( parameter_name=="trans_x")   transformation_parameters_[0] = stof(value);
		if ( parameter_name=="trans_y")   transformation_parameters_[1] = stof(value);
		if ( parameter_name=="trans_z")   transformation_parameters_[2] = stof(value);
		if ( parameter_name=="trans_ax")   transformation_parameters_[3] = stof(value);
		if ( parameter_name=="trans_ay")   transformation_parameters_[4] = stof(value);
		if ( parameter_name=="trans_az")   transformation_parameters_[5] = stof(value);

		if ( parameter_name=="thresh_min_x")   threshold_[0] = stof(value);
		if ( parameter_name=="thresh_max_x")   threshold_[1] = stof(value);
		if ( parameter_name=="thresh_min_y")   threshold_[2] = stof(value);
		if ( parameter_name=="thresh_max_y")   threshold_[3] = stof(value);
		if ( parameter_name=="thresh_min_z")   threshold_[4] = stof(value);
		if ( parameter_name=="thresh_max_z")   threshold_[5] = stof(value);

	}

	std::cerr << "pcl; parameters loaded" << std::endl;

	params_read_ = true;


}

//txtにパラメータ保存．ほとんどtransform用
void PCL::_save_parameters(std::string file_path) {
	if (!params_read_) return;

	ofstream ofs(file_path);
	ofs << std::fixed<<std::setprecision(6);


	for (int i = 0; i < 6; i++) if (abs(transformation_parameters_[i]) < 0.00001) transformation_parameters_[i] = 0.0;
	ofs << "trans_x "<<transformation_parameters_[0]<<endl;
	ofs << "trans_y "<<transformation_parameters_[1]<<endl;
	ofs << "trans_z "<<transformation_parameters_[2]<<endl;
	ofs << "trans_ax "<<transformation_parameters_[3]<<endl;
	ofs << "trans_ay "<<transformation_parameters_[4]<<endl;
	ofs << "trans_az "<<transformation_parameters_[5]<<endl;

	for (int i = 0; i < 6; i++) if (abs(threshold_[i]) < 0.00001) threshold_[i] = 0.0;
	ofs << "thresh_min_x "<<threshold_[0]<<endl;
	ofs << "thresh_max_x "<<threshold_[1]<<endl;
	ofs << "thresh_min_y "<<threshold_[2]<<endl;
	ofs << "thresh_max_y "<<threshold_[3]<<endl;
	ofs << "thresh_min_z "<<threshold_[4]<<endl;
	ofs << "thresh_max_z "<<threshold_[5]<<endl;
	ofs.close();

}






// arg:トラッキングしたい対象の点群，　削除用点群，　トラッキングしたい対象の点群の先端位置（1点のみ）
ContactDetector::ContactDetector(std::string pcd_target, std::string pcd_mask="", std::string pcd_tip=""):
	octree(pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA>(0.005))
	, cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>())
	, inliers(new pcl::PointIndices())
{
	//重ければ点群密度を下げる
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
	//grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
	//grid.setInputCloud(cloud);
	//grid.filter(result);

	//３つの点群を読み込む
	cloud_target_init_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcl::io::loadPCDFile(pcd_target, *cloud_target_init_) == -1) {
		std::cerr << "remover::remover; pcd target file not found" << std::endl;
		exit(-1);
	}
	cloud_mask_init_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcd_mask!="" && pcl::io::loadPCDFile(pcd_mask, *cloud_mask_init_) == -1) {
		std::cerr << "remover::remover; pcd mask file not found" << std::endl;
		exit(-1);
	}
	cloud_tip_init_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcd_tip!="" && pcl::io::loadPCDFile(pcd_tip, *cloud_tip_init_) == -1) {
		std::cerr << "remover::remover; pcd tip file not found" << std::endl;
		exit(-1);
	}
	std::cerr << "ContactDetector::ContactDetector succsessfully read pcd" << std::endl;
	cloud_target_transformed_ = cloud_target_init_;
	cloud_mask_transformed_ = cloud_mask_init_;
	cloud_tip_transformed_ = cloud_tip_init_;
	//gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
}

//読み込んだデータを姿勢変換する  init->transformed
void ContactDetector::transform_init(float* pos, float* ori, bool inverse) {
	//std::cerr << "ContactDetector::transform_init(): " << " pos-z " << pos[2] << std::endl;
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << pos[0], pos[1], pos[2];
	transform.rotate(Eigen::AngleAxisf(ori[0], Eigen::Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(ori[1], Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(ori[2], Eigen::Vector3f::UnitZ()));

	if (inverse == true) transform = transform.inverse();

	cloud_target_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_target_init_, *cloud_target_transformed_, transform);
	cloud_mask_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_mask_init_, *cloud_mask_transformed_, transform);
	cloud_tip_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_tip_init_, *cloud_tip_transformed_, transform);
}

//読み込んだデータを姿勢変換する  init->transformed
void ContactDetector::transform_init(const Eigen::Affine3f& transform) {
	//PL("aaa")
	//PL(cloud_target_init_->size())
	cloud_target_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_target_init_, *cloud_target_transformed_, transform);
	cloud_mask_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_mask_init_, *cloud_mask_transformed_, transform);
	cloud_tip_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_tip_init_, *cloud_tip_transformed_, transform);
}

//姿勢変換済みのデータをさらに姿勢変換する  transformed->transformed
void ContactDetector::transform_rotated(float* pos, float* ori, bool inverse) {
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

	//pcl::PointCloud<pcl::PointXYZRGBA> cloud_tmp;
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>());
	cloud_tmp = cloud_target_transformed_;
	pcl::transformPointCloud(*cloud_tmp, *cloud_target_transformed_, transform);
	cloud_tmp = cloud_mask_transformed_;
	pcl::transformPointCloud(*cloud_tmp, *cloud_mask_transformed_, transform);
	cloud_tmp = cloud_tip_transformed_;
	pcl::transformPointCloud(*cloud_tmp, *cloud_tip_transformed_, transform);
}

//cloud_xyzrgbaからpcd_maskを取り除き，pcd_tip周辺の点数を数える
void ContactDetector::remove_and_detect(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_xyzrgba) {
	//return;

	//cloud_ と resultの一致検出
	//pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA> octree(0.005);
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
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr consensus(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr different(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
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
		pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
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
Eigen::Affine3f ContactDetector::init_for_particlefilter() {
	Eigen::Vector4f c;
	pcl::compute3DCentroid<pcl::PointXYZRGBA>(*cloud_target_init_, c);
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);

	cloud_target_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_target_init_, *cloud_target_transformed_, trans.inverse());
	//gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
	*cloud_target_init_ = *cloud_target_transformed_;

	cloud_mask_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_mask_init_, *cloud_mask_transformed_, trans.inverse());
	*cloud_mask_init_ = *cloud_mask_transformed_;

	cloud_tip_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_tip_init_, *cloud_tip_transformed_, trans.inverse());
	*cloud_tip_init_ = *cloud_tip_transformed_;

	return  trans;
}