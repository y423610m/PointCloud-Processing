//PCL
#pragma once

#ifndef PCL_XYZ
#define PCL_XYZ 0
#endif 

#ifndef PCL_XYZRGB
#define PCL_XYZRGB 1
#endif 

#ifndef PCL_XYZRGBA
#define PCL_XYZRGBA 2
#endif 

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

#include<params.h>

//target
#include <coppeliasim_interface.h>
class ContactDetector {
private:
	// not transformed pointcloud .'. roe pcd data
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_target_init_;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_target_transformed_;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_mask_init_;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_mask_transformed_;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tip_init_;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tip_transformed_;



public:
	ContactDetector(std::string pcd_target, std::string pcd_mask, std::string pcd_tip);

	void transform_init(float* pos, float* ori, bool inverse=false);
	void transform_init(const Eigen::Affine3f& T);
	void transform_rotated(float* pos, float* ori, bool inverse=false);
	void remove_and_detect(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_xyzrgba);

	Eigen::Affine3f init_for_particlefilter();


	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_cloud_target() { return cloud_target_transformed_; }
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_cloud_mask() { return cloud_mask_transformed_; }
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_cloud_tip() { return cloud_tip_transformed_; }
};

using namespace pcl::tracking;
using namespace std::chrono_literals;

class PCL {
private:
	//int counter = 0;//not used anymore
	//float now;

	CoppeliaSimInterface* coppeliasim_interface_;

	//pcdの種類. XYZ, XYZRGB, XYZRGBA
	int pcd_type_ = PCL_XYZ;
	//点群格納用データ（vector）
	std::vector<float> points_;
	std::vector<int> color_;
	//点群データ（pcd）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyzrgba_;
	

	//detect changes()用
	//2つのポイントクラウドを比較したときのデータ
	std::vector<float> appeared_points_;
	std::vector<float> disappeared_points_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous_xyz_;

	//track()用
	bool particle_filter_initialized_ = false;
	typedef pcl::PointXYZRGBA RefPointType;
	typedef ParticleXYZRPY ParticleT;
	typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
	typedef Cloud::Ptr CloudPtr;
	typedef Cloud::ConstPtr CloudConstPtr;
	typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
	std::mutex mtx_;
	ParticleFilter::Ptr tracker_;
	CloudPtr transed_ref;
	CloudPtr transed_ref_downsampled;
	CloudPtr cloud_downsampled_;
	CloudPtr target_cloud;
	bool new_cloud_ = false;
	KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>::Ptr tracker;
	ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence;
	DistanceCoherence<RefPointType>::Ptr distance_coherence;
	pcl::search::Octree<RefPointType>::Ptr search;
	HSVColorCoherence<RefPointType>::Ptr hsv_coherence;
	NearestPairPointCloudCoherence<RefPointType>::Ptr nearest_pair_coherence;
	void gridSampleApprox(const CloudConstPtr& cloud, Cloud& result, double leaf_size);
	bool drawParticles(pcl::visualization::PCLVisualizer& viz);
	void drawResult();

	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	double downsampling_grid_size_ = 0.002;
	Eigen::Affine3f transformation;

	//除去
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_target_remover_;
	CloudPtr cloud_target_remover_ref_;
	CloudPtr cloud_target_remover_ref_downsampled_;
	//R探索
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Rcenter_;
	CloudPtr Rcenter_ref_;



	//recognite 用
	typedef pcl::PointXYZRGBA PointType;
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
	pcl::PointCloud<PointType>::Ptr model;
	pcl::PointCloud<PointType>::Ptr model_keypoints;
	pcl::PointCloud<PointType>::Ptr scene;
	pcl::PointCloud<PointType>::Ptr scene_keypoints;
	pcl::PointCloud<NormalType>::Ptr model_normals;
	pcl::PointCloud<NormalType>::Ptr scene_normals;
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors;
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors;
	double _computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr& cloud);

	//_filter_path_through()
	//pcl::PassThrough<pcl::PointXYZRGBA> pass;
	bool enable_filterPathThrough = true;
	double threshold_[6] = { -0.3, 0.0, -0.5, 0.0, 0.0, 0.3 };

	//_RadiusSearch()
	bool _RadiusSearch_is_first = true;

	//_edit用
	float tx = -0.08;
	float ty = -0.13;
	float tz = 0.15;

	float ZZ = -2.0;;


	//read(), write()用
	bool params_read_ = false;
	int cnt_write_ = 0;
	int cnt_read_ = 0;
	//pcd folder
	std::string filepath = "src/pointpkg/pcd/MS_xyzrgba/";
	std::string filename = "";

	//transform用
	double transformation_parameters_[6];//x,y,z,anglex,y,z

	//dwaw 用
	pcl::visualization::CloudViewer* viewer;

	//target()用
	std::unique_ptr<ContactDetector> contact_detector_;

	
	void _convert_array_to_pcd(int& number_of_points, std::vector<float>& points, std::vector<int>& color);
	void _convert_pcd_to_array(int& number_of_points, std::vector<float>& points, std::vector<int>& color);

	void _compress();
	void _detect_change(std::vector<int>& color);
	void _remove_plane();
	void _detect_contact_with_particlefilter();
	void _detect_contact_with_coppeliasim();
	void _recognite();
	void _filterPassThrough();
	void _noise_filter();
	void _RadiusSearch();
	void _transform();

	void _edit();

	void _read_pcd_file(std::string folder, int& number_of_points, std::vector<int>& color);
	void _write_pcd_file(std::string folder);
	void _load_parameters(std::string file_path);
	void _save_parameters(std::string file_path);

	void _get_tip_pose();

public:
	PCL(int option_pcd_type);
	~PCL();
	void init(CoppeliaSimInterface* coppeliasim_interface);
	void update(int& number_of_points, std::vector<float>& points, std::vector<int>& color);
	//void update_RGB(int& number_of_points, float** points);

	int get_points_size() { return points_.size() / 3; }
	float* get_points() { return &points_[0]; }
	int get_appeared_points_size() { return appeared_points_.size() / 3; }
	float* get_appeared_points() { return &appeared_points_[0]; }
	int get_disappeared_points_size() { return disappeared_points_.size() / 3; }
	float* get_disappeared_points() { return &disappeared_points_[0]; }
	double* get_transformation_parameters(int n) { return &transformation_parameters_[n]; }
	bool* get_enable_filterPassThrough() { return &enable_filterPathThrough; }
	double* get_filterPassThrough_threshold(int i) { return &threshold_[i]; }
};


