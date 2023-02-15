

#include <iostream>
#include <windows.h>
#include <numeric>
#include <cmath>
#include <string>
#include <fstream>
#include <algorithm>
#include <queue>

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
		showContactPoints_ = ROSParam::getIntParam("CD_ShowContactPoints");
		searchMode_ = ROSParam::getIntParam("CD_SearchMode");
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
		if (ROSParam::getIntParam("CD_KeepTip")) {//マスクの先端をなくす
			PL("keep tip!!!!")
			Cloud& cloud = *cloud_mask_init_;
			for (int i = 0; i < cloud.size(); i++) {
				if (cloud[i].y > -0.025) {
					swap(cloud[i], cloud.back());
					cloud.erase(cloud.end()-1);
					i--;
				}
			}
		}

		*cloud_mask_transformed_ = *cloud_mask_init_;

		if (pcd_tip != "" && pcl::io::loadPCDFile(pcd_tip, *cloud_tip_init_) == -1) {
			std::cerr << "remover::remover; pcd tip file not found" << std::endl;
			exit(-1);
		}
		cloud_tip_init_->emplace_back(PointTypeT());
		cloud_tip_init_->back().y = -0.02;
		*cloud_tip_transformed_ = *cloud_tip_init_;

		sort(cloud_target_init_->begin(), cloud_target_init_->end(), [](auto& a, auto& b) {return a.y > b.y; });

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
	pcl::KdTreeFLANN<PointTypeT> kdtree_;
	std::vector<int> pointIdxRadiusSearch_;
	std::vector<float> pointRadiusSquareDistance_;
	CloudPtr consensus_;
	CloudPtr different_;
	bool showConsensus_ = true;
	bool showTargetTool_ = true;
	bool showContactPoints_ = false;
	int loopCnt = 0;
	//queue<int> que_;
	using P = pair<double, int>;
	priority_queue<P, vector<P>, greater<P>> que_;//Tに近い順
	vector<int> idx_;
	multimap<double, int, greater<double>> st;//
	//int sum = 0;
	int searchMode_ = 0;
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


		//pcl::ExtractIndices<PointTypeT> extract;
		////extract.setKeepOrganized(true);
		//extract.setInputCloud(cloud);
		//extract.setIndices(inliers);
		//extract.setNegative(false);
		//extract.filter(*different_);
		//extract.setNegative(true);//red
		//extract.filter(*consensus_);


		//新判定法用にコメントアウト
		//*cloud = *different_;//!!!!!!!!!!!!!!!!!!



		kdtree_.setInputCloud(cloud);



		//ツール先端近傍点数を数える
		PointTypeT searchPoint = (*cloud_tip_transformed_)[0];
		searchPoint.r = 255;
		searchPoint.g = 255;
		searchPoint.b = 255;
		loopCnt++;


		//ポイント密度の低いツールの根元側から探索始める
		PointTypeT startPoint = (*cloud_tip_transformed_)[1];
		PointTypeT endPoint = (*cloud_tip_transformed_)[0];
		//カメラ側に若干ずらす
		startPoint.y -= 0.006;
		endPoint.y -= 0.000;
		PointTypeT midPoint;
		midPoint.x = (startPoint.x + endPoint.x) / 2;
		midPoint.y = (startPoint.y + endPoint.y) / 2;
		midPoint.z = (startPoint.z + endPoint.z) / 2;

		startPoint.r = 0;
		startPoint.g = 0;
		startPoint.b = 255;
		endPoint.r = 0;
		endPoint.g = 0;
		endPoint.b = 255;


		bool connected = false;

		if (searchMode_ || (20 <= loopCnt && loopCnt < 220 && cloud->size() > 0)) {
			//auto t = clock();
			double checkR = 0.006;
			int checkBoundary = 50;
			if (checkBoundary < kdtree_.radiusSearch(searchPoint, checkR, pointIdxRadiusSearch_, pointRadiusSquareDistance_)) {

				if (pointIdxRadiusSearch_.size()) pointIdxRadiusSearch_.clear();
				if (pointRadiusSquareDistance_.size()) pointRadiusSquareDistance_.clear();

				//double dist = sqrt();

				//int num = 200;

				//for (int i = 0; i < 15; i++) {
				//	double r = 0.001 * (2 + i);
				//	int n = kdtree_.radiusSearch(searchPoint, r, pointIdxRadiusSearch_, pointRadiusSquareDistance_);
				//	PS(r) PL(n);
				//}
				float R = 0.015;
				double r = 0.002;
				if (idx_.size()) idx_.clear();
				int n;
				//n = kdtree_.radiusSearch(startPoint, R, pointIdxRadiusSearch_, pointRadiusSquareDistance_);
				//for (auto id : pointIdxRadiusSearch_) idx_.emplace_back(id);
				//n = kdtree_.radiusSearch(endPoint, R, pointIdxRadiusSearch_, pointRadiusSquareDistance_);
				//for (auto id : pointIdxRadiusSearch_) idx_.emplace_back(id);
				n = kdtree_.radiusSearch(midPoint, R, pointIdxRadiusSearch_, pointRadiusSquareDistance_);
				for (auto id : pointIdxRadiusSearch_) idx_.emplace_back(id);

				pointIdxRadiusSearch_.clear();
				sort(idx_.begin(), idx_.end());
				idx_.erase(unique(idx_.begin(), idx_.end()), idx_.end());
				swap(idx_, pointIdxRadiusSearch_);

				//EL(clock() - t);
				//t = clock();

				/*
				終点側からA*法で経路探索
				*/
				int cntPoint = 0;


				auto dist = [&](const PointTypeT& a, const PointTypeT& b)->double {
					return hypot(a.x - b.x, hypot(a.y - b.y, a.z - b.z));
				};
				double distSM = 0.003;//始点から中間点
				double distMM = 0.003;//中間点から中間点
				double distMT = 0.003;//中間点から終点
				double distST = dist(startPoint, endPoint);

				if (searchMode_==1) {
					//PL("vector search")
					//Sの近く探索
					for (int i = 0; i < pointIdxRadiusSearch_.size(); i++) {
						int id = pointIdxRadiusSearch_[i];
						bool ok = false;
						double distS = dist(startPoint, (*cloud)[id]);
						double distT = dist(endPoint, (*cloud)[id]);
						if (distS < distSM) ok = true;
						if (distT > distST) ok = true;
						if (ok) {
							//que_.push(id);
							que_.emplace(distT, id);
							cntPoint++;
							swap(pointIdxRadiusSearch_[i], pointIdxRadiusSearch_.back());
							pointIdxRadiusSearch_.pop_back();
							i--;
						}
						else if (distS > distST) {//t側に近くて，遠すぎる場合，除去
							swap(pointIdxRadiusSearch_[i], pointIdxRadiusSearch_.back());
							pointIdxRadiusSearch_.pop_back();
							i--;
						}
					}
					//近傍点およびTからの距離を計算
					while (!que_.empty()) {
						if (connected) break;
						//int p = que_.front();
						int p = que_.top().second;
						que_.pop();
						(*cloud)[p].r = 255;
						(*cloud)[p].g = 0;
						(*cloud)[p].b = 0;

						//修論撮影用に強調
						for (double r = 0.0002; r < 0.00021; r += 0.0001) {
							for (double a = 0; a < M_PI * 2; a += 0.02) {
								for (double b = 0; b < M_PI * 2; b += 0.02) {
									PointTypeT point;
									point = (*cloud)[p];
									point.x += r * cos(a) * sin(b);
									point.y += r * sin(a) * sin(b);
									point.z += r * cos(b);
									cloud->emplace_back(point);
								}
							}
						}

						//他の中間点への移動
						for (int i = 0; i < pointIdxRadiusSearch_.size(); i++) {
							int id = pointIdxRadiusSearch_[i];



							if (dist((*cloud)[p], (*cloud)[id]) < distMM) {

								double d = dist(endPoint, (*cloud)[id]);
								//Tに近い？
								if (d < distMT) {
									connected = true;
									break;
								}

								//que_.push(id);
								que_.emplace(d, id);
								cntPoint++;
								swap(pointIdxRadiusSearch_[i], pointIdxRadiusSearch_.back());
								pointIdxRadiusSearch_.pop_back();
								i--;
							}
						}

						if (connected) break;
					}
					while (!que_.empty()) que_.pop();
				}
				else if (searchMode_ == 2) {//multiset探索．遅いので，vector searchにしたほうが良い
					PL("multiset search")
						if (!st.empty()) st.clear();
					for (int i = 0; i < pointIdxRadiusSearch_.size(); i++) {
						int id = pointIdxRadiusSearch_[i];
						bool ok = false;
						double distS = dist(startPoint, (*cloud)[id]);
						double distT = dist(endPoint, (*cloud)[id]);
						if (distS < distSM) ok = true;
						//if (distST < distT) ok = true;

						if (ok) {//始点から到達可能
							//que_.push(id);
							que_.emplace(distT, id);
							//cntPoint++;
							//swap(pointIdxRadiusSearch_[i], pointIdxRadiusSearch_.back());
							//pointIdxRadiusSearch_.pop_back();
							//i--;
						}
						//else if (distS > distST) {//終点を越してしまった場合
						//	//swap(pointIdxRadiusSearch_[i], pointIdxRadiusSearch_.back());
						//	//pointIdxRadiusSearch_.pop_back();
						//	//i--;
						//}
						else {
							st.emplace(distT, id);
						}
					}
					EL(st.size())

						//for (auto it = st.begin(); it != st.end(); it++) {
						//	int p = it->second;
						//	(*cloud)[p].r = 255;
						//	(*cloud)[p].g = 0;
						//	(*cloud)[p].b = 0;
						//}

						//近傍点およびTからの距離を計算
						while (!que_.empty()) {
							if (connected) break;
							double d = que_.top().first;
							int p = que_.top().second;
							que_.pop();
							(*cloud)[p].r = 255;
							(*cloud)[p].g = 0;
							(*cloud)[p].b = 0;

							//他の中間点への移動
							auto it = st.lower_bound(d + distMM);
							while (it != st.end()) {
								if (d - distMM > it->first) break;
								if (it->first < distMT) {//STが連結である時
									connected = true;
									break;
								}
								if (dist((*cloud)[p], (*cloud)[it->second]) < distMM) {
									que_.emplace(it->first, it->second);
									double fi = it->first;
									st.erase(it);
									it = st.lower_bound(fi);
								}
								else {
									it++;
								}
							}
							if (connected) break;
						}
					while (!que_.empty()) que_.pop();
					if (!st.empty()) st.clear();
				}
			}

			if (connected) {
				PL("Touched!!!");
				//sum++;
			}
			else {
				PL("not touched");
				//PL(0);
			}
			//EL(cloud->size());
			//PL(sum)
		}



		//半径内の点
		if (showContactPoints_) for (auto id : pointIdxRadiusSearch_) {
			(*cloud)[id].r = 255;
		}

		////先端位置強調
		cloud->push_back(searchPoint);
		for (double r = 0.0002; r <= 0.0004; r += 0.0001) {
			for (double a = 0; a < M_PI * 2; a += 0.02) {
				for (double b = 0; b < M_PI * 2; b += 0.02) {
					PointTypeT p;
					p = startPoint;
					p.x += r * cos(a) * sin(b);
					p.y += r * sin(a) * sin(b);
					p.z += r * cos(b);
					cloud->emplace_back(p);
					//p = midPoint;
					//p.x += r * cos(a) * sin(b);
					//p.y += r * sin(a) * sin(b);
					//p.z += r * cos(b);
					//cloud->emplace_back(p);
					p = endPoint;
					p.x += r * cos(a) * sin(b);
					p.y += r * sin(a) * sin(b);
					p.z += r * cos(b);
					cloud->emplace_back(p);
				}
			}
		}


		//ターゲットと重なる点
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
		//EL(clock() - t);
		//t = clock();
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

	//pcl::detect_with_yol()内で使う．target内で最もZ座標が低い点を返す
	array<float, 3> get_lowest_point() {
		PointTypeT point = cloud_target_transformed_->back();
		cloud_target_transformed_->erase(prev(cloud_target_transformed_->end()));

		for (auto& p : *cloud_target_transformed_) if (point.z > p.z) swap(point, p);
		
		array<float, 3> res;
		res[0] = point.x;
		res[1] = point.y;
		res[2] = point.z;

		return res;
	}

	void set_tip_position(const array<float, 3>& p) {
		(*cloud_tip_transformed_)[0].x = p[0];
		(*cloud_tip_transformed_)[0].y = p[1];
		(*cloud_tip_transformed_)[0].z = p[2];
	}

	CloudPtr get_cloud_target_init() { return cloud_target_init_; }
	CloudPtr get_cloud_target_transformed() { return cloud_target_transformed_; }
	//CloudPtr get_cloud_mask() { return cloud_mask_transformed_; }
	//CloudPtr get_cloud_tip() { return cloud_tip_transformed_; }

	~ContactDetector2(){
		PL("~CD");
	}
};

