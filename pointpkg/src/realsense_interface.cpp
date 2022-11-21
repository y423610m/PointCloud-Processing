#include "realsense_interface.h"
#include "development_commands.h"
#include "ros_param.h"
#include "array3d_extention.h"

#include <array>
#include <map>
#include <fstream>
#include <cmath>
#include <sstream>
#include <cassert>
//#include "rs_processing.hpp"
//#include "rs_internal.hpp"
#include <iostream>
#include <thread>
#include <chrono>


RealSenseInterface::RealSenseInterface(){
    if (!ROSParam::getIntParam("RS_enable_RS")) return;



    //RealSense configuration
    cfg_.enable_stream(RS2_STREAM_DEPTH, RSDepthWidth_, RSDepthHeight_, RS2_FORMAT_Z16, RSFps_);
    cfg_.enable_stream(RS2_STREAM_COLOR, RSImageWidth_, RSImageHeight_, RS2_FORMAT_RGB8, RSFps_);
    pipe_.start(cfg_);

    //set filters for RealSense Depth stream
    this->_setFilters();

    //construct YOLOv7 
    yolov7_.reset(new YOLODetector(ROSParam::getStringParam("YOLO_ModelPath"), ROSParam::getIntParam("YOLO_enable_GPU"), cv::Size(480, 480), cv::Size(RSImageWidth_, RSImageHeight_)));
    yolov7_->setConfThreshold(ROSParam::getDoubleParam("YOLO_Conf"));
    yolov7_->setIouThreshold(ROSParam::getDoubleParam("YOLO_IoU"));
    classIdImage_ = std::vector<std::vector<int>>(RSImageHeight_, std::vector<int>(RSImageWidth_, -1));
    class3DPositions_.resize(classNum);

    image_ = cv::Mat(RSImageHeight_, RSImageWidth_, CV_8UC3);
    classMeanPos_.resize(classNum);

    ratio_ty_over_yg = ROSParam::getDoubleParam("RS_TYoverYG");
    ratio_tm1_over_m1m2_.resize(2);
    ratio_tm1_over_m1m2_[0] = ROSParam::getDoubleParam("RS_TYoverYG");
    ratio_tm1_over_m1m2_[1] = ROSParam::getDoubleParam("RS_TBoverBP");
    length_tc = ROSParam::getDoubleParam("RS_length_TC");
    ZratioLimit_ = ROSParam::getDoubleParam("RS_ZratioLimit");
    updateLastZ_ = ROSParam::getIntParam("RS_updateLastZ");

    //if true, show Time log on terminal
    showTime_ = ROSParam::getIntParam("RS_ShowTime");
    showImage_ = ROSParam::getIntParam("RS_ShowImage");

    //���C�����C���s
    threshold_ = { 0.5, 0.5, 0.7 };


    //���񏈗��\�肾�������C���Ȃ��ق�������...
    ////process thread loop
    //thread t([&]() {
    //    while (true) {
    //        this->_process();
    //        PL("run")
    //    }
    //    });
    //swap(thread_, t);

    initialized_ = true;
    cerr << "RealSenseInterface constructed" << endl;
    return;

    //auto pro = pipe_.get_active_profile();
    //auto str = pro.get_streams();
    //for (int i = 0; i < str.size(); i++) {
    //    ES(str[i].stream_type());
    //    ES(str[i].format());
    //    ES(str[i].fps());
    //    PL("")
    //}
    //return;

    /*
    https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1config.html
    https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html
    str[i].stream_type(): Depth str[i].format(): Z16 str[i].fps(): 15
    str[i].stream_type(): Color str[i].format(): RGB8 str[i].fps(): 15
    str[i].stream_type(): Gyro str[i].format(): MOTION_XYZ32F str[i].fps(): 200
    str[i].stream_type(): Accel str[i].format(): MOTION_XYZ32F str[i].fps(): 63
    */

}

RealSenseInterface::~RealSenseInterface() {
    pipe_.stop();
}

void RealSenseInterface::_setFilters() {
    //defined in rs_processing.hpp 
    /// sample rs-post-processing.h
    //https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
    //filters
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
    rs2::hole_filling_filter hole_filter;
    hole_filter.set_option(RS2_OPTION_HOLES_FILL, 2); //1:farest 2:nearest

    rs2::disparity_transform depth_to_disparity(true);
    disparity_to_depth = new rs2::disparity_transform(false);
    // The following order of emplacement will dictate the orders in which filters are applied
    if (ROSParam::getIntParam("RS_enable_FILTER_Decimate"))filters_.emplace_back("Decimate", dec_filter);
    //if (ROSParam::getIntParam("RS_enable_RS")) filters_.emplace_back("Threshold", thr_filter);
    if (ROSParam::getIntParam("RS_enable_FILTER_Disparity")) filters_.emplace_back(disparity_filter_name, depth_to_disparity);
    if (ROSParam::getIntParam("RS_enable_FILTER_Spatial")) filters_.emplace_back("Spatial", spat_filter);
    if (ROSParam::getIntParam("RS_enable_FILTER_Temporal")) filters_.emplace_back("Temporal", temp_filter);
    if (ROSParam::getIntParam("RS_enable_FILTER_Hole")) filters_.emplace_back("Hole", hole_filter);
}

//pipe����󂯎��Cfilter�����Ă���queue�ɑ}��. �ʃX���b�h�ŉ񂷗\��
void RealSenseInterface::_process() {
    rs2::frameset frames = pipe_.wait_for_frames();

    rs2::frame depth = frames.get_depth_frame();
    if (!depth) {
        EL("No Depth Frame");
        return;
    }

    color_que_.enqueue(frames.get_color_frame());

    if (filters_.size()) {
        bool revert_disparity = false;
        for (auto& filter : filters_) {
            if (filter.is_enabled) {
                depth = filter.filter.process(depth);
                if (filter.filter_name == disparity_filter_name) revert_disparity = true;
            }
        }
        if (revert_disparity) depth = disparity_to_depth->process(depth);
    }
    //mtx2_.lock();
    depth_que_.enqueue(depth);
    //mtx2_.unlock();
}

void RealSenseInterface::_setBytesAndStride(const rs2::video_frame& texture) {
    bytes_per_pixel = texture.get_bytes_per_pixel();
    stride_in_bytes = texture.get_stride_in_bytes();
}

void RealSenseInterface::_getTextureColor(std::array<uint8_t, 3>& rgb, const rs2::video_frame& texture, const uint8_t* texture_data, float u, float v, bool& isOut, int& classId)
{

   /* const int W = texture.get_width(), H = texture.get_height();*/
    int w = int(u * RSImageWidth_ + .5f);
    int h = int(v * RSImageHeight_ + .5f);
    //y->h�����Cx->w����
    int x = std::min(std::max(w, 0), RSImageWidth_ - 1);
    int y = std::min(std::max(h, 0), RSImageHeight_ - 1);
    if (h < 0 || RSImageHeight_ <= h) isOut = true;
    if (w < 0 || RSImageHeight_ <= w) isOut = true;
    //ES(x) EL(y)
    classId = classIdImage_[y][x];
    //PS(129) EL(classId)
    int idx = x * bytes_per_pixel + y * stride_in_bytes;
    //return { texture_data[idx], texture_data[idx + 1], texture_data[idx + 2] };
    for (int i = 0; i < 3; i++) rgb[i] = texture_data[idx+i];
}

void RealSenseInterface::_getMatImage(cv::Mat& Image, const rs2::video_frame& texture, const uint8_t* texture_data) {
    //const int H = texture.get_height(), W = texture.get_width();
    //auto bytes_per_pixel = texture.get_bytes_per_pixel();
    //auto stride_in_bytes = texture.get_stride_in_bytes();
    auto it = &(Image.at<cv::Vec3b>(0, 0));
    for (int h = 0; h < RSImageHeight_; h++) {
        //auto it = &(Image.at<cv::Vec3b>(h, 0));
        for (int w = 0; w < RSImageWidth_; w++) {
            //auto& pixel = Image.at<cv::Vec3b>(h, w);
            for (int col = 0; col < 3; col++) {
                //pixel[col] = texture_data[w * bytes_per_pixel + h * stride_in_bytes + 2 - col];
                (*it)[col] = texture_data[w * bytes_per_pixel + h * stride_in_bytes + 2 - col];
            }
            it++;
        }
    }
}

void RealSenseInterface::_findMarkers(cv::Mat& BGRImage, std::vector<Detection>& result) {

    /*////////////////////////
    classIDImage�ɂ��āCTool�̗̈��0�Ŗ��߂�
    Yellow, Green�̗̈悪0�Ŗ��܂��Ă�����C�������m�������̂ō̗p
    �댟�o��h�����߁C{Yellow, Green}��{Tool}�̏ꍇ�̂ݎ��s
    HSV�Q��
    https://qiita.com/oyngtmhr/items/26a2feec9ca9b09a8d72
    //////////////////////*/


    //���o�������̂��P�ȉ��̏ꍇ�CtipPose��K���Ȓl�ɐݒ肵�Ĕ�����D
    if (result.size() <= 1) {
        return;
    }

    //��UHSV�ɕϊ����āCYellow, Green�̒��o�ȒP��
    cvtColor(BGRImage, HSVImage_, cv::COLOR_BGR2HSV);

    //classId���Ƀ\�[�g
    sort(result.begin(), result.end(), [](const Detection& a, const Detection& b) {return a.classId < b.classId;});

    // tip, yellow, green
    //0<=H<=180
    //0<=S<=256?
    /*
    Green
    ���~�����Ƃ��C87,110,100
    �ߐڂŔw�i�����Ƃ��C88,150,88
    �w�i�����Ƃ��C88,180,120
    �ߐڂŔw�i�����Ƃ��C93,240,140
    �w�i�Ɩ��邳������S,V�͂��Ȃ�ω�����̂ŁCcalcPositions()����ZRatioLimit�ŉ������������ق����ėp�I
    */
    int dH = 15;
    int oldH[] = { 0, 30, 85, 100, 0};
    int newH[] = { 0, 0, 0, 0, 0 };
    //int dS = 50;
    //int lowS[]  = { 0, 0, 0 };
    //int highS[] = { 0, 300, 300 };
    //int lowV[]  = { 0, 0, 0 };
    //int highV[] = { 0, 300, 300 };

    //auto itHSV = &(HSVImage_.at<cv::Vec3b>(0, 0));
    //auto maxH = (*itHSV)[0];
    //auto minH = (*itHSV)[0];
    //for (int h = 0; h <= RSImageHeight_; h++) {
    //    for (int w = 0; w <= RSImageWidth_; w++) {
    //        maxH = max(maxH, (*itHSV)[0]);
    //        minH = min(minH, (*itHSV)[0]);
    //        itHSV++;
    //    }
    //}
    //EL(int(maxH));
    //EL(int(minH));

    //classID==0(Tool)�Ȃ�classIdImage_�X�V
    //�����łȂ���΁CTool�ɓ����Ă��邩�m�F���āC����Ă���΍̗p
    for (auto& rect : result) {
        int classId = rect.classId;
        int h0 = rect.box.y;
        int w0 = rect.box.x;
        chmax(h0, 0);
        chmax(w0, 0);
        int height = rect.box.height;
        int width = rect.box.width;
        int h1 = h0 + height;
        int w1 = w0 + width;
        chmin(h1, RSImageHeight_ - 1);
        chmin(w1, RSImageWidth_ -1);
        //if (classId == 2) {
        //    auto itHSV = &(HSVImage_.at<cv::Vec3b>(h0, w0));
        //    ES("a") ES(classId) ES(int((*itHSV)[0])) ES(int((*itHSV)[1])) EL(int((*itHSV)[2]))
        //    itHSV = &(HSVImage_.at<cv::Vec3b>((h0+h1)/2, (w0+w1)/2));
        //    ES(classId) ES(int((*itHSV)[0])) ES(int((*itHSV)[1])) EL(int((*itHSV)[2]))
        //}
        for (int h = h0; h <= h1; h++) {
            auto itHSV = &(HSVImage_.at<cv::Vec3b>(h, w0));
            for (int w = w0; w <= w1; w++) {
                if (classId == 0) classIdImage_[h][w] = 0;
                else if (classIdImage_[h][w] == 0) {
                    //auto& HSV = HSVImage_.at<cv::Vec3b>(h, w);
                    if (-dH + oldH[classId] <= (*itHSV)[0] && (*itHSV)[0] <= dH + oldH[classId]
                        //&& lowS[classId] <= (*itHSV)[1] && (*itHSV)[1] <= highS[classId]
                        //&& lowV[classId] <= (*itHSV)[2] && (*itHSV)[2] <= highV[classId]
                        ) {
                        (*itHSV)[0] = newH[classId];
                        classIdImage_[h][w] = classId;
                    }

                }
                itHSV++;
            }
        }
    }




    //HSV->RGBImage. ����K�v�Ȃ��̂ŃR�����g�A�E�g
    cvtColor(HSVImage_, BGRImage, cv::COLOR_HSV2BGR);
}

void RealSenseInterface::_calcPositions(std::vector<float>& points, std::vector<int>& color) {
    if (found_.size() != classNum) found_.resize(classNum, false);

    //class3DPositions_�����ɁCYellow, Green��ǉ�����
    /*
    
    */

    for (int classId = 1; classId < classNum; classId++) {
        if (class3DPositions_[classId].size() == 0) continue;
        auto& vec = class3DPositions_[classId];
        //�J��������̋���(z)���߂����Ƀ\�[�g
        std::sort(vec.begin(), vec.end(), [](const std::array<double, 3>& a, const std::array<double, 3>& b) {return a[2] < b[2]; });

        int cnt = 0;
        double lastZ = vec[0][2];
        classMeanPos_[classId] = { 0., 0., 0. };
        //���Ԃɑ������킹��
        for (const std::array<double, 3>&p : class3DPositions_[classId]) {
            if (p[2] > lastZ * ZratioLimit_) {
                //EL("RS Z ratio break")
                break;
            }

            //�J�����ŋߓ_����̔䗦�ɂ��邩�C���O�̓_����̔䗦�ɂ��邩
            if (updateLastZ_) lastZ = p[2];
            cnt++;
            for (int i = 0; i < 3; i++) classMeanPos_[classId][i] += p[i];
        }

        //Yellow, Green�̊e���ς��Ƃ��Ēǉ�
        if (cnt != 0) {
            found_[classId] = true;
            for (int i = 0; i < 3; i++) classMeanPos_[classId][i] /= cnt;
            //pcl2���Ń\�[�g���邽�߂�r����classId�����Ă����D
            int rgb[] = { 255 - classId, 0, 0 };
            //classId==1,2�̎��C255+256(=1<<8), classId==3,4�̂Ƃ��C255+512(=1<<9)
            for (int i = 0; i < 3; i++) color.push_back(rgb[i] + (1<<(8+(classId/3))));
            for (int i = 0; i < 3; i++) points.push_back(classMeanPos_[classId][i]);
        }
    }


    //�c�[����[����
    for (int i = 0; i < 2; i++) {
        if (found_[2*i+1] && found_[2*i+2]) {
            if (norm(classMeanPos_[2*i+1] - classMeanPos_[2*i+2]) > 0.1) continue;

            //////////////////////////////////��[�ʒu�␳�I�I�I�I�I�I
            //tr��v�����90deg��]��������tc�Ɠ��������ɂȂ�͂�
            //https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
            /*
            �x�N�g��v���x�N�g��k�����th������]������Ƃ��C
            v' = vcos(th)+(kxv)sin(th)+k(k*v)(1-cos(th))
            th=90deg���C
            v' = v + kxv;

            c:�c�[����[���S
            t:�c�[����[����
            m1:�}�[�J�[��[��
            m2:�}�[�J�[�쓮����

            _g:�O���[�o�����W�i�J�������W)
            _l:���[�J�����W�it�����_�j
            */
            std::array<double, 3> t_g = classMeanPos_[2*i+1] + (classMeanPos_[2*i+1] - classMeanPos_[2*i+2]) * ratio_tm1_over_m1m2_[i];

            std::array<double, 3> m1_g = classMeanPos_[2*i+1];
            std::array<double, 3> m2_g = classMeanPos_[2*i+2];
            std::array<double, 3> m1_l = m1_g - t_g;
            std::array<double, 3> k = outer_prod(m1_l, t_g);//tr,tc�ɐ����ȃx�N�g��.��]���S���D
            //EL(k)
            k = unit(k);
            //EL(k)

            std::array<double, 3> oldV = unit(m1_l);
            //EL(oldV)

            double th = 90.; th *= M_PI / 180.;
            std::array<double, 3> newV = oldV * cos(th) + (outer_prod(k, oldV)) * sin(th) + k * (inner_prod(k, oldV)) * (1. - cos(th));
            //EL(newV)

            if (norm(newV) != 0) {
                //PL("Norm not 0")
                newV = t_g + newV * length_tc / norm(newV);
                //EL(newV)
            }
            else newV = newV + t_g;

            //EL(y_l);
            //EL(t)
            //EL(newV);

            //newV
            int rgb[] = { 255, 255, 0 };

            for (int j = 0; j < 3; j++) color.push_back(rgb[j] + (1<<(8+i)));
            for (int j = 0; j < 3; j++) points.push_back(newV[j]);

        }
    }
}

void RealSenseInterface::getPointCloud(std::vector<float>& points, std::vector<int>& color) {
    if (!initialized_) return;

    auto start = clock();

    this->_process();
    //this->_process();


    rs2::frame depth_frame;
    rs2::frame color_frame;
    //mtx_.lock();
    //while(depth_que_.capacity()==0){}
    if (!depth_que_.poll_for_frame(&depth_frame)) return;
    if (!color_que_.poll_for_frame(&color_frame)) return;
    //mtx_.unlock();

    if (showTime_) { PS("RS 242") PL(clock() - start) }


    //EL(color_que_.capacity())

    if (bytes_per_pixel == -1) {//������̂ݎ��s
        _setBytesAndStride(color_frame);
    }

	//if (!color_frame) color_frame = frames.get_infrared_frame();
	pc_.map_to(color_frame);
	points_ = pc_.calculate(depth_frame);

    //EL(points_.size())

    if (points_.size() == 0) return;

	//auto verts = points_.get_vertices();
 //   auto texture_data = reinterpret_cast<const uint8_t*>(color_frame.get_data());
 //   const auto texcoords = points_.get_texture_coordinates();
    const rs2::vertex* verts = points_.get_vertices();//�_�Q�̍��W
    const uint8_t* texture_data = reinterpret_cast<const uint8_t*>(color_frame.get_data());//rgbrgbrgb...�ꎟ���z��
    const rs2::texture_coordinate* texcoords = points_.get_texture_coordinates();//�_����p�̂ǂ̂����肩
    

    
    //if (showTime_) { PS("RS 242") PL(clock() - start) }


    _getMatImage(image_, color_frame, texture_data);
    if(showImage_) cv::imshow("before", image_);
    if (showTime_) { PS("RS 246") PL(clock() - start) }


    detection_result_ = yolov7_->detect(image_);
    if (showTime_) { PS("RS 249") PL(clock() - start) }


    _findMarkers(image_, detection_result_);
    if (showTime_) { PS("RS 265") PL(clock() - start) }


    /*
    480*480�̔z��쐬�D-1�ŏ������D->done
    findTip���ŁCYellow,Green�͑Ή�����N���XID�ɕύX->done
    getTexColor�̍ہC-1����Ȃ�������queue�ɓ����H
    �e�N���X�ɂ��āC
        depth�̏��������Ɍ��Ă����D
        ���_�̕��ς��Ƃ�D�ЂƂO�̓_�Ƃ�Depth�̔䂪���ȏ�Ȃ�C�����D
        Tip, Yellow, Green�̏��ɍ��W�L�^�D�ڗ��F�ŕ\���D
    */
    

    if (showImage_) cv::imshow("after", image_);
    if (showImage_) cv::waitKey(1);


    if (points.size()) points.clear();
    if (color.size()) color.clear();
    //
    double min_distance = 1e-6;
    //RGB�f�[�^�̂Ȃ����Ӄf�[�^�������H
    bool removeOut = true;
    std::array<uint8_t, 3> rgb;
    bool isOut = false;
    int classId = -2;
    //////////////////////////////////Yellow, Green�łȂ���΁C�i�[
    int sz = points_.size();
    for (int i = 0; i < sz; ++i) {
        //�J�����ɋ߂�������폜���Ă���
        if (verts[i].z <= min_distance) continue;
        //�������Ă��r��
        if (verts[i].z >= threshold_[2]) continue;
//        if (fabs(verts[i].x) <= min_distance || fabs(verts[i].y) <= min_distance || fabs(verts[i].z) <= min_distance) continue;
        //if (fabs(verts[i].x) >= threshold_[0] || fabs(verts[i].y) >= threshold_[1] || fabs(verts[i].z) >= threshold_[2]) continue;
        isOut = false;
        classId = -2;

        //�������̂��߁C�֐��ŌĂяo�����W�J�Dinline���������C���ʂȂ��D
        //_getTextureColor(rgb, color_frame, texture_data, texcoords[i].u, texcoords[i].v, isOut, classId);
        int w = int(texcoords[i].u * RSImageWidth_ + .5f);
        int h = int(texcoords[i].v * RSImageHeight_ + .5f);
        //y->h�����Cx->w����
        int x = std::min(std::max(w, 0), RSImageWidth_ - 1);
        int y = std::min(std::max(h, 0), RSImageHeight_ - 1);
        if (h < 0 || RSImageHeight_ <= h) isOut = true;
        if (w < 0 || RSImageWidth_ <= w) isOut = true;
        classId = classIdImage_[y][x];
        int idx = x * bytes_per_pixel + y * stride_in_bytes;
        //return { texture_data[idx], texture_data[idx + 1], texture_data[idx + 2] };
        for (int i = 0; i < 3; i++) rgb[i] = texture_data[idx + i];
        
        
        if (removeOut && isOut) continue;
        if (classId > 0) {
            class3DPositions_[classId].emplace_back(array<double, 3>({ verts[i].x, verts[i].y, verts[i].z }));
            //��UYellow�����������߂ɔ�����
            //continue;
        }

        for (int j = 0; j < 3; j++) color.emplace_back(rgb[j]);
        points.emplace_back(verts[i].x);
        points.emplace_back(verts[i].y);
        points.emplace_back(verts[i].z);

    }
    if (showTime_) { PS("RS 296") PL(clock() - start) }


    _calcPositions(points, color);
    if (showTime_) { PS("RS 398") PL(clock() - start) }



    //�ϐ����Z�b�g
    for (int h = 0; h < RSImageHeight_; h++) for (int w = 0; w < RSImageWidth_; w++) classIdImage_[h][w] = -1;
    for (int classId = 0; classId < classNum; classId++) for (int i = 0; i < 3; i++) classMeanPos_[classId][i] = 0.0;
    for (int classId = 0; classId < classNum; classId++) if (class3DPositions_[classId].size()) class3DPositions_[classId].clear();
    for (int i= 0; i < classNum; i++) found_[i] = false;
    if (showTime_) { PS("RS 406") PL(clock() - start) }

    //EL(points.size());
    //EL(color.size());
}
















bool filter_slider_ui::is_all_integers(const rs2::option_range& range)
{
    const auto is_integer = [](float f)
    {
        return (fabs(fmod(f, 1)) < std::numeric_limits<float>::min());
    };

    return is_integer(range.min) && is_integer(range.max) &&
        is_integer(range.def) && is_integer(range.step);
}



filter_options::filter_options(const std::string name, rs2::filter& flt) :
    filter_name(name),
    filter(flt),
    is_enabled(true)
{
    const std::array<rs2_option, 5> possible_filter_options = {
        RS2_OPTION_FILTER_MAGNITUDE,
        RS2_OPTION_FILTER_SMOOTH_ALPHA,
        RS2_OPTION_MIN_DISTANCE,
        RS2_OPTION_MAX_DISTANCE,
        RS2_OPTION_FILTER_SMOOTH_DELTA
    };

    //Go over each filter option and create a slider for it
    for (rs2_option opt : possible_filter_options)
    {
        if (flt.supports(opt))
        {
            rs2::option_range range = flt.get_option_range(opt);
            supported_options[opt].range = range;
            supported_options[opt].value = range.def;
            supported_options[opt].is_int = filter_slider_ui::is_all_integers(range);
            supported_options[opt].description = flt.get_option_description(opt);
            std::string opt_name = flt.get_option_name(opt);
            supported_options[opt].name = name + "_" + opt_name;
            std::string prefix = "Filter ";
            supported_options[opt].label = opt_name;
        }
    }
}

filter_options::filter_options(filter_options&& other) :
    filter_name(std::move(other.filter_name)),
    filter(other.filter),
    supported_options(std::move(other.supported_options)),
    is_enabled(other.is_enabled.load())
{
}