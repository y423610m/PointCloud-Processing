#include "realsense_interface.h"
#include "development_commands.h"
#include "ros_param.h"

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

    //幅，高さ，奥行
    threshold_ = { 0.5, 2.0, 1.0 };

    cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
    cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 15);
    pipe_.start(cfg_);

    this->_setFilters();

    yolov7_.reset(new YOLODetector("C:/Users/y4236/Documents/ToolDetectYOLO/yolov7/CobottaTool/result/train8/weights/last.onnx", false, cv::Size(480, 480)));
    yolov7_->setConfThreshold(0.25);
    yolov7_->setIouThreshold(0.65);

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
    EL(ROSParam::getIntParam("RS_enable_FILTER_Decimate"))
    // The following order of emplacement will dictate the orders in which filters are applied
    if (ROSParam::getIntParam("RS_enable_FILTER_Decimate"))filters_.emplace_back("Decimate", dec_filter);
    //if (ROSParam::getIntParam("RS_enable_RS")) filters_.emplace_back("Threshold", thr_filter);
    if (ROSParam::getIntParam("RS_enable_FILTER_Disparity")) filters_.emplace_back(disparity_filter_name, depth_to_disparity);
    if (ROSParam::getIntParam("RS_enable_FILTER_Spatial")) filters_.emplace_back("Spatial", spat_filter);
    if (ROSParam::getIntParam("RS_enable_FILTER_Temporal")) filters_.emplace_back("Temporal", temp_filter);
    if (ROSParam::getIntParam("RS_enable_FILTER_Hole")) filters_.emplace_back("Hole", hole_filter);
}

//pipeから受け取り，filterかけてからqueueに挿入. 別スレッドで回す予定
void RealSenseInterface::_process() {
    rs2::frameset frames = pipe_.wait_for_frames();

    rs2::frame depth = frames.get_depth_frame();
    if (!depth) {
        EL("No Depth Frame");
        return;
    }


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

    depth_que_.enqueue(depth);
    color_que_.enqueue(frames.get_color_frame());
}

std::array<uint8_t, 3> get_texcolor(const rs2::video_frame& texture, const uint8_t* texture_data, float u, float v, bool& isOut)
{
    const int w = texture.get_width(), h = texture.get_height();
    int x = std::min(std::max(int(u * w + .5f), 0), w - 1);
    if (w <= int(u * w + .5f)) isOut = true;
    int y = std::min(std::max(int(v * h + .5f), 0), h - 1);
    //ES(x) EL(y)
    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    return { texture_data[idx], texture_data[idx + 1], texture_data[idx + 2] };
}

cv::Mat makeMatImage(const rs2::video_frame& texture, const uint8_t* texture_data) {
    const int H = texture.get_height(), W = texture.get_width();
    cv::Mat Image(H, W, CV_8UC3);//BGRで格納
    for (int h = 0; h < H; h++) {
        //auto Image_row_h = Image.ptr(h);
        for (int w = 0; w < W; w++) {
            for (int col = 0; col < 3; col++) {
                Image.at<cv::Vec3b>(h, w)[col] = texture_data[w * texture.get_bytes_per_pixel() + h * texture.get_stride_in_bytes()+2-col];
            }
        }
    }
    return Image;
}

void RealSenseInterface::getPointCloud(std::vector<float>& points, std::vector<int>& color) {
    if (!initialized_) return;

    this->_process();

    rs2::frame depth_frame;
    rs2::frame color_frame;
    if (!depth_que_.poll_for_frame(&depth_frame)) return;
    if (!color_que_.poll_for_frame(&color_frame)) return;



	//if (!color_frame) color_frame = frames.get_infrared_frame();
	pc_.map_to(color_frame);
	points_ = pc_.calculate(depth_frame);

    //EL(points_.size())

    if (points_.size() == 0) return;

	//auto verts = points_.get_vertices();
 //   auto texture_data = reinterpret_cast<const uint8_t*>(color_frame.get_data());
 //   const auto texcoords = points_.get_texture_coordinates();
    const rs2::vertex* verts = points_.get_vertices();//点群の座標
    const uint8_t* texture_data = reinterpret_cast<const uint8_t*>(color_frame.get_data());//rgbrgbrgb...一次元配列
    const rs2::texture_coordinate* texcoords = points_.get_texture_coordinates();//点が画角のどのあたりか

    cv::Mat image = makeMatImage(color_frame, texture_data);
    std::vector<Detection> result = yolov7_->detect(image);
    //tipPosionを求める
    
    cv::imshow("image", image);
    cv::waitKey(1);

    if (points.size()) points.clear();
    if (color.size()) color.clear();

    double min_distance = 1e-6;
    //RGBデータのない周辺データを除く？
    bool removeOut = true;
    for (size_t i = 0; i < points_.size(); ++i) {
        if (fabs(verts[i].x) <= min_distance || fabs(verts[i].y) <= min_distance || fabs(verts[i].z) <= min_distance) continue;
        if (fabs(verts[i].x) >= threshold_[0] || fabs(verts[i].y) >= threshold_[1] || fabs(verts[i].z) >= threshold_[2]) continue;
        bool isOut = false;
        std::array<uint8_t, 3> rgb = get_texcolor(color_frame, texture_data, texcoords[i].u, texcoords[i].v, isOut);
        if (removeOut && isOut) continue;
        for (int j = 0; j < 3; j++) color.push_back(rgb[j]);
        points.push_back(verts[i].x);
        points.push_back(verts[i].y);
        points.push_back(verts[i].z);
    }
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