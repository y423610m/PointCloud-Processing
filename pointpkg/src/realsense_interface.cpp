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


RealSenseInterface::RealSenseInterface() {
    if (!ROSParam::getIntParam("RS_enable_RS")) return;

    //ïùÅCçÇÇ≥ÅCâúçs
    threshold_ = { 0.5, 2.0, 1.0 };

    cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
    cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 15);
    pipe_.start(cfg_);

    initialized_ |= 1;
    cerr << "RealSenseInterface constructed" << endl;
    return;

    auto pro = pipe_.get_active_profile();
    auto str = pro.get_streams();
    for (int i = 0; i < str.size(); i++) {
        ES(str[i].stream_type());
        ES(str[i].format());
        ES(str[i].fps());
        PL("")
    }
    return;

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


std::array<uint8_t, 3> get_texcolor(const rs2::video_frame& texture, const uint8_t* texture_data, float u, float v)
{
    const int w = texture.get_width(), h = texture.get_height();
    int x = std::min(std::max(int(u * w + .5f), 0), w - 1);
    int y = std::min(std::max(int(v * h + .5f), 0), h - 1);
    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    return { texture_data[idx], texture_data[idx + 1], texture_data[idx + 2] };
}

void RealSenseInterface::getPointCloud(std::vector<float>& points, std::vector<int>& color) {

    if (!initialized_) return;

    rs2::frameset frames = pipe_.wait_for_frames();
	rs2::video_frame color_frame = frames.get_color_frame();
	//if (!color_frame) color_frame = frames.get_infrared_frame();
	pc_.map_to(color_frame);
	rs2::depth_frame depth = frames.get_depth_frame();
	points_ = pc_.calculate(depth);

    //EL(points_.size())

    if (points_.size() == 0) return;

	auto verts = points_.get_vertices();
    auto texture_data = reinterpret_cast<const uint8_t*>(color_frame.get_data());
    const auto texcoords = points_.get_texture_coordinates();

    if (points.size()) points.clear();
    if (color.size()) color.clear();

    double min_distance = 1e-6;
    for (size_t i = 0; i < points_.size(); ++i) {
        if (fabs(verts[i].x) <= min_distance || fabs(verts[i].y) <= min_distance || fabs(verts[i].z) <= min_distance) continue;
        if (fabs(verts[i].x) >= threshold_[0] || fabs(verts[i].y) >= threshold_[1] || fabs(verts[i].z) >= threshold_[2]) continue;
        
        //points.push_back(verts[i].x);
        //points.push_back(verts[i].z);
        //points.push_back(-verts[i].y);
        points.push_back(verts[i].x);
        points.push_back(verts[i].y);
        points.push_back(verts[i].z);
        auto rgb = get_texcolor(color_frame, texture_data, texcoords[i].u, texcoords[i].v);
        for (int j = 0; j < 3; j++) color.push_back(rgb[j]);

    }
}

