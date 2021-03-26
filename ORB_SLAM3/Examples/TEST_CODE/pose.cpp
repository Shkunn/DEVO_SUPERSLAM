// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <librealsense2/rsutil.h>
#include <array>
#include <cmath>
#include <iostream>
using namespace cv;

int main(int argc, char * argv[])
{
	int width = 1280;
	int height = 720;
	int fps = 30;
	rs2::config config;
	config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);

	// start pipeline
	rs2::pipeline pipeline;
	rs2::pipeline_profile pipeline_profile = pipeline.start(config);
    rs2::device select_divise = pipeline_profile.get_device();
    auto depth_depth = select_divise.first<rs2::depth_sensor>();
	depth_depth.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
    while (1) // Application still alive?
	{
		// wait for frames and get frameset
		rs2::frameset frameset = pipeline.wait_for_frames();

		// get single infrared frame from frameset
		//rs2::video_frame ir_frame = frameset.get_infrared_frame();

		// get left and right infrared frames from frameset
		rs2::video_frame ir_frame_left = frameset.get_infrared_frame(1);
		rs2::video_frame ir_frame_right = frameset.get_infrared_frame(2);

		cv::Mat dMat_left = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
		cv::Mat dMat_right = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());

		cv::imshow("img_l", dMat_left);
		cv::imshow("img_r", dMat_right);
		char c = cv::waitKey(1);
		if (c == 's')
		{

		}
		else if (c == 'q')
			break;
	}

	return EXIT_SUCCESS;
}

