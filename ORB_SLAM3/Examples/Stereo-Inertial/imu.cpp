// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iomanip>
#include <librealsense2/rsutil.h>
#include <array>
#include <cmath>
#include <iostream>

int main(int argc, char * argv[]) try
{
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_POSE);
    pipe.start(cfg);

    while (true) // Application still alive?
    {
        rs2::frameset frameset = pipe.wait_for_frames();
        // Find and retrieve IMU and/or tracking data
        if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
        {
            rs2_vector accel_sample = accel_frame.get_motion_data();
            std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
            //...
        }
        if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
        {
            rs2_vector gyro_sample = gyro_frame.get_motion_data();
            std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
            //...
        }
        if (rs2::pose_frame pose_frame = frameset.first_or_default(RS2_STREAM_POSE))
        {
            rs2_pose pose_sample = pose_frame.get_pose_data();
            //std::cout << "Pose:" << pose_sample.translation.x << ", " << pose_sample.translation.y << ", " << pose_sample.translation.z << std::endl;
            //...
        }
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
