// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iomanip>
#include <librealsense2/rsutil.h>
#include <array>
#include <cmath>
#include <iostream>
// #include <vector>
// #include "example.hpp"

int main(int argc, char * argv[]) try
{
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2);
    // Start pipeline with chosen configuration
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);

    // T265 has two fisheye sensors, we can choose any of them (index 1 or 2)
    const int fisheye_sensor_idx = 1;

    // Get fisheye sensor intrinsics parameters
    rs2::stream_profile fisheye_stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
    rs2_intrinsics intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();

    std::cout << "Device got. Streaming data" << std::endl;

    // // Create an OpenGL display window and a texture to draw the fisheye image
    // window app(intrinsics.width, intrinsics.height, "Intel RealSense T265 Augmented Reality Example");
    // window_key_listener key_watcher(app);
    // texture fisheye_image;


    // // Start pipeline with chosen configuration
    // pipe.start(cfg);

    // Main loop
    while (true)
    {

        rs2_pose device_pose_in_world; // This will contain the current device pose
        {
            rs2_pose device_pose_in_world; // This will contain the current device pose
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();
            // Get a frame from the fisheye stream
            rs2::video_frame fisheye_frame = frames.get_fisheye_frame(fisheye_sensor_idx);
            // Get a frame from the pose stream
            rs2::pose_frame pose_frame = frames.get_pose_frame();

            // Get a frame from the pose stream
            auto f = frames.first_or_default(RS2_STREAM_POSE);
            // Cast the frame to pose_frame and get its data
            auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

            // Print the x, y, z values of the translation, relative to initial position
            std::cout << "\r" << "Device Position: " << std::setprecision(3) << pose_data.translation.x << " " <<
                pose_data.translation.y << " " << pose_data.translation.z << " (meters)" << std::endl;

            std::cout << "\r" << "Device Acceleration: " << std::setprecision(3) << pose_data.acceleration << std::endl;

            std::cout << "\r" << "Device Velocity: " << std::setprecision(3) << pose_data.velocity << std::endl;


            // Copy current camera pose
            device_pose_in_world = pose_frame.get_pose_data();

            // Render the fisheye image
            // fisheye_image.render(fisheye_frame, { 0, 0, app.width(), app.height() });

            // By closing the current scope we let frames be deallocated, so we do not fill up librealsense queues while we do other computation.
        }


        // // Wait for the next set of frames from the camera
        // auto frames = pipe.wait_for_frames();
        // // Get a frame from the pose stream
        // auto f = frames.first_or_default(RS2_STREAM_POSE);
        // // Cast the frame to pose_frame and get its data
        // auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        // // Print the x, y, z values of the translation, relative to initial position
        // std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
        //     pose_data.translation.y << " " << pose_data.translation.z << " (meters)";
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
