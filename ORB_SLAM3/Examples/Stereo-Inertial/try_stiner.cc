/**
 * @file main.cpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief OAK-D/ORB_SLAM3 Experiments main file.
 * 
 * @version 0.1
 * @date 2021-01-13
 * 
 * @copyright Copyright (c) Duncan Hamill 2021
 */

/* -------------------------------------------------------------------------
 * INCLUDES
 * ------------------------------------------------------------------------- */

#include <iostream>

#include <opencv2/opencv.hpp>

#include "System.h"

/* NEW */
#include <librealsense2/rs.hpp>
#include <iomanip>
#include <librealsense2/rsutil.h>
#include <array>
#include <cmath>
#include <ctime>
#include <iostream>
#include <mutex>
#include <sys/time.h>

using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::chrono::system_clock;


/* -------------------------------------------------------------------------
 * CONSTANTS
 * ------------------------------------------------------------------------- */

// WLS parameters, taken from the OpenCV WLS filter docs recommended values.
#define WLS_LAMBDA (8000)
#define WLS_SIGMA (1.0)

/* -------------------------------------------------------------------------
 * STRUCTS
 * ------------------------------------------------------------------------- */

// A simple pose structure containing position vector and rotation matrix.
typedef struct _Pose {
    cv::Mat position;
    cv::Mat rotation;
} Pose;

// Global variable.
vector<ORB_SLAM3::IMU::Point> vImuMeas;
std::mutex mon_mutex;
// rs2_vector accel_temp;


void thread_IMU(rs2::pipeline pipe, vector<ORB_SLAM3::IMU::Point> & vImuMeas)
{
    // Main loop
    while (1)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        
        rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
        rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);

        rs2_vector accel_sample = accel_frame.get_motion_data();
        rs2_vector gyro_sample = gyro_frame.get_motion_data();

        // auto elapsed_time = std::chrono::steady_clock::now() - 0;

        auto millisec_since_epoch = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
        double frame_timestamp_s = millisec_since_epoch / 1000000000.0;

        mon_mutex.lock();
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(accel_sample.x, accel_sample.y, accel_sample.z,
                                                gyro_sample.x, gyro_sample.y, gyro_sample.z,
                                                frame_timestamp_s));
        mon_mutex.unlock();
        
        // if (accel_temp.x != accel_sample.x && accel_temp.y != accel_sample.y && accel_temp.z != accel_sample.z)
        // {
        // //     std::cout << " ACCEL " << accel_sample.x << " - " << accel_sample.y << " - " << accel_sample.z << std::endl <<
        // //             " GYRO " << gyro_sample.x << " - " << gyro_sample.y << " - " << gyro_sample.z << " - " << std::endl <<
        // //             " TIMESTAMPS " << millisec_since_epoch << std::endl << std::endl;

        //     accel_temp = accel_sample;

        //     mon_mutex.lock();
        //     /*vImuMeas.push_back(ORB_SLAM3::IMU::Point(accel_sample.x, accel_sample.y, accel_sample.z,
        //                                 gyro_sample.x, gyro_sample.y, gyro_sample.z,
        //                                 millisec_since_epoch / 1000000000.0));*/
        //     vImuMeas.push_back(ORB_SLAM3::IMU::Point(-accel_sample.y, accel_sample.x, accel_sample.z,
        //     -gyro_sample.y, gyro_sample.x, gyro_sample.z,
        //     millisec_since_epoch / 1000000000.0));
        //     //std::cout << " TIME " << millisec_since_epoch / 1000000000.0 << std::endl;
        //     mon_mutex.unlock();

        //     compteur ++;
        // }
    }
}


void thread_MAIN(rs2::pipeline pipeline, vector<ORB_SLAM3::IMU::Point> & vImuMeas, ORB_SLAM3::System & SLAM)
{
    cv::Mat R1, R2, P1, P2, Q;
    // We also want somewhere to store our pose data
    Pose pose;

    cv::Mat imLeft;
    cv::Mat imRight;

    int width = 640;
    int height = 480;

    while (1) {
        // Read the output frames from the OAK-D. These are blocking calls, so
        // they will wait until there's data available.
        
        rs2::frameset frameset = pipeline.wait_for_frames();

		rs2::video_frame ir_frame_left = frameset.get_infrared_frame(1);
		rs2::video_frame ir_frame_right = frameset.get_infrared_frame(2);

        imLeft = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
        imRight = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());

        // Get the time between the epoch and now, allowing us to get a
        // timestamp (in seconds) to pass into the slam system.
        
        // auto elapsed_time = std::chrono::steady_clock::now() - slam_epoch;
        
        auto millisec_since_epoch = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();

        double frame_timestamp_s = millisec_since_epoch / 1000000000.0;
        
        //std::cout << frame_timestamp_s << ": " << std::endl;

        // Into the SLAM system. This produces a matrix with
        // the pose information of the camera.

        // mon_mutex.lock();
        // std::cout << "DEBUT LECTURE" << std::endl;
        // for (auto &&vector : vImuMeas)
        // {
        //     std::cout << vector.a.x << ", " << vector.a.y << ", "<< vector.a.z << ", " << vector.w.x << ", " << vector.w.y << ", "<< vector.w.z << ", " << std::endl;
        // }
        // std::cout << "FIN DE LA LECTURE" << std::endl;
        // std::cout << "IMU SIZE BEFORE SLAM " << vImuMeas.size() << std::endl;
        
        
        // if (compteur > 9)
        // {
        //     mon_mutex.lock();
        //     cv::Mat raw_pose = SLAM.TrackStereo(
        //         imLeft,
        //         imRight,
        //         frame_timestamp_s,  // rs2::frameset frameset = pipe.wait_for_frames();
        //         vImuMeas
        //     // Pass the images i
        //     );

        //     vImuMeas.clear();
        //     compteur = 0;
        //     mon_mutex.unlock();
        // }

        mon_mutex.lock();
        cv::Mat raw_pose = SLAM.TrackStereo(
            imLeft,
            imRight,
            frame_timestamp_s,  // rs2::frameset frameset = pipe.wait_for_frames();
            vImuMeas
        );
        vImuMeas.clear();
        mon_mutex.unlock();

        cv::imshow("disparity", imLeft);
        if (cv::waitKey(1) == 'q') {
            break;
        }
        std::cout << std::endl;
    }

    // Stop all SLAM threads
    SLAM.Shutdown();
}

/* -------------------------------------------------------------------------
 * MAIN
 * ------------------------------------------------------------------------- */

int main(int argc, char *argv[]) {

    std::cout << "OAK-D/ORB_SLAM3 Experiment" << std::endl;
    
    // The last input tells the system to display it's UI.
    ORB_SLAM3::System SLAM(
        "Vocabulary/ORBvoc.txt",
        "Examples/Stereo-Inertial/EuRoC.yaml",
        ORB_SLAM3::System::IMU_STEREO, 
        true
    );

    //vector<ORB_SLAM3::IMU::Point> vImuMeas;

    // Debut du SLAM.
	// std::chrono::_V2::steady_clock::time_point slam_epoch = std::chrono::steady_clock::now();
	
	// CAMERA D435
    int width = 640;
	int height = 480;
	int fps = 30;
	rs2::config config_d435;
	std::string serial_number_d435 = "909512070031";
	config_d435.enable_device(serial_number_d435); // Serial number of d435.
	config_d435.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	config_d435.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    rs2::pipeline pipe_d435;

	rs2::pipeline_profile pipeline_profile = pipe_d435.start(config_d435);
    rs2::device select_divise = pipeline_profile.get_device();
    auto depth_depth = select_divise.first<rs2::depth_sensor>();
	depth_depth.set_option(RS2_OPTION_EMITTER_ENABLED, 0);   

	// CAMERA T265
	rs2::config config_t265;
	std::string serial_number_t265 = "925122110153";
	config_t265.enable_device(serial_number_t265); // Serial number of t265.
	config_t265.enable_stream(RS2_STREAM_GYRO);
    config_t265.enable_stream(RS2_STREAM_ACCEL);
	rs2::pipeline pipe_t265;
	pipe_t265.start(config_t265);
    // int compteur = 0;
	
	// RUN THREAD.
	auto thread1 = std::thread(&thread_IMU,  std::ref(pipe_t265), std::ref(vImuMeas)); //, std::ref(slam_epoch)); //, std::ref(compteur));
	auto thread2 = std::thread(&thread_MAIN, std::ref(pipe_d435), std::ref(vImuMeas), std::ref(SLAM)); //, std::ref(slam_epoch), std::ref(SLAM)); //, std::ref(compteur));
	
	thread1.join();
    thread2.join();

    return 0;
}
