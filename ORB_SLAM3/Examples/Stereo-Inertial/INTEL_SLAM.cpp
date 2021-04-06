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
#include<fstream>
#include <sstream>

using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::chrono::system_clock;

int main(int argc, char * argv[])
{

    // The last input tells the system to display it's UI.
    ORB_SLAM3::System SLAM(
        "Vocabulary/ORBvoc.txt",
        "Examples/Stereo-Inertial/EuRoC.yaml",
        ORB_SLAM3::System::IMU_STEREO
    );

    cv::Mat imLeft;
    cv::Mat imRight;

    int width = 640;
    int height = 480;

    // Start a streaming pipe per each connected device
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

    int counter = 0;

    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    cv::Mat imLeftRect, imRightRect;

    // Read rectification parameters
    cv::FileStorage fsSettings("Examples/Stereo-Inertial/EuRoC.yaml", cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    rs2::frameset frames_T265 = pipe_t265.wait_for_frames();
    rs2::motion_frame accel_frame = frames_T265.first(RS2_STREAM_ACCEL);
    rs2::motion_frame gyro_frame = frames_T265.first(RS2_STREAM_GYRO);
    rs2_vector accel_sample = accel_frame.get_motion_data();;
    rs2_vector gyro_sample = gyro_frame.get_motion_data();

    while (1)
    {
        while (counter < 10)
        {
            frames_T265 = pipe_t265.wait_for_frames();
            accel_frame = frames_T265.first(RS2_STREAM_ACCEL);
            gyro_frame = frames_T265.first(RS2_STREAM_GYRO);

            accel_sample = accel_frame.get_motion_data();
            gyro_sample = gyro_frame.get_motion_data();

            // std::cout << " ACCEL SAMPLE........ " << accel_sample << " .......END "<< std::endl;
            // std::cout << " GYRO  SAMPLE........ " << gyro_sample << " .......END "<< std::endl;

            // std::cout << " ACCEL " << accel_sample.x << " - " << accel_sample.y << " - " << accel_sample.z << std::endl;

            // auto millisec_since_epoch = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
            // double frame_timestamp_s = millisec_since_epoch / 1000000000.0;
       
            // vImuMeas.push_back(ORB_SLAM3::IMU::Point(accel_sample.x, accel_sample.y, accel_sample.z,
            //                                         gyro_sample.x, gyro_sample.y, gyro_sample.z,
            //                                         frame_timestamp_s));

            vImuMeas.push_back(ORB_SLAM3::IMU::Point(accel_sample.x, accel_sample.y, accel_sample.z,
                                                    gyro_sample.x, gyro_sample.y, gyro_sample.z,
                                                    gyro_frame.get_timestamp()));

            std::cout << " TIMESTAMP : " << accel_frame.get_timestamp() << std::endl;
            
            counter ++;
        }
        counter = 0;

        // std::cout << "DEBUT LECTURE" << std::endl;
        // for (auto &&vector : vImuMeas)
        // {
        //     std::cout << vector.a.x << ", " << vector.a.y << ", "<< vector.a.z << ", " << vector.w.x << ", " << vector.w.y << ", "<< vector.w.z << ", " << std::endl;
        // }
        // std::cout << "FIN DE LA LECTURE" << std::endl;
        // std::cout << "IMU SIZE BEFORE SLAM " << vImuMeas.size() << std::endl;

        rs2::frameset frames_D435 = pipe_d435.wait_for_frames();
		rs2::video_frame ir_frame_left = frames_D435.get_infrared_frame(1);
		rs2::video_frame ir_frame_right = frames_D435.get_infrared_frame(2);

        auto millisec_since_epoch = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
        double frame_timestamp_s = millisec_since_epoch / 1000000000.0;

        imLeft = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
        imRight = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());

        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

        if(vImuMeas.size() > 0)
        {
            frame_timestamp_s = vImuMeas[vImuMeas.size()-1].t;
            std::cout << "HELLOOOOOO " << vImuMeas[0].t << std::endl;
        }
        else
        {
            millisec_since_epoch = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
            frame_timestamp_s = millisec_since_epoch / 1000000000.0;
        } 

        cv::Mat raw_pose = SLAM.TrackStereo(
            imLeftRect,
            imRightRect,
            frame_timestamp_s,  // rs2::frameset frameset = pipe.wait_for_frames();
            vImuMeas
        );
        vImuMeas.clear();

        // D435 FRAME
        // cv::imshow("disparity", imLeftRect);
        if (cv::waitKey(1) == 'q') {
            break;
        }   
        // std::cout << " ...........WHILE END.......... " << std::endl;     
    }

    // Stop all SLAM threads
    SLAM.Shutdown();
}



// // Wait for the next set of frames from the camera
// auto frames = pipe.wait_for_frames();
// // Get a frame from the pose stream
// auto f = frames.first_or_default(RS2_STREAM_POSE);
// // Cast the frame to pose_frame and get its data
// auto pose_data = f.as<rs2::pose_frame>().get_pose_data();


// while (1)
// {
//     while (counter < 10)
//     {
//         auto frames = pipe.wait_for_frames();
//         f = frames.first_or_default(RS2_STREAM_POSE);
//         pose_data = f.as<rs2::pose_frame>().get_pose_data();

//         // auto millisec_since_epoch = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
//         // double frame_timestamp_s = millisec_since_epoch / 1000000000.0;

//         vImuMeas.push_back(ORB_SLAM3::IMU::Point(pose_data.acceleration.x, pose_data.acceleration.y, pose_data.acceleration.z,
//                                                 pose_data.angular_velocity.x, pose_data.angular_velocity.y, pose_data.angular_velocity.z,
//                                                 pose_data.));

//         // Print the x, y, z values of the acceleration, relative to initial position
//         std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.acceleration.x << " " <<
//             pose_data.acceleration.y << " " << pose_data.acceleration.z << " (meters/sec2)" << std::endl;
        
//         std::cout << "....................." << std::endl;

//         std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.angular_velocity.x << " " <<
//             pose_data.angular_velocity.y << " " << pose_data.angular_velocity.z << " (radian/sec)" << std::endl;
        
//         counter ++;
//     }
// counter = 0;