// // /**
// //  * @file main.cpp
// //  * @author Duncan Hamill (duncanrhamill@googlemail.com)
// //  * @brief OAK-D/ORB_SLAM3 Experiments main file.
// //  * 
// //  * @version 0.1
// //  * @date 2021-01-13
// //  * 
// //  * @copyright Copyright (c) Duncan Hamill 2021
// //  */

// // /* -------------------------------------------------------------------------
// //  * INCLUDES
// //  * ------------------------------------------------------------------------- */

// // #include <iostream>

// // #include <opencv2/opencv.hpp>

// // #include "System.h"

// // /* NEW */
// // #include <librealsense2/rs.hpp>
// // #include <iomanip>
// // #include <librealsense2/rsutil.h>
// // #include <array>
// // #include <cmath>
// // #include <iostream>

// // using namespace cv;

// // /* -------------------------------------------------------------------------
// //  * CONSTANTS
// //  * ------------------------------------------------------------------------- */

// // // WLS parameters, taken from the OpenCV WLS filter docs recommended values.
// // #define WLS_LAMBDA (8000)
// // #define WLS_SIGMA (1.0)

// // /* -------------------------------------------------------------------------
// //  * STRUCTS
// //  * ------------------------------------------------------------------------- */

// // // A simple pose structure containing position vector and rotation matrix.
// // typedef struct _Pose {
// //     cv::Mat position;
// //     cv::Mat rotation;
// // } Pose;

// // /* -------------------------------------------------------------------------
// //  * MAIN
// //  * ------------------------------------------------------------------------- */

// // int main(int argc, char *argv[]) {

// //     std::cout << "OAK-D/ORB_SLAM3 Experiment" << std::endl;
    
// //     cv::Mat R1, R2, P1, P2, Q;

// //     // Create the SLAM system. First argument is path to the ORB_SLAM3 vocab
// //     // file. The second is the path to the settings file for this particular
// //     // camera setup. The values in this file were taken from what's printed out
// //     // of `depthai_demo.py`.
// //     //
// //     // While the OAK-D does have an IMU we can't use it right now, but support
// //     // is coming soon! For now just use stereo mode for SLAM, which isn't as
// //     // accurate as IMU_STEREO.
// //     //
// //     // The last input tells the system to display it's UI.
// //     ORB_SLAM3::System SLAM(
// //         "Vocabulary/ORBvoc.txt",
// //         "Examples/Stereo/EuRoCT265.yaml",
// //         ORB_SLAM3::System::STEREO, 
// //         true
// //     );

// //     // We also want somewhere to store our pose data
// //     Pose pose;

// //     // The time of each frame is required for SLAM, so we take an epoch time
// //     // (i.e. our start time) now
// //     auto slam_epoch = std::chrono::steady_clock::now();

// //     // CAMERA.
// //     int width = 848;
// // 	int height = 800;
// // 	int fps = 30;

// //     // CAMERA T265
// // 	rs2::config config_t265;
// // 	std::string serial_number_t265 = "925122110153";
// // 	config_t265.enable_device(serial_number_t265); // Serial number of t265.
// //     config_t265.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
// //     config_t265.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

// //    	rs2::pipeline pipe_t265;
// //     pipe_t265.start(config_t265);

// //     cv::Mat imLeft; 
// //     cv::Mat imRight; 

// //     // Now for the main loop
// //     while (1) {

// // 		// rs2::frameset frameset = pipeline.wait_for_frames();

// // 		// rs2::video_frame ir_frame_left = frameset.get_infrared_frame(1);
// // 		// rs2::video_frame ir_frame_right = frameset.get_infrared_frame(2);

// //         // imLeft = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
// //         // imRight = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());
// //         rs2::frameset frameset = pipe_t265.wait_for_frames();

// // 		rs2::video_frame fisheye_frame_left = frameset.get_fisheye_frame(1);
// // 		rs2::video_frame fisheye_frame_right = frameset.get_fisheye_frame(2);

// //         imLeft = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)fisheye_frame_left.get_data());
// //         imRight = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)fisheye_frame_right.get_data());

// //         // Get the time between the epoch and now, allowing us to get a
// //         // timestamp (in seconds) to pass into the slam system.
// //         auto elapsed_time = std::chrono::steady_clock::now() - slam_epoch;
// //         double frame_timestamp_s = elapsed_time.count() / 1000000000.0;

// //         std::cout << std::setprecision(4) << "TimeStamp (sec): " << frame_timestamp_s << ": " << std::endl;

// //         // Into the SLAM system. This produces a matrix with
// //         // the pose information of the camera.
// //         cv::Mat raw_pose = SLAM.TrackStereo(
// //             imLeft,
// //             imRight,
// //             frame_timestamp_s  // rs2::frameset frameset = pipe.wait_for_frames();
// //         );

// //         // The output pose may be empty if the system was unable to track the
// //         // movement, so only get position and rotation if pose isn't empty. We
// //         // also put this info an a localisation fix available flag for later
// //         // use. 
// //         bool loc_fix_available = !raw_pose.empty();
// //         if (loc_fix_available) {
// //             // The pose matrix is a 4x4 extrinsic matrix, with the form:
// //             // [R_3x3 T_3x1; [0 0 0 1]], we can find the camera position with 
// //             // C = -R'T (R' = R transpose).
// //             pose.rotation = raw_pose(cv::Rect(0, 0, 3, 3));
// //             cv::Mat T = raw_pose(cv::Rect(3, 0, 1, 3));
// //             pose.position = -pose.rotation.t()*T;

// //             // Print the updated position, but transpose it so that instead of
// //             // a column vector we have a row vector, which is easier to read.
// //             // std::cout << 
// //             //     "position: " << 
// //             //     pose.position.t() << 
// //             //     std::endl;
// //         }
// //         else {
// //             // If we didn't get a pose update log it.
// //             std::cout << "no pose update" << std::endl;
// //         }

// //         // // Apply a colormap to the filtered disparity map, but don't normalise
// //         // // it. Normalising the map will mean that the color doesn't correspond
// //         // // directly with disparity.
// //         // cv::Mat colour_disp;
// //         // /*cv::applyColorMap(imLeft, colour_disp, cv::COLORMAP_JET);*/
        
// //         // cv::imshow("disparity", imLeft);

// //         // See if q pressed, if so quit
// //         if (cv::waitKey(1) == 'q') {
// //             break;
// //         }
// //     }

// //     // Stop all SLAM threads
// //     SLAM.Shutdown();

// //     return EXIT_SUCCESS;
// // }




// /**
//  * @file main.cpp
//  * @author Duncan Hamill (duncanrhamill@googlemail.com)
//  * @brief OAK-D/ORB_SLAM3 Experiments main file.
//  * 
//  * @version 0.1
//  * @date 2021-01-13
//  * 
//  * @copyright Copyright (c) Duncan Hamill 2021
//  */

// /* -------------------------------------------------------------------------
//  * INCLUDES
//  * ------------------------------------------------------------------------- */

// #include <iostream>

// #include <opencv2/opencv.hpp>


// /* NEW */
// #include <librealsense2/rs.hpp>
// #include <iomanip>
// #include <librealsense2/rsutil.h>
// #include <array>
// #include <cmath>
// #include <ctime>
// #include <iostream>
// #include <mutex>
// #include <thread>
// #include <sys/time.h>

// using std::chrono::duration_cast;
// using std::chrono::nanoseconds;
// using std::chrono::system_clock;


// /* -------------------------------------------------------------------------
//  * CONSTANTS
//  * ------------------------------------------------------------------------- */

// // WLS parameters, taken from the OpenCV WLS filter docs recommended values.
// #define WLS_LAMBDA (8000)
// #define WLS_SIGMA (1.0)

// /* -------------------------------------------------------------------------
//  * STRUCTS
//  * ------------------------------------------------------------------------- */

// // A simple pose structure containing position vector and rotation matrix.
// typedef struct _Pose {
//     cv::Mat position;
//     cv::Mat rotation;
// } Pose;

// // Global variable.
// std::mutex mon_mutex;
// // rs2_vector accel_temp;


// void thread_IMU(rs2::pipeline pipe)
// {
//     // Main loop
//     auto frames = pipe.wait_for_frames();
//     auto last_frame = frames.get_timestamp();
//     while (1)
//     {
//         // Wait for the next set of frames from the camera
//         frames = pipe.wait_for_frames();
        
//         rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
//         rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);

//         rs2_vector accel_sample = accel_frame.get_motion_data();
//         rs2_vector gyro_sample = gyro_frame.get_motion_data();

//         // auto elapsed_time = std::chrono::steady_clock::now() - 0;

//         auto millisec_since_epoch = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
//         double frame_timestamp_s = millisec_since_epoch / 1000000000.0;

//         /*std::cout << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << 
//                     ", " << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << 
//                     ", " << 1/(frames.get_timestamp()-last_frame)*1000 << std::endl;*/
//         std::cout << "IMU (fps) : " << 1/(frames.get_timestamp()-last_frame)*1000 << " timestamp : " << std::setprecision(20) <<  frames.get_timestamp() << std::endl;

//         last_frame = frames.get_timestamp();
//     }
// }


// void thread_MAIN(rs2::pipeline pipeline)
// {
//     cv::Mat R1, R2, P1, P2, Q;
//     // We also want somewhere to store our pose data
//     Pose pose;

//     cv::Mat imLeft;
//     cv::Mat imRight;

//     int width = 848;
// 	int height = 800;

//     rs2::frameset frameset = pipeline.wait_for_frames();
//     auto last_frame = frameset.get_timestamp();

//     while (1) {
//         // Read the output frames from the OAK-D. These are blocking calls, so
//         // they will wait until there's data available.
        
//         frameset = pipeline.wait_for_frames();

//         rs2::video_frame fisheye_frame_left = frameset.get_fisheye_frame(1);
// 		rs2::video_frame fisheye_frame_right = frameset.get_fisheye_frame(2);

//         imLeft = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)fisheye_frame_left.get_data());
//         imRight = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)fisheye_frame_right.get_data());
        
//         /*auto millisec_since_epoch = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();

//         double frame_timestamp_s = millisec_since_epoch / 1000000000.0;*/
        
//         cv::imshow("disparity", imLeft);
//         if (cv::waitKey(1) == 'q') {
//             break;
//         }
//         std::cout << "CAMERA (fps) : " << 1/(frameset.get_timestamp()-last_frame)*1000 << " timestamp : " << std::setprecision(20) << frameset.get_timestamp() << std::endl;
//         last_frame = frameset.get_timestamp();
//     }
// }

// // /* -------------------------------------------------------------------------
// //  * MAIN
// //  * ------------------------------------------------------------------------- */

// // int main(int argc, char *argv[]) {

// //     /*std::cout << "OAK-D/ORB_SLAM3 Experiment" << std::endl;	

// // 	// CAMERA T265
// // 	rs2::config config_t265;
// // 	std::string serial_number_t265 = "925122110153";
// // 	config_t265.enable_device(serial_number_t265); // Serial number of t265.
// // 	config_t265.enable_stream(RS2_STREAM_GYRO);
// //     config_t265.enable_stream(RS2_STREAM_ACCEL);
// //     config_t265.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
// //     config_t265.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

// // 	rs2::pipeline pipe_t265;
// // 	pipe_t265.start(config_t265);*/

// //     // Declare RealSense pipeline, encapsulating the actual device and sensors
// //     rs2::pipeline pipe_t265;
// //     // Create a configuration for configuring the pipeline with a non default profile
// //     rs2::config cfg;
// //     // Add pose stream
// //     cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
// //     // Enable both image streams
// //     // Note: It is not currently possible to enable only one
// //     cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
// //     cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

// //     std::mutex data_mutex;
// //     uint64_t pose_counter = 0;
// //     uint64_t frame_counter = 0;
// //     bool first_data = true;
// //     auto last_print = std::chrono::system_clock::now();
// //     auto callback = [&](const rs2::frame& frame)
// //     {
// //         std::lock_guard<std::mutex> lock(data_mutex);
// //         // Only start measuring time elapsed once we have received the
// //         // first piece of data
// //         if (first_data) {
// //             first_data = false;
// //             last_print = std::chrono::system_clock::now();
// //         }

// //         if (auto fp = frame.as<rs2::pose_frame>()) {
// //             pose_counter++;
// //         }
// //         else if (auto fs = frame.as<rs2::frameset>()) {
// //             frame_counter++;
// //         }

// //         // Print the approximate pose and image rates once per second
// //         auto now = std::chrono::system_clock::now();
// //         if (now - last_print >= std::chrono::seconds(1)) {
// //             std::cout << "\r" << std::setprecision(0) << std::fixed 
// //                     << "Pose rate: "  << pose_counter << " "
// //                     << "Image rate: " << frame_counter << std::flush;
// //             pose_counter = 0;
// //             frame_counter = 0;
// //             last_print = now;
// //         }
// //     };

// //     // Start streaming through the callback
// //     rs2::pipeline_profile profiles = pipe_t265.start(cfg, callback);

// //     // Sleep this thread until we are done
// //     while(true) {
// //         std::this_thread::sleep_for(std::chrono::milliseconds(10));
// //     }


// // 	// RUN THREAD.
// //     /*
// // 	auto thread1 = std::thread(&thread_IMU,  std::ref(pipe_t265));
// // 	auto thread2 = std::thread(&thread_MAIN, std::ref(pipe_t265));
	
// // 	thread1.join();
// //     thread2.join();*/

// //     return 0;
// // }


// /* -------------------------------------------------------------------------
//  * NEW MAIN CHANGED
//  * ------------------------------------------------------------------------- */

// int main(int argc, char *argv[]) {

//     /*std::cout << "OAK-D/ORB_SLAM3 Experiment" << std::endl;	

// 	// CAMERA T265
// 	rs2::config config_t265;
// 	std::string serial_number_t265 = "925122110153";
// 	config_t265.enable_device(serial_number_t265); // Serial number of t265.
// 	config_t265.enable_stream(RS2_STREAM_GYRO);
//     config_t265.enable_stream(RS2_STREAM_ACCEL);
//     config_t265.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
//     config_t265.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

// 	rs2::pipeline pipe_t265;
// 	pipe_t265.start(config_t265);*/

//     // Declare RealSense pipeline, encapsulating the actual device and sensors
//     rs2::pipeline pipe_t265;
//     // Create a configuration for configuring the pipeline with a non default profile
//     rs2::config cfg;
//     // Add pose stream
//     // cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
//     // Add motion stream
//     cfg.enable_stream(RS2_STREAM_GYRO);
//     cfg.enable_stream(RS2_STREAM_ACCEL);
//     // Enable both image streams
//     // Note: It is not currently possible to enable only one
//     cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
//     cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

//     cv::Mat imLeft;
//     cv::Mat imRight;

//     int width = 848;
// 	int height = 800;

//     bool new_frame = false;

//     std::mutex data_mutex;
//     uint64_t accel_counter = 0;
//     uint64_t gyro_counter = 0;
//     uint64_t motion_counter = 0;
//     uint64_t frame_counter = 0;
//     double current_timestamp = 0;
//     bool first_data = true;
//     auto last_print = std::chrono::system_clock::now();
//     auto callback = [&](const rs2::frame& frame)
//     {
//         std::lock_guard<std::mutex> lock(data_mutex);
//         // Only start measuring time elapsed once we have received the
//         // first piece of data
//         if (first_data) {
//             first_data = false;
//             last_print = std::chrono::system_clock::now();
//         }

//         // if (auto fp = frame.as<rs2::pose_frame>()) {
//         //     pose_counter++;
//         // }
//         if (auto fp = frame.as<rs2::motion_frame>()) {
//             if (fp && fp.get_profile().stream_type() == RS2_STREAM_GYRO && 
//                 fp.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
//             {
//                 // Get the timestamp of the current frame
//                 double ts = fp.get_timestamp();
//                 // Get gyro measurements
//                 rs2_vector gyro_data = fp.get_motion_data();
//                 // std::cout << "GYRO: " << gyro_data.x << ", " << gyro_data.y << ", " << gyro_data.z << std::endl;
                
//                 // std::cout << "GYRO TIMESTAMP: " << ts << std::endl;
//                 gyro_counter++;
//             }
//             // If casting succeeded and the arrived frame is from accelerometer stream
//             if (fp && fp.get_profile().stream_type() == RS2_STREAM_ACCEL && 
//                 fp.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
//             {
//                 double ts_accel = fp.get_timestamp();
//                 // Get accelerometer measurements
//                 rs2_vector accel_data = fp.get_motion_data();
//                 // std::cout << "ACCEL: " << accel_data.x << ", " << accel_data.y << ", " << accel_data.z << std::endl;
               
//                 // std::cout << "ACCEL TIMESTAMP: " << ts_accel << std::endl;
//                 accel_counter++;
//             }

//             // if (fp && fp.get_profile().stream_type() == RS2_STREAM_GYRO || 
//             //     fp.get_profile().stream_type() == RS2_STREAM_ACCEL &&
//             //     fp.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
//             // {
//             //     // Get the timestamp of the current frame
//             //     double ts = fp.get_timestamp();
//             //     // Get gyro measurements
//             //     rs2_vector motion_data = fp.get_motion_data();
//             //     // std::cout << "GYRO: " << gyro_data.x << ", " << gyro_data.y << ", " << gyro_data.z << std::endl;
//             //     std::cout << "MOTION TIMESTAMP: " << ts << std::endl;
//             //     motion_counter++;
//             // }
//         }        
//         else if (auto fs = frame.as<rs2::frameset>()) {
//             double ts_frame = fs.get_timestamp();
//             if(current_timestamp != ts_frame)
//             {
//                 current_timestamp = ts_frame;

//                 // std::cout << "FRAME TIMESTAMP: " << ts_frame << std::endl;

//                 new_frame = true;

//                 rs2::video_frame fisheye_frame_left = fs.get_fisheye_frame(1);
//                 imLeft = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)fisheye_frame_left.get_data());
//                 // cv::imshow("disparity", imLeft);
//                 // if (cv::waitKey(1) == 'q') {
//                 //     exit(1);
//                 // }
//                 frame_counter++;
//             }
//         }

//         // // Print the approximate pose and image rates once per second
//         // auto now = std::chrono::system_clock::now();
//         // if (now - last_print >= std::chrono::seconds(1)) {
//         //     // std::cout << "\r" << std::setprecision(0) << std::fixed 
//         //     //         << "Accel rate:  "  << accel_counter << " "
//         //     //         << "Gyro  rate:  "  << gyro_counter << " "
//         //     //         << "Motion rate: "  << motion_counter << " "
//         //     //         << "Image rate:  " << frame_counter << std::flush;
//         //     accel_counter = 0;
//         //     gyro_counter  = 0;
//         //     motion_counter = 0;
//         //     frame_counter = 0;
//         //     last_print = now;
//         // }
//     };

//     // Start streaming through the callback
//     rs2::pipeline_profile profiles = pipe_t265.start(cfg, callback);

//     // Sleep this thread until we are done
//     while(true) {
//         // std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         // std::cout << "\r" << &new_frame << std::endl;
//         if(new_frame)
//         {
//             std::cout << "\r" << "WHILE LOOP" << std::endl;

//             new_frame = false;

//             cv::imshow("CAMERA", imLeft);
//             if (cv::waitKey(1) == 'q') {
//                 exit(1);
//             }
//         }        
//     }


// 	// RUN THREAD.
//     /*
// 	auto thread1 = std::thread(&thread_IMU,  std::ref(pipe_t265));
// 	auto thread2 = std::thread(&thread_MAIN, std::ref(pipe_t265));
	
// 	thread1.join();
//     thread2.join();*/

//     return 0;
// }



// /**
//  * @file main.cpp
//  * @author Duncan Hamill (duncanrhamill@googlemail.com)
//  * @brief OAK-D/ORB_SLAM3 Experiments main file.
//  * 
//  * @version 0.1
//  * @date 2021-01-13
//  * 
//  * @copyright Copyright (c) Duncan Hamill 2021
//  */

// /* -------------------------------------------------------------------------
//  * INCLUDES
//  * ------------------------------------------------------------------------- */

// #include <iostream>

// #include <opencv2/opencv.hpp>

// #include "System.h"

// /* NEW */
// #include <librealsense2/rs.hpp>
// #include <iomanip>
// #include <librealsense2/rsutil.h>
// #include <array>
// #include <cmath>
// #include <iostream>

// using namespace cv;

// /* -------------------------------------------------------------------------
//  * CONSTANTS
//  * ------------------------------------------------------------------------- */

// // WLS parameters, taken from the OpenCV WLS filter docs recommended values.
// #define WLS_LAMBDA (8000)
// #define WLS_SIGMA (1.0)

// /* -------------------------------------------------------------------------
//  * STRUCTS
//  * ------------------------------------------------------------------------- */

// // A simple pose structure containing position vector and rotation matrix.
// typedef struct _Pose {
//     cv::Mat position;
//     cv::Mat rotation;
// } Pose;

// /* -------------------------------------------------------------------------
//  * MAIN
//  * ------------------------------------------------------------------------- */

// int main(int argc, char *argv[]) {

//     std::cout << "OAK-D/ORB_SLAM3 Experiment" << std::endl;
    
//     cv::Mat R1, R2, P1, P2, Q;

//     // Create the SLAM system. First argument is path to the ORB_SLAM3 vocab
//     // file. The second is the path to the settings file for this particular
//     // camera setup. The values in this file were taken from what's printed out
//     // of `depthai_demo.py`.
//     //
//     // While the OAK-D does have an IMU we can't use it right now, but support
//     // is coming soon! For now just use stereo mode for SLAM, which isn't as
//     // accurate as IMU_STEREO.
//     //
//     // The last input tells the system to display it's UI.
//     ORB_SLAM3::System SLAM(
//         "Vocabulary/ORBvoc.txt",
//         "Examples/Stereo/EuRoCT265.yaml",
//         ORB_SLAM3::System::STEREO, 
//         true
//     );

//     // We also want somewhere to store our pose data
//     Pose pose;

//     // The time of each frame is required for SLAM, so we take an epoch time
//     // (i.e. our start time) now
//     auto slam_epoch = std::chrono::steady_clock::now();

//     // CAMERA.
//     int width = 848;
// 	int height = 800;
// 	int fps = 30;

//     // CAMERA T265
// 	rs2::config config_t265;
// 	std::string serial_number_t265 = "925122110153";
// 	config_t265.enable_device(serial_number_t265); // Serial number of t265.
//     config_t265.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
//     config_t265.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

//    	rs2::pipeline pipe_t265;
//     pipe_t265.start(config_t265);

//     cv::Mat imLeft; 
//     cv::Mat imRight; 

//     // Now for the main loop
//     while (1) {

// 		// rs2::frameset frameset = pipeline.wait_for_frames();

// 		// rs2::video_frame ir_frame_left = frameset.get_infrared_frame(1);
// 		// rs2::video_frame ir_frame_right = frameset.get_infrared_frame(2);

//         // imLeft = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
//         // imRight = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());
//         rs2::frameset frameset = pipe_t265.wait_for_frames();

// 		rs2::video_frame fisheye_frame_left = frameset.get_fisheye_frame(1);
// 		rs2::video_frame fisheye_frame_right = frameset.get_fisheye_frame(2);

//         imLeft = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)fisheye_frame_left.get_data());
//         imRight = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)fisheye_frame_right.get_data());

//         // Get the time between the epoch and now, allowing us to get a
//         // timestamp (in seconds) to pass into the slam system.
//         auto elapsed_time = std::chrono::steady_clock::now() - slam_epoch;
//         double frame_timestamp_s = elapsed_time.count() / 1000000000.0;

//         std::cout << std::setprecision(4) << "TimeStamp (sec): " << frame_timestamp_s << ": " << std::endl;

//         // Into the SLAM system. This produces a matrix with
//         // the pose information of the camera.
//         cv::Mat raw_pose = SLAM.TrackStereo(
//             imLeft,
//             imRight,
//             frame_timestamp_s  // rs2::frameset frameset = pipe.wait_for_frames();
//         );

//         // The output pose may be empty if the system was unable to track the
//         // movement, so only get position and rotation if pose isn't empty. We
//         // also put this info an a localisation fix available flag for later
//         // use. 
//         bool loc_fix_available = !raw_pose.empty();
//         if (loc_fix_available) {
//             // The pose matrix is a 4x4 extrinsic matrix, with the form:
//             // [R_3x3 T_3x1; [0 0 0 1]], we can find the camera position with 
//             // C = -R'T (R' = R transpose).
//             pose.rotation = raw_pose(cv::Rect(0, 0, 3, 3));
//             cv::Mat T = raw_pose(cv::Rect(3, 0, 1, 3));
//             pose.position = -pose.rotation.t()*T;

//             // Print the updated position, but transpose it so that instead of
//             // a column vector we have a row vector, which is easier to read.
//             // std::cout << 
//             //     "position: " << 
//             //     pose.position.t() << 
//             //     std::endl;
//         }
//         else {
//             // If we didn't get a pose update log it.
//             std::cout << "no pose update" << std::endl;
//         }

//         // // Apply a colormap to the filtered disparity map, but don't normalise
//         // // it. Normalising the map will mean that the color doesn't correspond
//         // // directly with disparity.
//         // cv::Mat colour_disp;
//         // /*cv::applyColorMap(imLeft, colour_disp, cv::COLORMAP_JET);*/
        
//         // cv::imshow("disparity", imLeft);

//         // See if q pressed, if so quit
//         if (cv::waitKey(1) == 'q') {
//             break;
//         }
//     }

//     // Stop all SLAM threads
//     SLAM.Shutdown();

//     return EXIT_SUCCESS;
// }