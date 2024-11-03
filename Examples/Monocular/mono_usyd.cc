/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>
// #include <opencv2/opencv.hpp>

#include"System.h"

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Step 1: Open the video file
    string strPathToVideo = string(argv[3]);
    cv::VideoCapture cap(strPathToVideo);
    
    
    
    // Step 2: Check if the video opened successfully
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video file." << std::endl;
        return -1;
    }

    int nImages = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    double t_resize = 0.f;
    double t_track = 0.f;

    // cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    // cameraMatrix.at<double>(0, 0) = 1176.931662006163;
    // cameraMatrix.at<double>(1, 1) = 1177.715660133758;
    // cameraMatrix.at<double>(0, 2) = 962.2754188883206;
    // cameraMatrix.at<double>(1, 2) = 612.7245350750861;

    // cv::Mat distanceCoeffs = cv::Mat(4, 1, CV_64F);
    // cameraMatrix.at<double>(0, 0) = -0.05400957120448697;
    // cameraMatrix.at<double>(1, 0) = -0.07842753582468161;
    // cameraMatrix.at<double>(2, 0) = 0.09596410068935728;
    // cameraMatrix.at<double>(3, 0) = -0.05152529532743679;

    // cv::Size initImageSize = cv::Size(1920, 1208);

    // cv::Mat identity_mat = cv::Mat::eye(3, 3, CV_64F);

    // cv::Mat modified_cameraMatrix;
    // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distanceCoeffs, initImageSize,
    //                                                         identity_mat, modified_cameraMatrix);

    // for(int i = 0; i < 3; i++){
    //     for(int j = 0; j < 4; j++){
    //         cout << modified_cameraMatrix.at<double>(i, j) << "\n";
    //     }
    // }
    // cv::Mat map1, map2;
    // cv::initUndistortRectifyMap(cameraMatrix,
    //                                         distanceCoeffs,
    //                                         identity_mat,
    //                                         modified_cameraMatrix,
    //                                         initImageSize,
    //                                         CV_16SC2,
    //                                         map1, map2);


    
    // Step 3: Loop to read frames
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++) {
        cout << ni << "\n";
        // Read a new frame
        cap >> im;
        auto now = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
        double tframe = duration.count();

        // Break the loop if no more frames. Should never happen.
        if (im.empty()) {
            cerr << endl << "Failed to load image at: " << ni << endl;
            return 1;
        }
        // cv::Mat im = im_dist;
        // cv::undistort(im_dist, im, cameraMatrix, distanceCoeffs);
        // cv::remap(im_dist, im, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        // cv::imshow("Remapped", im);
        // cv::waitKey(0);
        // cv::destroyAllWindows();

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe,vector<ORB_SLAM3::IMU::Point>());
        cout << "OK1\n";

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // // Wait to load the next frame
        // double T=0;
        // if(ni<nImages-1)
        //     T = vTimestamps[ni+1]-tframe;
        // else if(ni>0)
        //     T = tframe-vTimestamps[ni-1];

        // if(ttrack<T)
        //     usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();


    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}
