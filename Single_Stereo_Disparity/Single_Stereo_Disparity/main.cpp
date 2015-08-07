//
//  main.cpp
//  ReadAndDisplayImage
//
//  Created by Michael Schwendeman on 8/6/15.
//  Copyright (c) 2015 APL. All rights reserved.
//

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, const char** argv )
{
    
    // LOAD AND DISPLAY RAW IMAGES
    Mat imgLeft = imread("/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/28Dec2014/1848UTC/flea35_2014-12-28-184907-0011.pgm", CV_LOAD_IMAGE_UNCHANGED); //left image
    Mat imgRight = imread("/Volumes/Data/PAPA/TGTcruise2015/StereoSystem/28Dec2014/1848UTC/flea34_2014-12-28-184909-0000.pgm", CV_LOAD_IMAGE_UNCHANGED); //right image
    
    
    if (imgLeft.empty() || imgRight.empty()) //check whether the images is loaded or not
    {
        cout << "Error : One or both images couldn't load!" << endl;
        return -1;
    }

    namedWindow("Raw Left Image", CV_WINDOW_AUTOSIZE); //create window for left image
    imshow("Raw Left Image", imgLeft); //display the left image
    namedWindow("Raw Right Image", CV_WINDOW_AUTOSIZE); //create window for right image
    imshow("Raw Right Image", imgRight); //display the right image
    
    // MANUAL INPUT CAMERA PARAMETERS
    Mat cameraMatLeft = (Mat_<double>(3,3) << 1923.188639424867, 0, 522.300865840401,
                         0, 1923.103223246226, 390.110176489687,
                         0, 0, 1);
    Mat cameraMatRight= (Mat_<double>(3,3) << 1929.007304383480, 0, 525.540100244319,
                         0, 1931.105500487271, 404.018254429260,
                         0, 0, 1);
    Mat distCoeffsLeft = (Mat_<double>(4,1) << -0.279993870204181, 0.444984775863282, 0, 0);
    Mat distCoeffsRight = (Mat_<double>(4,1) << -0.268444068184339, 0.679917196937752, 0, 0);
    Mat rotationLeftRight = (Mat_<double>(3,3) << 0.999757252231485,  0.021913494024872, 0.002288097500478,
                             -0.021943962745142,  0.999657361545489, 0.014269618318320,
                             -0.001974616314531, -0.014316364326623, 0.999895565848193);
    Mat translationLeftRight = (Mat_<double>(3,1) << -1986.639953355410,  -7.743896974344,   7.026833161269);
    
    // UNDISTORT
    Mat imgLeftUndistort, imgRightUndistort;
    undistort(imgLeft, imgLeftUndistort,cameraMatLeft, distCoeffsLeft);
    undistort(imgRight, imgRightUndistort,cameraMatRight, distCoeffsRight);
    
    namedWindow("Undistorted Left Image", CV_WINDOW_AUTOSIZE); //create window for left image
    imshow("Undistorted Left Image", imgLeftUndistort); //display the left image
    namedWindow("Undistorted Right Image", CV_WINDOW_AUTOSIZE); //create window for right image
    imshow("Undistorted Right Image", imgRightUndistort); //display the right image
    
    // RECTIFY
    Mat R1, R2, P1, P2, Q;
    Rect roiLeft, roiRight;
    Size imageSize = imgLeftUndistort.size();
    stereoRectify(cameraMatLeft, distCoeffsLeft,
                  cameraMatRight, distCoeffsRight,
                  imageSize, rotationLeftRight, translationLeftRight, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &roiLeft, &roiRight);

    Mat rmap[2][2];
    initUndistortRectifyMap(cameraMatLeft, distCoeffsLeft, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatRight, distCoeffsRight, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
    Mat imgLeftRectified, imgRightRectified;
    
    remap(imgLeft, imgLeftRectified, rmap[0][0], rmap[0][1], INTER_LINEAR);
    remap(imgRight, imgRightRectified, rmap[1][0], rmap[1][1], INTER_LINEAR);
    
    namedWindow("Rectified Left Image", CV_WINDOW_AUTOSIZE); //create window for left image
    imshow("Rectified Left Image", imgLeftRectified); //display the left image
    namedWindow("Rectified Right Image", CV_WINDOW_AUTOSIZE); //create window for right image
    imshow("Rectified Right Image", imgRightRectified); //display the right image
    
    // STEREO MATCHING
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,112,7);
    Mat disparity, disparityShow;
    sgbm->compute(imgLeftRectified, imgRightRectified, disparity);
    namedWindow("Disparity Map", CV_WINDOW_AUTOSIZE); //create window for disparity
    disparity.convertTo(disparityShow, CV_8U, 255/(96*16.));
    imshow("Disparity Map",disparityShow);
    
    // CLOSE WINDOWS
    waitKey(0); //wait infinite time for a keypress
    
    destroyWindow("Raw Left Image"); //destroy the windows
    destroyWindow("Raw Right Image");
    destroyWindow("Undistorted Right Image");
    destroyWindow("Undistorted Left Image");
    destroyWindow("Rectified Right Image");
    destroyWindow("Rectified Left Image");
    destroyWindow("Disparity Map");
    return 0;
}
