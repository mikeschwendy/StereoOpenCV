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

void computeDisparity(Mat img1, Mat img2, Mat cameraMat1, Mat cameraMat2, Mat distCoeffs1, Mat distCoeffs2, Mat rotMat, Mat transVect, Mat& disp, Rect& roiLeft, Mat& Q)
{
    // RECTIFY
    Mat R1, R2, P1, P2;
    Rect roiRight;
    Size imageSize = img1.size();
    stereoRectify(cameraMat1, distCoeffs1,cameraMat2, distCoeffs2,
                  imageSize, rotMat, transVect, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &roiLeft, &roiRight);
    
    Mat rmap[2][2];
    initUndistortRectifyMap(cameraMat1, distCoeffs1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMat2, distCoeffs2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
    Mat rimg1, rimg2;
    
    remap(img1, rimg1, rmap[0][0], rmap[0][1], INTER_LINEAR);
    remap(img2, rimg2, rmap[1][0], rmap[1][1], INTER_LINEAR);
    
    // STEREO MATCHING
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,112,7);
    sgbm->compute(rimg1, rimg2, disp);
    
}

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
    
    // COMPUTE DISPARITY
    Mat disparity, Q;
    Rect roi;
    computeDisparity(imgLeft, imgRight, cameraMatLeft, cameraMatRight, distCoeffsLeft, distCoeffsRight, rotationLeftRight, translationLeftRight, disparity, roi, Q);
    
    // Save disparity as image and yml
    Mat dispSave;
    disparity.convertTo(dispSave, CV_8U, 255/(112*16.));
    imwrite("/Users/mike/Desktop/StereoOpenCV/Single_Stereo_Disparity/flea35_2014-12-28-184907-0011.pgm", dispSave);
    
    FileStorage fs_disp("/Users/mike/Desktop/StereoOpenCV/Single_Stereo_Disparity/flea35_2014-12-28-184907-0011_disp.yml", FileStorage::WRITE);
    fs_disp << "disparity" << disparity;
    fs_disp.release();

    // PROJECT TO 3D
    Mat image3;
    Mat xyz[3];
    reprojectImageTo3D(disparity, image3, Q, true, CV_32F);
    split(image3,xyz);
    Mat z = xyz[2];
    Mat zShow, dispShow;
    z.convertTo(zShow, CV_8U, 255/(10000.));
    disparity.convertTo(dispShow, CV_8U, 255/(112*16.));
 
    FileStorage fs_xyz("/Users/mike/Desktop/StereoOpenCV/Single_Stereo_Disparity/flea35_2014-12-28-184907-0011_xyz.yml", FileStorage::WRITE);
    fs_xyz << "z" << z;
    fs_xyz.release();
    
    // Display images
    namedWindow("Left Image", CV_WINDOW_AUTOSIZE); //create window for left image
    imshow("Left Image", imgLeft); //display the left image
    namedWindow("Right Image", CV_WINDOW_AUTOSIZE); //create window for right image
    imshow("Right Image", imgRight); //display the right image
    namedWindow("Disparity Map", CV_WINDOW_AUTOSIZE); //create window for disparity
    imshow("Disparity Map", dispShow);
    namedWindow("Z Map", CV_WINDOW_AUTOSIZE); //create window for disparity
    imshow("Z Map",zShow);
    
    // CLOSE WINDOWS
    waitKey(0); //wait infinite time for a keypress
    destroyWindow("Right Image");
    destroyWindow("Left Image");
    destroyWindow("Disparity Map");
    destroyWindow("Z Map");
    
    return 0;
}
