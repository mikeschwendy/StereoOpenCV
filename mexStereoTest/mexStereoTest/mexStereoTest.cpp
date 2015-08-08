//
//  main.cpp
//  mexStereoTest
//
//  Created by Michael Schwendeman on 8/8/15.
//  Copyright (c) 2015 MSS. All rights reserved.
//

#include "opencvmex.hpp"
//#include <opencv2/opencv.hpp>

#define _DO_NOT_EXPORT
#if defined(_DO_NOT_EXPORT)
#define DllExport
#else
#define DllExport __declspec(dllexport)
#endif

using namespace cv;
//using namespace std;


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
    //Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,112,7);
    //sgbm->compute(rimg1, rimg2, disp);
    StereoSGBM sgbm(0,112,7);
    sgbm.operator()(rimg1,rimg2,disp);
    
}


///////////////////////////////////////////////////////////////////////////
// Main entry point to a MEX function
///////////////////////////////////////////////////////////////////////////
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Check inputs to mex function
    // checkInputs(nrhs, prhs);
    
    // Convert mxArray inputs into OpenCV types
    Ptr<Mat> imgLeft = ocvMxArrayToImage_uint8(prhs[0], true);
    Ptr<Mat> imgRight = ocvMxArrayToImage_uint8(prhs[1], true);
    Ptr<Mat> cameraMatLeft = ocvMxArrayToMat_double(prhs[2], true);
    Ptr<Mat> cameraMatRight = ocvMxArrayToMat_double(prhs[3], true);
    Ptr<Mat> distCoeffsLeft = ocvMxArrayToMat_double(prhs[4], true);
    Ptr<Mat> distCoeffsRight = ocvMxArrayToMat_double(prhs[5], true);
    Ptr<Mat> rotationMat = ocvMxArrayToMat_double(prhs[6], true);
    Ptr<Mat> translationMat = ocvMxArrayToMat_double(prhs[7], true);
    
    Mat disparity, Q;
    Rect roi;
    computeDisparity(*imgLeft, *imgRight, *cameraMatLeft, *cameraMatRight, *distCoeffsLeft, *distCoeffsRight, *rotationMat, *translationMat, disparity, roi, Q);
    
    
    Mat disp;
    disparity.convertTo(disp,'CV_32F');
    
    // Put the data back into the output MATLAB array
    plhs[0] = ocvMxArrayFromMat_double(disp);
}