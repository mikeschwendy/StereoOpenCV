//
//  main.cpp
//  ReadAndDisplayImage
//
//  Created by Michael Schwendeman on 3/24/14.
//  Copyright (c) 2014 APL. All rights reserved.
//

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, const char** argv )
{
    Mat img = imread("/Users/mike/Documents/UW/Research/Miscellaneous/OpenCvTest/TestAug2015/TestOpenCvUsingXcode/TestOpenCvUsingXcode/MyPic.jpg", CV_LOAD_IMAGE_UNCHANGED); //read the image data in the file "MyPic.JPG" and store it in 'img'
    
    if (img.empty()) //check whether the image is loaded or not
    {
        cout << "Error : Image cannot be loaded..!!" << endl;
        //system("pause"); //wait for a key press
        return -1;
    }
    
    namedWindow("MyWindow", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"
    imshow("MyWindow", img); //display the image which is stored in the 'img' in the "MyWindow" window
    
    waitKey(0); //wait infinite time for a keypress
    
    destroyWindow("MyWindow"); //destroy the window with the name, "MyWindow"
    
    return 0;
}
