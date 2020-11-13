//
//  main.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 11.11.2020.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "OpticalFlowService.hpp"
#include "ImageHelper.hpp"
#include <cstdlib>
#include <ctime>
#include "Scene.hpp"

using namespace std;
using namespace cv;

const static string DEBUG_IN = "/Users/mariuspenteliuc/Assets/PhD/debug/debug_in";
const static string DEBUG_OUT = "/Users/mariuspenteliuc/Assets/PhD/debug/debug_out";

int main(int argc, const char * argv[]) {
    std::cout << "OpenCV version: " << OpticalFlowService::getOpenCVVersion() << endl;

    return 0;
}



//    Mat img = imread("/Users/mariuspenteliuc/Desktop/image.png"); //Change the image path here.
//        if (img.data == 0) {
//            cerr << "Image not found!" << endl;
//            return -1;
//        }
//    namedWindow("image", WINDOW_AUTOSIZE);
//        imshow("image", img);
//        waitKey();
