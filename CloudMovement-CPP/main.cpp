//
//  main.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 11.11.2020.
//

#include <iostream>
#include "OpticalFlowService.hpp"

using namespace std;

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
