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
    srand (static_cast <unsigned> (time(0)));
    std::cout << "OpenCV version: " << OpticalFlowService::getOpenCVVersion() << endl;

    return 0;
}
