//
//  OpticalFlowService.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 12.11.2020.
//

#include "OpticalFlowService.hpp"
#include <opencv2/opencv.hpp>

using namespace std;

string OpticalFlowService::getOpenCVVersion() {
    return CV_VERSION;
}
