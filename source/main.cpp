//
//  main.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 11.11.2020.
//

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "OpticalFlow/OpticalFlowService.hpp"
#include <cstdlib>
#include <ctime>
#include <omp.h>
#include "Boids/Scene.hpp"

using namespace std;
using namespace cv;

const static string DEBUG_IN = "/Users/macbookair/Documents/uvt/disertatie/Assets/debug_in";
const static string DEBUG_OUT = "/Users/macbookair/Documents/uvt/disertatie/Assests/new_out";

int main(int argc, const char * argv[]) {

    int numberOfThreads = argv[1] ? atoi(argv[1]) : 1;
    srand (static_cast <unsigned> (time(0)));
    // std::cout << "OpenCV version: " << OpticalFlowService::getOpenCVVersion() << endl;

    cout << "Computing optical flow..." << endl;

    OpticalFlowService ofService;
    OpticalFlowService::computeFlowForImages(DEBUG_IN, DEBUG_OUT, "jpg", true, true, false, numberOfThreads);
    cv::String pathToFlows(DEBUG_OUT + "/flows");

    cout << "Computing Wind Map..." << endl;
    Mat average = OpticalFlowService::averageFlows(pathToFlows);
    FileHelper::writeFile("/Users/macbookair/Documents/uvt/disertatie/Assests/debug_out/flows/average_flow.npy", average);

    cout << "Populating scene with 10 000 boids..." << endl;
    Scene scene = Scene(1920, 1080);

    omp_set_num_threads(numberOfThreads); // set number of threads in "parallel" blocks
    // #pragma omp parallel for
    for (int i = 0; i < 10000; ++i) {
        scene.addRandomBoid();
    }

    cout << "Starting Simulation..." << endl;
    scene.updateWindMap(average);
    scene.startSimulation();
    scene.runSimulation(10);

    return 0;
}
