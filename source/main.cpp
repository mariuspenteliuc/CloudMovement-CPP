//
//  main.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 11.11.2020.
//

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "OpticalFlowService.hpp"
#include "FileHelper.hpp"
#include <cstdlib>
#include <ctime>
#include "Scene.hpp"

using namespace std;
using namespace cv;

const static string DEBUG_IN = "/Users/mariuspenteliuc/Assets/PhD/debug/debug_in";
const static string DEBUG_OUT = "/Users/mariuspenteliuc/Assets/PhD/debug/debug_out";

int main(int argc, const char * argv[]) {
    srand (static_cast <unsigned> (time(0)));

    cout << "Computing optical flow..." << endl;
//    OpticalFlowService ofService;
    OpticalFlowService::computeFlowForImages(DEBUG_IN, DEBUG_OUT, "jpg", true, true, false);
    cv::String pathToFlows(DEBUG_OUT + "/flows");

    cout << "Computing Wind Map..." << endl;
    Mat average = OpticalFlowService::averageFlows(pathToFlows);
    FileHelper::writeFile("/Users/mariuspenteliuc/Assets/PhD/debug/debug_out/flows/average_flow.npy", average);

    cout << "Populating scene with 10 000 boids..." << endl;
    Scene scene = Scene(1920, 1080);
    for (int i = 0; i < 10000; ++i) {
        scene.addRandomBoid();
    }

    cout << "Starting Simulation..." << endl;
    scene.updateWindMap(average);
    scene.startSimulation();
    scene.runSimulation(50);

    return 0;
}
