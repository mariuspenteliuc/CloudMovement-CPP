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

int runExperiments() {
    cout << "Running Experiments..." << endl;
    
    cout << "One Day Window test" << endl;
    
//    Mat average = OpticalFlowService::averageFlows("/Volumes/Transfers/Experiments/day_window_1/test_1/historic_flows");
//
//    FileHelper::writeFile("/Volumes/Transfers/Experiments/day_window_1/test_1/averages/average_wind_map.npy", average);

//    Mat average = FileHelper::readFile("/Volumes/Transfers/Experiments/day_window_1/test_1/averages/average_wind_map.npy");
    Mat average = FileHelper::readFile("/Users/mariuspenteliuc/Assets/PhD/debug/debug_out/visualization/averageFlow.jpg");
//
//    cout << "Populating scene with 5000 boids..." << endl;
    Scene scene = Scene(1920, 1080);
//    for (int i = 0; i < 5000; ++i) {
//        scene.addRandomBoid();
//    }
//
//    cout << "Starting Simulation..." << endl;
    scene.updateWindMap(average);
    scene.updateWindMapUsingBoids();
//    scene.startSimulation();
//    scene.runSimulation(50);
    
//    Mat blankImage = FileHelper::readFile("/Volumes/Transfers/Experiments/day_window_1/test_1/blank.jpg");
//    Mat grayImage;// = FileHelper::convertToGray(im2);
//
//    vector<cv::String> fileNames;
//    cv::String path("/Volumes/Transfers/Experiments/day_window_1/test_1/historic_flows/*.npy");
//    cv::glob(path,fileNames,false);
//    for (size_t k=0; k<fileNames.size(); ++k) {
//        cv::Mat flow = FileHelper::readFile(fileNames[k]);
//        cv::cvtColor(blankImage, grayImage, cv::COLOR_BGR2GRAY);
//        Mat overlayedImage = OpticalFlowService::overlayFlowLines(flow, grayImage);
//        FileHelper::writeFile("/Volumes/Transfers/Experiments/day_window_1/test_1/simulated_wind_map/wind_map_" + string(5 - to_string(k).length(), '0') + to_string(k) + ".jpg", overlayedImage);
//    }

    return 0;
}

int main(int argc, const char * argv[]) {
    srand (static_cast <unsigned> (time(0)));

//    cout << "Computing optical flow..." << endl;
//    OpticalFlowService ofService;
//    OpticalFlowService::computeFlowForImages(DEBUG_IN, DEBUG_OUT, "jpg", true, true, false);
//    cv::String pathToFlows(DEBUG_OUT + "/flows");

//    cout << "Computing Wind Map..." << endl;
//    Mat average = OpticalFlowService::averageFlows(pathToFlows);
//    FileHelper::writeFile("/Users/mariuspenteliuc/Assets/PhD/debug/debug_out/flows/average_flow.npy", average);

//    cout << "Populating scene with 10 000 boids..." << endl;
//    Scene scene = Scene(1920, 1080);
//    for (int i = 0; i < 10000; ++i) {
//        scene.addRandomBoid();
//    }
//
//    cout << "Starting Simulation..." << endl;
//    scene.updateWindMap(average);
//    scene.startSimulation();
//    scene.runSimulation(50);
    
    
    runExperiments();

    return 0;
}
