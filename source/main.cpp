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

    OpticalFlowService ofService;
//    ofService.computeFlowForImages(DEBUG_IN, DEBUG_OUT, "jpg", true, true, false);
    cv::String pathToFlows(DEBUG_OUT + "/flows");

//    Mat img = imread("/Users/mariuspenteliuc/Assets/PhD/debug/debug_in/GERVISIR2017-10-01-001418.jpg");
//    Mat imgGray;
//    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
//    imgGray = ofService.overlayFlowLines(average, imgGray);
//    namedWindow("OpticalFlow", WINDOW_AUTOSIZE);
//    imshow("OpticalFlow", imgGray);
//    waitKey();

    Scene scene = Scene(1920, 1080);
    for (int i = 0; i < 10000; ++i) {
        scene.addRandomBoid();
    }
    cout << "Computing Wind Map..." << endl;
    Mat average = OpticalFlowService::averageFlows(pathToFlows);
    scene.updateWindMap(average);

    cout << "Starting Simulation..." << endl;
    scene.startSimulation();
//    cout << "Scene has " << scene.getBoidsCount() << " objects." << endl;
//    std::vector<Boid> allBoids = scene.getAllBoids();
//
//    std::vector<Boid> neighbors = scene.getNeighbors(allBoids[0], 10);
//    cout << "Found " << neighbors.size() << " neighbors." << endl;
//
    for (int i = 0; i < 500; ++i) {
        scene.update();
        if (i%100 == 0) cout << i << " images so far..." << endl;
    }
//
//    neighbors = scene.getNeighbors(allBoids[0], 10);
//    cout << "Found " << neighbors.size() << " neighbors." << endl;

    return 0;
}

//TODO: create demo functions to demonstrate functionality 😅
