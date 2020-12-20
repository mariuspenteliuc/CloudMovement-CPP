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
#include "Helpers/ImageHelper.hpp"
#include <cstdlib>
#include <ctime>
#include <omp.h>
#include "Boids/Scene.hpp"

using namespace std;
using namespace cv;

const static string DEBUG_IN = "/Users/macbookair/Documents/uvt/disertatie/Assets/new";
const static string DEBUG_OUT = "/Users/macbookair/Documents/uvt/disertatie/Assests/new_out";

int main(int argc, const char * argv[]) {

    int numberOfThreads = argv[1] ? atoi(argv[1]) : 1;
    srand (static_cast <unsigned> (time(0)));
    // std::cout << "OpenCV version: " << OpticalFlowService::getOpenCVVersion() << endl;

    OpticalFlowService ofService;
    ofService.computeFlowForImages(DEBUG_IN, DEBUG_OUT, "jpg", true, true, false, numberOfThreads);

    //TODO: add function to load and overlay flows from disk.

    Scene scene = Scene(1920, 1080);

    omp_set_num_threads(numberOfThreads); // set number of threads in "parallel" blocks
    #pragma omp parallel for
    for (int i = 0; i < 10000; ++i) {
        scene.addRandomBoid();
    }
    scene.startSimulation();
    cout << "Scene has " << scene.getBoidsCount() << " objects." << endl;
    std::vector<Boid> allBoids = scene.getAllBoids();

//    std::vector<Boid> neighbors = scene.getNeighbors(allBoids[0], 10);
//    cout << "Found " << neighbors.size() << " neighbors." << endl;

//     for (int i = 0; i < 50; ++i) {
//         scene.update();
//     }

//    neighbors = scene.getNeighbors(allBoids[0], 10);
//    cout << "Found " << neighbors.size() << " neighbors." << endl;

    return 0;
}
