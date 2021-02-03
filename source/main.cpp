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
#include "Helpers/FileHelper.hpp"
#include "Helpers/MathHelper.hpp"
#include <cstdlib>
#include <ctime>
#include <omp.h>
#include "Boids/Scene.hpp"

using namespace std;
using namespace cv;

const static string DEBUG_IN = "/Users/macbookair/Documents/uvt/disertatie/Assets/debug_in";
const static string DEBUG_OUT = "/Users/macbookair/Documents/uvt/disertatie/Assests/new_out";

int runExperiments(int CONFIG = 0) {
//    FileHelper::writeFile("/Volumes/Transfers/Experiments/day_window_1/test_1/averages/average_wind_map.npy", average);

//    Mat flow = FileHelper::readFile("/Users/mariuspenteliuc/Assets/PhD/debug/debug_out/flows/flow_00001.npy");
    vector<bool> updateWindMapUsingBoids;
    updateWindMapUsingBoids.push_back(false);
    updateWindMapUsingBoids.push_back(true);
    updateWindMapUsingBoids.push_back(false);
    updateWindMapUsingBoids.push_back(true);
    updateWindMapUsingBoids.push_back(false);
    updateWindMapUsingBoids.push_back(true);
    updateWindMapUsingBoids.push_back(false);
    updateWindMapUsingBoids.push_back(true);
    updateWindMapUsingBoids.push_back(false);
    updateWindMapUsingBoids.push_back(true);
    updateWindMapUsingBoids.push_back(false);
    updateWindMapUsingBoids.push_back(true);
    vector<size_t> startIndex;
    startIndex.push_back(47);
    startIndex.push_back(0);
    startIndex.push_back(100);
    startIndex.push_back(0);
    startIndex.push_back(0);
    startIndex.push_back(0);
    startIndex.push_back(0);
    startIndex.push_back(0);
    startIndex.push_back(0);
    startIndex.push_back(0);
    startIndex.push_back(0);
    vector<int> numberOfFlows;
    numberOfFlows.push_back(48);
    numberOfFlows.push_back(48);
    numberOfFlows.push_back(48);
    numberOfFlows.push_back(48);
    numberOfFlows.push_back(336);
    numberOfFlows.push_back(336);
    numberOfFlows.push_back(336);
    numberOfFlows.push_back(336);
    numberOfFlows.push_back(1460);
    numberOfFlows.push_back(1460);
    numberOfFlows.push_back(1460);
    numberOfFlows.push_back(1460);
    vector<int> neighborhoodRadius;
    neighborhoodRadius.push_back(0);
    neighborhoodRadius.push_back(5);
    neighborhoodRadius.push_back(11);
    neighborhoodRadius.push_back(31);
    neighborhoodRadius.push_back(0);
    neighborhoodRadius.push_back(5);
    neighborhoodRadius.push_back(11);
    neighborhoodRadius.push_back(31);
    neighborhoodRadius.push_back(0);
    neighborhoodRadius.push_back(5);
    neighborhoodRadius.push_back(11);
    neighborhoodRadius.push_back(31);
    vector<String> outputFolders;
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_0");
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_1");
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_2");
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_3");
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_4");
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_5");
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_6");
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_7");
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_8");
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_9");
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_10");
    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_11");

    vector<string> fileNames;
    glob("/Volumes/Transfers/debug_out/flows/*.npy", fileNames, false);

//    Mat flow = OpticalFlowService::averageFlows(outputFolders[CONFIG] + "/historic_flows", startIndex[CONFIG], numberOfFlows[CONFIG]);
    Mat flow = FileHelper::readFile(outputFolders[CONFIG] + "/averages/average_flow.npy");
    if (flow.empty()) {
        flow = OpticalFlowService::averageFlows("/Volumes/Transfers/debug_out/flows", startIndex[CONFIG], numberOfFlows[CONFIG]);
        FileHelper::writeFile(outputFolders[CONFIG] + "/averages/average_flow.npy", flow);
    }

    cout << "averaged " << numberOfFlows[CONFIG] << " flows." << endl;

    Scene scene = Scene(1920, 1080);
    scene.updateWindMap(flow);

    Mat initialMask = FileHelper::readFile(fileNames[startIndex[CONFIG]]);
    for(int row = 0; row < initialMask.rows; ++row) {
        for(int col = 0; col < initialMask.cols; ++col) {
            Point2f &fxy = initialMask.at<Point2f>(row, col);
            if (round(fxy.x) > 0 || round(fxy.y) > 0) {
                for (int k = rand() %100; k > 98; --k) {
                    scene.addBoid(col, row, 2);
                }
            }
        }
    }

    scene.startSimulation(outputFolders[CONFIG], startIndex[CONFIG]);
    if (neighborhoodRadius[CONFIG] == 0) {
        scene.runSimulation(336);
    } else {
        Mat blankImage = FileHelper::readFile("/Volumes/Transfers/Experiments/blank.jpg");
        Mat grayImage;
        cv::cvtColor(blankImage, grayImage, cv::COLOR_BGR2GRAY);
        Mat overlayedImage = OpticalFlowService::overlayFlowLines(scene.getWindMap(), grayImage);
        FileHelper::writeFile(outputFolders[CONFIG] + "/simulated_wind_map/wind_map_" + string(5 - to_string(startIndex[CONFIG]).length(), '0') + to_string(startIndex[CONFIG]) + ".jpg", overlayedImage);
        for (int k = 1; k < 336; ++k) {
            scene.runSimulation(1);
            scene.updateWindMapUsingBoids(neighborhoodRadius[CONFIG], outputFolders[CONFIG]);
            cv::cvtColor(blankImage, grayImage, cv::COLOR_BGR2GRAY);
            Mat overlayedImage = OpticalFlowService::overlayFlowLines(scene.getWindMap(), grayImage);
            FileHelper::writeFile(outputFolders[CONFIG] + "/simulated_wind_map/wind_map_" + string(5 - to_string(startIndex[CONFIG] + k).length(), '0') + to_string(startIndex[CONFIG] + k) + ".jpg", overlayedImage);
        }
    }

    return 0;
}

int main(int argc, const char * argv[]) {

    int numberOfThreads = argv[1] ? atoi(argv[1]) : 1;
    srand (static_cast <unsigned> (time(0)));
    // std::cout << "OpenCV version: " << OpticalFlowService::getOpenCVVersion() << endl;

    // cout << "Computing optical flow..." << endl;

    // OpticalFlowService ofService;
    // OpticalFlowService::computeFlowForImages(DEBUG_IN, DEBUG_OUT, "jpg", true, true, false, numberOfThreads);
    // cv::String pathToFlows(DEBUG_OUT + "/flows");

    // cout << "Computing Wind Map..." << endl;
    // Mat average = OpticalFlowService::averageFlows(pathToFlows);
    // FileHelper::writeFile("/Users/macbookair/Documents/uvt/disertatie/Assests/debug_out/flows/average_flow.npy", average);

    // cout << "Populating scene with 10 000 boids..." << endl;
    // Scene scene = Scene(1920, 1080);

    // omp_set_num_threads(numberOfThreads); // set number of threads in "parallel" blocks
    // // #pragma omp parallel for
    // for (int i = 0; i < 10000; ++i) {
    //     scene.addRandomBoid();
    // }

    // cout << "Starting Simulation..." << endl;
    // scene.updateWindMap(average);
    // scene.startSimulation();
    // scene.runSimulation(10);

    for (int configuration = 0; configuration < 12; ++configuration) {
        cout << "Running experiment " << configuration + 1 << endl;
        runExperiments(configuration);
    }

    return 0;
}
