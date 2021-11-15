//
//  Experiments.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 21.06.2021.
//

#include "Experiments.hpp"

int runExperiments() {
    
//    string INPUT = "/Users/mariuspenteliuc/Assets/PhD/ExperimentalSets/OutMovement_1/frames";
    string ROOT = "/Users/mariuspenteliuc/Assets/PhD/ThesisKeynote";
    string INPUT = ROOT + "/frames";
    string OUTPUT = ROOT + "";
    cout << ">>> Output folder: " << OUTPUT << endl;
    cv::String pathToFlows(OUTPUT + "/flows");
    if (!std::__fs::filesystem::exists(pathToFlows)) {
        cout << "Computing optical flow..." << endl;
        OpticalFlowService::computeFlowForImages(INPUT, OUTPUT, "jpg", true, true, false);
        cout << "[done]" << endl;
    }
    
    cv::String pathToAverage(OUTPUT + "/average_flow.npy");
    Mat averageFlowMap;
    if (!std::__fs::filesystem::exists(pathToAverage)) {
        cout << "Computing Average Wind Map..." << endl;
        averageFlowMap = OpticalFlowService::averageFlows(pathToFlows);
        FileHelper::writeFile(pathToAverage, averageFlowMap);
        cout << "[done]" << endl;
    } else {
        averageFlowMap = FileHelper::readFile(OUTPUT + "/average_flow.npy");
    }
    
    if (!std::__fs::filesystem::exists(OUTPUT + "/average_flow.jpg")) {
        Mat overlayedImage = OpticalFlowService::overlayFlowLines(averageFlowMap);
        FileHelper::writeFile(OUTPUT + "/average_flow.jpg", overlayedImage);
    }
    
    if (!std::__fs::filesystem::exists(OUTPUT + "/simulated_clouds")) {
        int boidCount = 10000;
        int simulationRuns = 150;
        bool honorMask = true;
        Mat initialMask = FileHelper::readFile(OUTPUT + "/flows/flow_00002.npy");
        Scene scene = Scene(averageFlowMap.cols, averageFlowMap.rows);
        if (honorMask) {
            cout << "Populating scene with boids according to mask..." << endl;
            for (int i = 0; i < scene.getWidth(); i+=5) {
                for (int j = 0; j < scene.getHeight(); j+=5) {
                    Point2f& fxy = initialMask.at<Point2f>(j, i);
                    if (abs(fxy.x) > 1.0 || abs(fxy.y) > 1.0) { // a
//                    if (fxy.x != 0 || fxy.y != 0) { // b  ← can't use this because OF creates lines of close to zero values
                        scene.addBoid(i, j, 1);
                    }
                }
            }
        } else {
            cout << "Populating scene with " << boidCount << " boids..." << endl;
            for (int i = 0; i < boidCount; ++i) {
                scene.addRandomBoid();
            }
        }
        cout << "[done]" << endl;
        
        cout << "Starting Simulation for " << simulationRuns << " runs..." << endl;
        scene.updateWindMap(averageFlowMap);
        scene.startSimulation(OUTPUT);
        scene.runSimulation(simulationRuns);
//        TODO: update wind map to continue wind movement.
//        TODO: cloud points near the edge are slowing down probably because of averaging with neighboring windless environment.
        cout << "[done]" << endl;
    }
    //    int configuration = 15;
    //    for (int configuration = 1; configuration < 12; ++configuration) {
    //        cout << "Running experiment " << configuration << endl;
    //        runExperiments(configuration);
    //    }
    //    runExperiments(5);
    
    //    computeError("/Volumes/Transfers/Experiments/Configuration_d", "/Volumes/Transfers/debug_out/flows");
    //    computeError("/Volumes/Transfers/Experiments/Configuration_d", "/Volumes/Transfers/Experiments/Configuration_d");
    
    return 0;
}
