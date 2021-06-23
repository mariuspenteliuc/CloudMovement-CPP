//
//  main.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 11.11.2020.
//

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "OpticalFlowService.hpp"
#include "FileHelper.hpp"
#include "MathHelper.hpp"
#include <cstdlib>
#include <chrono>
#include <ctime>
#include <ctime>
#include "Scene.hpp"
#include "Experiments.hpp"
using namespace std;
using namespace cv;

const static string DEBUG_IN = "/Users/mariuspenteliuc/Assets/PhD/NewTest/debug_in";
const static string DEBUG_OUT = "/Users/mariuspenteliuc/Assets/PhD/NewTest/debug_out";

int computeNNError(String pathToNNForecasts, String pathToActuals) {
    ofstream ofs;
    ofs.open ("/Users/mariuspenteliuc/Desktop/recomputeErrors.txt", ofstream::out | ofstream::app);
    
    ofs << "step" << ", " << "mae_inside" << ", " << "mape1_inside" << ", " << "mape2_inside" << "mae_outside" << ", " << "mape1_outside" << ", " << "mape2_outside" << endl;//"forecast" << ", " << "actual" << endl;
    
    vector<string> actualsFileNames, NNforecastsFileNames;
    //    glob("/Volumes/Transfers/debug_out/flows/*.npy", fileNames, false);
    glob(pathToActuals + "/*.npy", actualsFileNames, false);
    glob(pathToNNForecasts + "/*.npy", NNforecastsFileNames, false);
    int step = 0;
    int count = 0;  // skip first actual flow
    for (int i = 0; i < 366; i++) {
        //    for (String fileName : forecastsFileNames) {
        //        Mat forecast = FileHelper::readFile(fileName);
        //        Mat actual = FileHelper::readFile(actualsFileNames[count]);
        Mat forecast = FileHelper::readFile(NNforecastsFileNames[i]);
        Mat actual = FileHelper::readFile(actualsFileNames[i]);
        
        float mae_inside = 0;
        float mape1_inside = 0;
        float mape2_inside = 0;
        float mae_outside = 0;
        float mape1_outside = 0;
        float mape2_outside = 0;
        Point2f referencePoint1 = Point2f(1,0);
        Point2f referencePoint2 = Point2f(-1,0);
        
        float percent = 0.0;
        int rows = actual.rows;
        int cols = actual.cols;
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                Point2f forecastPoint = forecast.at<Point2f>(row, col);
                Point2f realPoint = actual.at<Point2f>(row, col);
                float angle = Vector::getAngleBetween(realPoint, forecastPoint);
                float actualAngle1 = Vector::getAngleBetween(realPoint, referencePoint1);
                float forcastAngle1 = Vector::getAngleBetween(forecastPoint, referencePoint1);
                float actualAngle2 = Vector::getAngleBetween(realPoint, referencePoint2);
                float forcastAngle2 = Vector::getAngleBetween(forecastPoint, referencePoint2);
                if (!isnan(angle)) {
                    if (row >= rows*percent && row < rows-rows*percent && col >= cols*percent && col < cols-cols*percent) {
                        mae_inside += (angle / M_PI) / (rows * (1.0-2*percent) * cols * (1.0-2*percent));  //1080*(1-2*0,1)*1920*(1-2*0,1)
                        if (actualAngle1 != 0 && forcastAngle1 != 0) {
                            mape1_inside += abs((actualAngle1 - forcastAngle1) / actualAngle1)
                            / (rows * (1.0-2*percent) * cols * (1.0-2*percent));
                        }
                        if (actualAngle2 != 0 && forcastAngle2 != 0) {
                            mape2_inside += abs((actualAngle2 - forcastAngle2) / actualAngle2)
                            / (rows * (1.0-2*percent) * cols * (1.0-2*percent));
                        }
                    } else {
                        mae_outside += (angle / M_PI) / (rows*cols - rows * (1.0-2*percent) * cols * (1.0-2*percent));  //1080*1920-1080*(1-2*0,1)*1920*(1-2*0,1)
                        if (actualAngle1 != 0 && forcastAngle1 != 0) {
                            mape1_outside += abs((actualAngle1 - forcastAngle1) / actualAngle1)
                            / (rows*cols - rows * (1.0-2*percent) * cols * (1.0-2*percent));
                        }
                        if (actualAngle2 != 0 && forcastAngle2 != 0) {
                            mape2_outside += abs((actualAngle2 - forcastAngle2) / actualAngle2)
                            / (rows*cols - rows * (1.0-2*percent) * cols * (1.0-2*percent));
                        }
                    }
                    
                }
            }
        }
        ofs  << step << ", " << mae_inside << ", " << mape1_inside << ", " << mape2_inside << ", " << mae_outside << ", " << mape1_outside << ", " << mape2_outside << endl;//fileName << ", " << actualsFileNames[count] << endl;
        cout << step << ", " << mae_inside << ", " << mape1_inside << ", " << mape2_inside << ", " << mae_outside << ", " << mape1_outside << ", " << mape2_outside << endl;//fileName << ", " << actualsFileNames[count] << endl;
        step++;
        count++;
    }
    return 0;
}

int computeError(String pathToForecasts, String pathToActuals) {
    ofstream ofs;
    ofs.open ("/Users/mariuspenteliuc/Desktop/recomputeErrors.txt", ofstream::out | ofstream::app);
    
    ofs << "step" << ", " << "mae_inside" << ", " << "mape1_inside" << ", " << "mape2_inside" << "mae_outside" << ", " << "mape1_outside" << ", " << "mape2_outside" << endl;//"forecast" << ", " << "actual" << endl;
    
    vector<string> actualsFileNames, forecastsFileNames;
//    glob("/Volumes/Transfers/debug_out/flows/*.npy", fileNames, false);
    glob(pathToActuals + "/*.npy", actualsFileNames, false);
    glob(pathToForecasts + "/*.npy", forecastsFileNames, false);
    int step = 0;
    int count = 0;  // skip first actual flow
    for (int i = 0; i < 366; i++) {
//    for (String fileName : forecastsFileNames) {
//        Mat forecast = FileHelper::readFile(fileName);
//        Mat actual = FileHelper::readFile(actualsFileNames[count]);
        Mat forecast = FileHelper::readFile(forecastsFileNames[i]);
        Mat actual = FileHelper::readFile(actualsFileNames[i]);
        
        float mae_inside = 0;
        float mape1_inside = 0;
        float mape2_inside = 0;
        float mae_outside = 0;
        float mape1_outside = 0;
        float mape2_outside = 0;
        Point2f referencePoint1 = Point2f(1,0);
        Point2f referencePoint2 = Point2f(-1,0);
    
        float percent = 0.4;
        int rows = actual.rows;
        int cols = actual.cols;
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                Point2f forecastPoint = forecast.at<Point2f>(row, col);
                Point2f realPoint = actual.at<Point2f>(row, col);
                float angle = Vector::getAngleBetween(realPoint, forecastPoint);
                float actualAngle1 = Vector::getAngleBetween(realPoint, referencePoint1);
                float forcastAngle1 = Vector::getAngleBetween(forecastPoint, referencePoint1);
                float actualAngle2 = Vector::getAngleBetween(realPoint, referencePoint2);
                float forcastAngle2 = Vector::getAngleBetween(forecastPoint, referencePoint2);
                if (!isnan(angle)) {
                    if (row >= rows*percent && row < rows-rows*percent && col >= cols*percent && col < cols-cols*percent) {
                        mae_inside += (angle / M_PI) / (rows * (1.0-2*percent) * cols * (1.0-2*percent));  //1080*(1-2*0,1)*1920*(1-2*0,1)
//                        if (actualAngle1 != 0 && forcastAngle1 != 0) {
//                            mape1_inside += abs((actualAngle1 - forcastAngle1) / actualAngle1)
//                            / (rows * (1.0-2*percent) * cols * (1.0-2*percent));
//                        }
//                        if (actualAngle2 != 0 && forcastAngle2 != 0) {
//                            mape2_inside += abs((actualAngle2 - forcastAngle2) / actualAngle2)
//                            / (rows * (1.0-2*percent) * cols * (1.0-2*percent));
//                        }
                    } else {
                        mae_outside += (angle / M_PI) / (rows*cols - rows * (1.0-2*percent) * cols * (1.0-2*percent));  //1080*1920-1080*(1-2*0,1)*1920*(1-2*0,1)
//                        if (actualAngle1 != 0 && forcastAngle1 != 0) {
//                            mape1_outside += abs((actualAngle1 - forcastAngle1) / actualAngle1)
//                            / (rows*cols - rows * (1.0-2*percent) * cols * (1.0-2*percent));
//                        }
//                        if (actualAngle2 != 0 && forcastAngle2 != 0) {
//                            mape2_outside += abs((actualAngle2 - forcastAngle2) / actualAngle2)
//                            / (rows*cols - rows * (1.0-2*percent) * cols * (1.0-2*percent));
//                        }
                    }
                    
                }
            }
        }
        ofs  << step << ", " << mae_inside << ", " << mape1_inside << ", " << mape2_inside << ", " << mae_outside << ", " << mape1_outside << ", " << mape2_outside << endl;//fileName << ", " << actualsFileNames[count] << endl;
        cout << step << ", " << mae_inside << ", " << mape1_inside << ", " << mape2_inside << ", " << mae_outside << ", " << mape1_outside << ", " << mape2_outside << endl;//fileName << ", " << actualsFileNames[count] << endl;
        step++;
        count++;
    }
    return 0;
}

//int runExperiments(int CONFIG = 0) {
////    FileHelper::writeFile("/Volumes/Transfers/Experiments/day_window_1/test_1/averages/average_wind_map.npy", average);
//
////    Mat flow = FileHelper::readFile("/Users/mariuspenteliuc/Assets/PhD/debug/debug_out/flows/flow_00001.npy");
//    vector<bool> updateWindMapUsingBoids;
//    updateWindMapUsingBoids.push_back(false); updateWindMapUsingBoids.push_back(true);
//    updateWindMapUsingBoids.push_back(false); updateWindMapUsingBoids.push_back(true);
//    updateWindMapUsingBoids.push_back(false); updateWindMapUsingBoids.push_back(true);
//    updateWindMapUsingBoids.push_back(false); updateWindMapUsingBoids.push_back(true);
//    updateWindMapUsingBoids.push_back(false); updateWindMapUsingBoids.push_back(true);
//    updateWindMapUsingBoids.push_back(false); updateWindMapUsingBoids.push_back(true);
//    updateWindMapUsingBoids.push_back(true); updateWindMapUsingBoids.push_back(true);
//    updateWindMapUsingBoids.push_back(true); updateWindMapUsingBoids.push_back(true);
//    updateWindMapUsingBoids.push_back(true); updateWindMapUsingBoids.push_back(true);
//    vector<size_t> startIndex;
//    startIndex.push_back(0); startIndex.push_back(0); startIndex.push_back(0); startIndex.push_back(0);
//    startIndex.push_back(100); startIndex.push_back(100); startIndex.push_back(100); startIndex.push_back(100);
//    startIndex.push_back(200); startIndex.push_back(200); startIndex.push_back(200); startIndex.push_back(200);
//    startIndex.push_back(0); startIndex.push_back(0); startIndex.push_back(0); startIndex.push_back(0);
//    startIndex.push_back(0); startIndex.push_back(0); startIndex.push_back(0);
//    vector<int> numberOfFlows;
//    numberOfFlows.push_back(48); numberOfFlows.push_back(48); numberOfFlows.push_back(48); numberOfFlows.push_back(48);
//    numberOfFlows.push_back(336); numberOfFlows.push_back(336); numberOfFlows.push_back(336); numberOfFlows.push_back(336);
//    numberOfFlows.push_back(1460); numberOfFlows.push_back(1460); numberOfFlows.push_back(1460); numberOfFlows.push_back(1460);
//    numberOfFlows.push_back(48); numberOfFlows.push_back(1); numberOfFlows.push_back(4); numberOfFlows.push_back(4);
//    numberOfFlows.push_back(19); numberOfFlows.push_back(19); numberOfFlows.push_back(9);
//    vector<int> neighborhoodRadius;
//    neighborhoodRadius.push_back(0); neighborhoodRadius.push_back(5); neighborhoodRadius.push_back(11); neighborhoodRadius.push_back(31);
//    neighborhoodRadius.push_back(0); neighborhoodRadius.push_back(5); neighborhoodRadius.push_back(11); neighborhoodRadius.push_back(31);
//    neighborhoodRadius.push_back(0); neighborhoodRadius.push_back(5); neighborhoodRadius.push_back(11); neighborhoodRadius.push_back(31);
//    neighborhoodRadius.push_back(1); neighborhoodRadius.push_back(1); neighborhoodRadius.push_back(1); neighborhoodRadius.push_back(1);
//    neighborhoodRadius.push_back(1); neighborhoodRadius.push_back(1); neighborhoodRadius.push_back(1);
//    vector<String> outputFolders;
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_0");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_1");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_2");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_3");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_4");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_5");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_6");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_7");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_8");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_9");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_10");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_11");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_a");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_b");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_c");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_d"); // experiment 15
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_NN_1");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_NN_2");
//    outputFolders.push_back("/Volumes/Transfers/Experiments/Configuration_NN_3");
//
//    vector<string> fileNames, nnFileNames;
//    glob("/Volumes/Transfers/debug_out/flows/*.npy", fileNames, false);
//    glob("/Volumes/Transfers/debug_out/flows/*.npy", nnFileNames, false);
////    glob("/Users/mariuspenteliuc/Downloads/Sat24_images/Boids_flows/*.npy", fileNames, false);
//
////    Mat flow = OpticalFlowService::averageFlows(outputFolders[CONFIG] + "/historic_flows", startIndex[CONFIG], numberOfFlows[CONFIG]);
////    Mat flow = FileHelper::readFile(outputFolders[CONFIG] + "/averages/average_flow.npy");
//    Mat flow = FileHelper::readFile(fileNames[startIndex[CONFIG]]);
////    if (flow.empty()) {
//////        flow = OpticalFlowService::averageFlows("/Volumes/Transfers/debug_out/flows", startIndex[CONFIG], numberOfFlows[CONFIG]);
////        flow = OpticalFlowService::averageFlows("/Users/mariuspenteliuc/Downloads/Sat24_images/Boids_flows", startIndex[CONFIG], numberOfFlows[CONFIG]);
////        FileHelper::writeFile(outputFolders[CONFIG] + "/averages/average_flow.npy", flow);
////    }
//
////    cout << "averaged " << numberOfFlows[CONFIG] << " flows." << endl;
//
//    Scene scene = Scene(1920, 1080);
////    Scene scene = Scene (845, 615);
//    scene.updateWindMap(flow);
//    
////    scene.updateUsingWeightedAverage();
////    return 0;
//
//    Mat initialMask = FileHelper::readFile(fileNames[startIndex[CONFIG]]);
////    for(int row = 0; row < initialMask.rows; ++row) {
////        for(int col = 0; col < initialMask.cols; ++col) {
////            Point2f &fxy = initialMask.at<Point2f>(row, col);
////            if (round(fxy.x) > 0 || round(fxy.y) > 0) {
////                for (int k = rand() %100; k > 98; --k) { // reduce computational cost
////                    scene.addBoid(col, row, 2);
////                }
////            }
////        }
////    }
//    int maxBoids = 10000;
//    while(scene.getBoidsCount() < maxBoids){
//        int row = rand()% initialMask.rows;
//        int col = rand()%initialMask.cols;
//        auto fxy = initialMask.at<Point2f>(row, col);
//        if (round(fxy.x) > 0 || round(fxy.y) > 0) {
//            
//            if (rand() % 2) {
//                scene.addBoid(col, row, 2);
//            }
//            
//        }
//    }
//
//    scene.startSimulation(outputFolders[CONFIG], startIndex[CONFIG]);
////    if (neighborhoodRadius[CONFIG] == 0) {
////        scene.runSimulation(336);
////    } else {
//        ofstream ofs, ofs2;
//        ofs.open ("/Users/mariuspenteliuc/Desktop/boids_log.txt", ofstream::out | ofstream::app);
//        ofs2.open ("/Users/mariuspenteliuc/Downloads/boids_log_backup.txt", ofstream::out | ofstream::app);
//        
//        ofs << "maxBoids: " << maxBoids << endl;
//        ofs2 << "maxBoids: " << maxBoids << endl;
//        
//        
////        Mat blankImage = FileHelper::readFile("/Volumes/Transfers/Experiments/blank.jpg");
////        Mat grayImage;
////        cv::cvtColor(blankImage, grayImage, cv::COLOR_BGR2GRAY);
////        Mat overlayedImage = OpticalFlowService::overlayFlowLines(scene.getWindMap(), grayImage);
////        FileHelper::writeFile(outputFolders[CONFIG] + "/simulated_wind_map/wind_map_" + string(5 - to_string(startIndex[CONFIG]).length(), '0') + to_string(startIndex[CONFIG]) + ".jpg", overlayedImage);
////        for (int k = 1; k < 336; ++k) {
//        for (int k = 1; k < 336; ++k) { // to keep things fast
//            auto start = std::chrono::system_clock::now();
//
//            scene.runSimulation(1);
//            auto end = std::chrono::system_clock::now();
//
//            scene.updateWindMap(FileHelper::readFile(outputFolders[CONFIG] + "/forecastedWindMap " + string(5 - to_string(startIndex[CONFIG] + k).length(), '0') + to_string(startIndex[CONFIG] + k) + ".npy"));
//            
////            scene.updateWindMapUsingBoids(neighborhoodRadius[CONFIG], outputFolders[CONFIG]);
////            scene.updateUsingWeightedAverage();
//            
//            std::chrono::duration<double> elapsed_seconds = end-start;
//            std::time_t end_time = std::chrono::system_clock::to_time_t(end);
//            
////            cv::cvtColor(blankImage, grayImage, cv::COLOR_BGR2GRAY);
////            Mat overlayedImage = OpticalFlowService::overlayFlowLines(scene.getWindMap(), grayImage);
////            FileHelper::writeFile(outputFolders[CONFIG] + "/simulated_wind_map/wind_map_" + string(5 - to_string(startIndex[CONFIG] + k).length(), '0') + to_string(startIndex[CONFIG] + k) + ".jpg", overlayedImage);
////            Mat forecastedWindMap = scene.getWindMap();
////            FileHelper::writeFile(outputFolders[CONFIG] + "/forecastedWindMap " + string(5 - to_string(startIndex[CONFIG] + k).length(), '0') + to_string(startIndex[CONFIG] + k) + ".npy", forecastedWindMap);
////            forecastedWindMap = FileHelper::readFile(outputFolders[CONFIG] + "/forecastedWindMap.npy");
////            forecastedWindMap = FileHelper::readFile(outputFolders[CONFIG] + "/forecastedWindMap " + string(5 - to_string(startIndex[CONFIG] + k).length(), '0') + to_string(startIndex[CONFIG] + k) + ".npy");
////            Mat realWindMap = FileHelper::readFile(fileNames[startIndex[CONFIG] + k]);
////            Mat realWindMap = FileHelper::readFile(fileNames[startIndex[CONFIG] + k]);
////            Mat nnWindMap = FileHelper::readFile(fileNames[numberOfFlows[CONFIG] + k]);
////            Mat nnWindMap = FileHelper::readFile("/Users/mariuspenteliuc/git/CloudMovement-NN/images/trimmed/output/prediction/prediction.npy");
////            forecastedWindMap = nnWindMap;
//            
//
//            
////            float error = 0;
////            float mape = 0;
////            Point2f referencePoint = Point2f(1,0);
//            
////            float mape_x = 0;
////            float mape_y = 0;
////            float mape_error = 0;
////            float mape_error_1p = 0;
////            float mape_error_2p = 0;
////            float mape_error_4p = 0;
////            float mape_error_1p_out = 0;
////            float mape_error_2p_out = 0;
////            float mape_error_4p_out = 0;
//            
////            float percent = 0;
//            
//            // Calcul Eroare și MAPE
////            int rows = realWindMap.rows;
////            int cols = realWindMap.cols;
////            float pi = 2 * acos(0.0);
////            for (int row = 0; row < rows; row++) {
////                for (int col = 0; col < cols; col++) {
////                    Point2f forecastPoint = forecastedWindMap.at<Point2f>(row, col);
////                    Point2f realPoint = realWindMap.at<Point2f>(row, col);
////                    float angle = Vector::getAngleBetween(forecastPoint, realPoint);
////                    if (!isnan(angle)) {
////                        error += (angle * 100 / pi) / (rows * cols);
////                    }
////                    float actualAngle = Vector::getAngleBetween(realPoint, referencePoint);
////                    float forcastAngle = Vector::getAngleBetween(forecastPoint, referencePoint);
////                    if (actualAngle != 0 && forcastAngle != 0) {
////                        mape += abs((actualAngle - forcastAngle) / actualAngle)
////                        / (rows*cols);
////                    }
////                    if (isnan(error) || isinf(mape)) {
////                        cout << "found weirness..." << endl;
////                    }
////                }
////            }
//            cout << startIndex[CONFIG] + k << ", " << elapsed_seconds.count() << ", " << fileNames[startIndex[CONFIG] + k].substr(fileNames[startIndex[CONFIG] + k].find_last_of("/") + 1) << endl;
////            cout << startIndex[CONFIG] + k << ", " << error << ", " << mape << ", " << elapsed_seconds.count() << ", " << fileNames[startIndex[CONFIG] + k].substr(fileNames[startIndex[CONFIG] + k].find_last_of("/") + 1) << endl;
////            cout << "\tfinished computation at " << std::ctime(&end_time) << "\telapsed time: " << elapsed_seconds.count() << "s\n";
//            
//            //            ofs << startIndex[CONFIG] + k << " — " <<  " MAPE: <" << mape_error << "> 10% trim: <" << mape_error_1p << "> 20% trim: <" << mape_error_2p << "> 40% trim: <" << mape_error_4p << "> 10% edge: <" << mape_error_1p_out <<  "> 20% edge: <" << mape_error_2p_out <<  "> 40% edge: <" << mape_error_4p_out << ">\t// " << fileNames[startIndex[CONFIG] + k] << " AND " << outputFolders[CONFIG] << "/simulated_wind_map/wind_map_" << string(5 - to_string(startIndex[CONFIG] + k).length(), '0') << to_string(startIndex[CONFIG] + k) << ".jpg" << "— Time:" << elapsed_seconds.count() << endl;
//            //            ofs2 << startIndex[CONFIG] + k << " — " <<  " MAPE: <" << mape_error << "> 10% trim: <" << mape_error_1p << "> 20% trim: <" << mape_error_2p << "> 40% trim: <" << mape_error_4p << "> 10% edge: <" << mape_error_1p_out <<  "> 20% edge: <" << mape_error_2p_out <<  "> 40% edge: <" << mape_error_4p_out << ">\t// " << fileNames[startIndex[CONFIG] + k] << " AND " << outputFolders[CONFIG] << "/simulated_wind_map/wind_map_" << string(5 - to_string(startIndex[CONFIG] + k).length(), '0') << to_string(startIndex[CONFIG] + k) << ".jpg" << "— Time:" << elapsed_seconds.count() << endl;
//            ofs << startIndex[CONFIG] + k << ", " << elapsed_seconds.count() << ", " << fileNames[startIndex[CONFIG] + k].substr(fileNames[startIndex[CONFIG] + k].find_last_of("/") + 1) << endl;
//            ofs2 << startIndex[CONFIG] + k << ", " << elapsed_seconds.count() << ", " << fileNames[startIndex[CONFIG] + k].substr(fileNames[startIndex[CONFIG] + k].find_last_of("/") + 1) << endl;
////            ofs << startIndex[CONFIG] + k << ", " << error << ", " << mape << ", " << elapsed_seconds.count() << ", " << fileNames[startIndex[CONFIG] + k].substr(fileNames[startIndex[CONFIG] + k].find_last_of("/") + 1) << endl;
////            ofs2 << startIndex[CONFIG] + k << ", " << error << ", " << mape << ", " << elapsed_seconds.count() << ", " << fileNames[startIndex[CONFIG] + k].substr(fileNames[startIndex[CONFIG] + k].find_last_of("/") + 1) << endl;
////            for(int row = 0 + realWindMap.rows * percent; row < realWindMap.rows - realWindMap.rows * percent; ++row) {
////                for(int col = 0 + realWindMap.cols * percent; col < realWindMap.cols - realWindMap.cols * percent; ++col) {
////                    if (row < 0 + rows * 0.2 || row > rows - rows * 0.2 || col < 0 + cols * 0.2 || col > cols - cols * 0.2) {
////                        Point2f forecastPoint = transformToSimpleDirection(Point2f(forecastedWindMap.at<int>(row+col*1154),forecastedWindMap.at<int>(row+col*1154+1)));
////                        Point2f realPoint = transformToSimpleDirection(realWindMap.at<Point2f>(row+432, col+768));
////                        if ((realPoint.x > 0 || realPoint.y > 0) && (realPoint.x != forecastPoint.x || realPoint.y != forecastPoint.y) && realPoint.x - forecastPoint.x == 0 && realPoint.y - forecastPoint.y == 0) {
////                            mape_error_2p_out += 100.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        } else {
////                            float distance = Vector::getEuclidianDistance(forecastPoint, realPoint);
////                            if (distance == 0) {
////
////                            } else if (distance <= 1) {
////                                mape_error_2p_out += 25.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                            } else if (distance <= 2) {
////                                mape_error_2p_out += 50.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                            } else mape_error_2p_out += 75.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        }
////                        Point2f point = computeAPE(realWindMap.at<Point2f>(row, col), forecastedWindMap.at<Point2f>(row, col));
////                        mape_x += point.x / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        mape_y += point.y / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    }
////                    if (row < 0 + rows * 0.1 || row > rows - rows * 0.1 || col < 0 + cols * 0.1 || col > cols - cols * 0.1) {
////                        Point2f forecastPoint = transformToSimpleDirection(Point2f(forecastedWindMap.at<int>(row+col*1154),forecastedWindMap.at<int>(row+col*1154+1)));
////                        Point2f realPoint = transformToSimpleDirection(realWindMap.at<Point2f>(row, col));
////                        if ((realPoint.x > 0 || realPoint.y > 0) && (realPoint.x != forecastPoint.x || realPoint.y != forecastPoint.y) && realPoint.x - forecastPoint.x == 0 && realPoint.y - forecastPoint.y == 0) {
////                            mape_error_1p_out += 100.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        } else {
////                            float distance = Vector::getEuclidianDistance(forecastPoint, realPoint);
////                            if (distance == 0) {
////
////                            } else if (distance <= 1) {
////                                mape_error_1p_out += 25.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                            } else if (distance <= 2) {
////                                mape_error_1p_out += 50.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                            } else mape_error_1p_out += 75.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        }
////                        Point2f point = computeAPE(realWindMap.at<Point2f>(row, col), forecastedWindMap.at<Point2f>(row, col));
////                        mape_x += point.x / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        mape_y += point.y / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    }
////                    if (row < 0 + rows * 0.4 || row > rows - rows * 0.4 || col < 0 + cols * 0.4 || col > cols - cols * 0.4) {
////                        Point2f forecastPoint = transformToSimpleDirection(Point2f(forecastedWindMap.at<int>(row+col*1154),forecastedWindMap.at<int>(row+col*1154+1)));
////                        Point2f realPoint = transformToSimpleDirection(realWindMap.at<Point2f>(row, col));
////                        if ((realPoint.x > 0 || realPoint.y > 0) && (realPoint.x != forecastPoint.x || realPoint.y != forecastPoint.y) && realPoint.x - forecastPoint.x == 0 && realPoint.y - forecastPoint.y == 0) {
////                            mape_error_4p_out += 100.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        } else {
////                            float distance = Vector::getEuclidianDistance(forecastPoint, realPoint);
////                            if (distance == 0) {
////
////                            } else if (distance <= 1) {
////                                mape_error_4p_out += 25.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                            } else if (distance <= 2) {
////                                mape_error_4p_out += 50.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                            } else mape_error_4p_out += 75.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        }
////                        Point2f point = computeAPE(realWindMap.at<Point2f>(row, col), forecastedWindMap.at<Point2f>(row, col));
////                        mape_x += point.x / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        mape_y += point.y / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    }
////                    Point2f forecastPoint = transformToSimpleDirection(Point2f(forecastedWindMap.at<float>(row+col*1154),forecastedWindMap.at<float>(row+col*1154+1)));
////                    Point2f realPoint = transformToSimpleDirection(realWindMap.at<Point2f>(row, col));
////                    if ((realPoint.x > 0 || realPoint.y > 0) && (realPoint.x != forecastPoint.x || realPoint.y != forecastPoint.y) && realPoint.x - forecastPoint.x == 0 && realPoint.y - forecastPoint.y == 0) {
////                        mape_error += 100.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    } else {
////                        float distance = Vector::getEuclidianDistance(forecastPoint, realPoint);
////                        if (distance == 0) {
////
////                        } else if (distance <= 1) {
////                            mape_error += 25.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        } else if (distance <= 2) {
////                            mape_error += 50.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        } else mape_error += 75.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    }
////                    Point2f point = computeAPE(realWindMap.at<Point2f>(row, col), forecastedWindMap.at<Point2f>(row, col));
////                    mape_x += point.x / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    mape_y += point.y / (forecastedWindMap.rows * forecastedWindMap.cols);
////                }
////            }
////            percent = 0.2;
////            for(int row = 0 + forecastedWindMap.rows * percent; row < forecastedWindMap.rows - forecastedWindMap.rows * percent; ++row) {
////                for(int col = 0 + forecastedWindMap.cols * percent; col < forecastedWindMap.cols - forecastedWindMap.cols * percent; ++col) {
////                    Point2f forecastPoint = transformToSimpleDirection(forecastedWindMap.at<Point2f>(row, col));
////                    Point2f realPoint = transformToSimpleDirection(realWindMap.at<Point2f>(row, col));
////                    if ((realPoint.x > 0 || realPoint.y > 0) && (realPoint.x != forecastPoint.x || realPoint.y != forecastPoint.y) && realPoint.x - forecastPoint.x == 0 && realPoint.y - forecastPoint.y == 0) {
////                        mape_error_2p += 100.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    } else {
////                        float distance = Vector::getEuclidianDistance(forecastPoint, realPoint);
////                        if (distance == 0) {
////
////                        } else if (distance <= 1) {
////                            mape_error_2p += 25.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        } else if (distance <= 2) {
////                            mape_error_2p += 50.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        } else mape_error_2p += 75.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    }
////                    Point2f point = computeAPE(realWindMap.at<Point2f>(row, col), forecastedWindMap.at<Point2f>(row, col));
////                    mape_x += point.x / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    mape_y += point.y / (forecastedWindMap.rows * forecastedWindMap.cols);
////                }
////            }
////            percent = 0.1;
////            for(int row = 0 + forecastedWindMap.rows * percent; row < forecastedWindMap.rows - forecastedWindMap.rows * percent; ++row) {
////                for(int col = 0 + forecastedWindMap.cols * percent; col < forecastedWindMap.cols - forecastedWindMap.cols * percent; ++col) {
////                    Point2f forecastPoint = transformToSimpleDirection(forecastedWindMap.at<Point2f>(row, col));
////                    Point2f realPoint = transformToSimpleDirection(realWindMap.at<Point2f>(row, col));
////                    if ((realPoint.x > 0 || realPoint.y > 0) && (realPoint.x != forecastPoint.x || realPoint.y != forecastPoint.y) && realPoint.x - forecastPoint.x == 0 && realPoint.y - forecastPoint.y == 0) {
////                        mape_error_1p += 100.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    } else {
////                        float distance = Vector::getEuclidianDistance(forecastPoint, realPoint);
////                        if (distance == 0) {
////
////                        } else if (distance <= 1) {
////                            mape_error_1p += 25.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        } else if (distance <= 2) {
////                            mape_error_1p += 50.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        } else mape_error_1p += 75.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    }
////                    Point2f point = computeAPE(realWindMap.at<Point2f>(row, col), forecastedWindMap.at<Point2f>(row, col));
////                    mape_x += point.x / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    mape_y += point.y / (forecastedWindMap.rows * forecastedWindMap.cols);
////                }
////            }
////            percent = 0.4;
////            for(int row = 0 + forecastedWindMap.rows * percent; row < forecastedWindMap.rows - forecastedWindMap.rows * percent; ++row) {
////                for(int col = 0 + forecastedWindMap.cols * percent; col < forecastedWindMap.cols - forecastedWindMap.cols * percent; ++col) {
////                    Point2f forecastPoint = transformToSimpleDirection(forecastedWindMap.at<Point2f>(row, col));
////                    Point2f realPoint = transformToSimpleDirection(realWindMap.at<Point2f>(row, col));
////                    if ((realPoint.x > 0 || realPoint.y > 0) && (realPoint.x != forecastPoint.x || realPoint.y != forecastPoint.y) && realPoint.x - forecastPoint.x == 0 && realPoint.y - forecastPoint.y == 0) {
////                        mape_error_4p += 100.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    } else {
////                        float distance = Vector::getEuclidianDistance(forecastPoint, realPoint);
////                        if (distance == 0) {
////
////                        } else if (distance <= 1) {
////                            mape_error_4p += 25.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        } else if (distance <= 2) {
////                            mape_error_4p += 50.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                        } else mape_error_4p += 75.0 / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    }
////                    Point2f point = computeAPE(realWindMap.at<Point2f>(row, col), forecastedWindMap.at<Point2f>(row, col));
////                    mape_x += point.x / (forecastedWindMap.rows * forecastedWindMap.cols);
////                    mape_y += point.y / (forecastedWindMap.rows * forecastedWindMap.cols);
////                }
////            }
////            cout << "MAPE between real " << fileNames[startIndex[CONFIG] + k] << " and forecast " << outputFolders[CONFIG] << "/simulated_wind_map/wind_map_" << string(5 - to_string(startIndex[CONFIG] + k).length(), '0') << to_string(startIndex[CONFIG] + k) << ".jpg" << " is:" << endl;
////            cout << "AVG("<< mape_x << ", " << mape_y << ") = " << (mape_x * mape_y) / 2 << endl;
////            cout << "directional MAPE: " << mape_error << endl;
////            cout << startIndex[CONFIG] + k << " — " <<  " MAPE: <" << mape_error << "> 10% trim: <" << mape_error_1p << "> 20% trim: <" << mape_error_2p << "> 40% trim: <" << mape_error_4p << "> 10% edge: <" << mape_error_1p_out <<  "> 20% edge: <" << mape_error_2p_out <<  "> 40% edge: <" << mape_error_4p_out << ">\t// " << fileNames[startIndex[CONFIG] + k] << " AND " << outputFolders[CONFIG] << "/simulated_wind_map/wind_map_" << string(5 - to_string(startIndex[CONFIG] + k).length(), '0') << to_string(startIndex[CONFIG] + k) << ".jpg" << "— Time:" << elapsed_seconds.count() << endl;
////            cout << "\tfinished computation at " << std::ctime(&end_time) << "\telapsed time: " << elapsed_seconds.count() << "s\n";
//            
////            ofs << startIndex[CONFIG] + k << " — " <<  " MAPE: <" << mape_error << "> 10% trim: <" << mape_error_1p << "> 20% trim: <" << mape_error_2p << "> 40% trim: <" << mape_error_4p << "> 10% edge: <" << mape_error_1p_out <<  "> 20% edge: <" << mape_error_2p_out <<  "> 40% edge: <" << mape_error_4p_out << ">\t// " << fileNames[startIndex[CONFIG] + k] << " AND " << outputFolders[CONFIG] << "/simulated_wind_map/wind_map_" << string(5 - to_string(startIndex[CONFIG] + k).length(), '0') << to_string(startIndex[CONFIG] + k) << ".jpg" << "— Time:" << elapsed_seconds.count() << endl;
////            ofs2 << startIndex[CONFIG] + k << " — " <<  " MAPE: <" << mape_error << "> 10% trim: <" << mape_error_1p << "> 20% trim: <" << mape_error_2p << "> 40% trim: <" << mape_error_4p << "> 10% edge: <" << mape_error_1p_out <<  "> 20% edge: <" << mape_error_2p_out <<  "> 40% edge: <" << mape_error_4p_out << ">\t// " << fileNames[startIndex[CONFIG] + k] << " AND " << outputFolders[CONFIG] << "/simulated_wind_map/wind_map_" << string(5 - to_string(startIndex[CONFIG] + k).length(), '0') << to_string(startIndex[CONFIG] + k) << ".jpg" << "— Time:" << elapsed_seconds.count() << endl;
////            ofs << startIndex[CONFIG] + k << " — " <<  " MAPE: <" << mape_error << ">\t// " << fileNames[startIndex[CONFIG] + k] << " AND " << outputFolders[CONFIG] << "/simulated_wind_map/wind_map_" << string(5 - to_string(startIndex[CONFIG] + k).length(), '0') << to_string(startIndex[CONFIG] + k) << ".jpg" << "— Time:" << elapsed_seconds.count() << endl;
////            ofs2 << startIndex[CONFIG] + k << " — " <<  " MAPE: <" << mape_error << ">\t// " << fileNames[startIndex[CONFIG] + k] << " AND " << outputFolders[CONFIG] << "/simulated_wind_map/wind_map_" << string(5 - to_string(startIndex[CONFIG] + k).length(), '0') << to_string(startIndex[CONFIG] + k) << ".jpg" << "— Time:" << elapsed_seconds.count() << endl;
//        }
//        ofs << endl;
//        ofs.close();
//        ofs2 << endl;
//        ofs2.close();
////    }
//
//    return 0;
//}

int main(int argc, const char * argv[]) {
    srand (static_cast <unsigned> (time(0)));
    runExperiments();
//    cout << "Computing optical flow..." << endl;
//    OpticalFlowService ofService;
//    string INPUT = "/Users/mariuspenteliuc/Assets/PhD/ExperimentalSets/GrowMovement_1/Frames";
//    string OUTPUT = "/Users/mariuspenteliuc/Assets/PhD/ExperimentalSets/GrowMovement_1";
//    OpticalFlowService::computeFlowForImages(DEBUG_IN, DEBUG_OUT, "jpg", true, true, false);
//    OpticalFlowService::computeFlowForImages(INPUT, OUTPUT, "jpg", true, true, false);
//    OpticalFlowService::computeFlowForImages("/Users/mariuspenteliuc/Downloads/Sat24_images", DEBUG_OUT, "png", true, true, false);
//    cv::String pathToFlows(DEBUG_OUT + "/flows");

//    cout << "Computing Wind Map..." << endl;
//    Mat average = OpticalFlowService::averageFlows(pathToFlows);
//    FileHelper::writeFile("/Users/mariuspenteliuc/Assets/PhD/debug/debug_out/flows/average_flow.npy", average);
//    Mat average = FileHelper::readFile("/Users/mariuspenteliuc/Assets/PhD/debug/debug_out/flows/average_flow.npy");

//    cout << "Populating scene with 10 000 boids..." << endl;
//    cout << "Populating scene with boids...";
//    Scene scene = Scene(1920, 1080);
//    for (int i = 1; i <= 1920; i+=5) {
//        for (int j = 1; j <= 1080; j+=5) {
//            Point2f& fxy = average.at<Point2f>(j, i);
//            if (abs(fxy.x) > 1 && abs(fxy.y) > 1) {
//                scene.addBoid(i, j, 1);
//            }
//        }
//    }
//    for (int i = 0; i < 10000; ++i) {
//        scene.addRandomBoid();
//    }
//
//    cout << "Starting Simulation..." << endl;
//    scene.updateWindMap(average);
//    scene.startSimulation(DEBUG_OUT, 0);
//    scene.runSimulation(150);
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
