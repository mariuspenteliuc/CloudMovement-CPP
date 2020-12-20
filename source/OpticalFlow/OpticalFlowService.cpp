//
//  OpticalFlowService.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 12.11.2020.
//

#include "OpticalFlowService.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <omp.h>
#include <unistd.h>

using namespace std;
using namespace std::chrono;
using namespace cv;

OpticalFlowService::OpticalFlowService() {
}

string OpticalFlowService::getOpenCVVersion() {
    return CV_VERSION;
}

/// Compute Farneback Optical Flow on the images available at the specified directory.
/// @param fileType The type of images to be loaded. Can be `jpg`, `png`, and so on.
/// @param inputPath The path to the folder containing the images.
/// @param outputPath The path to the folder where results should be saved.
/// @param saveOverlays `true` if images should have flow lines overlayed on them and be saved to disk, `false` otherwise.
/// @param saveFlows `true` if flows should be saved to disk, `false` otherwise.
/// @param numberOfThreads Number of threads. Default is `1` (secvential). 
int OpticalFlowService::computeFlowForImages(string inputPath, string outputPath, string fileType = "jpg", bool saveOverlays = false, bool saveFlows = true, bool previewOverlays = false, int numberOfThreads = 1) {
    if (saveFlows) {
        std::__fs::filesystem::create_directories(outputPath + "/flows/");
    }
    if (saveOverlays) {
        std::__fs::filesystem::create_directories(outputPath + "/overlays/");
    }
    OpticalFlowService::fileType = fileType;
    cv::String path(inputPath + "/*." + fileType);
    cv::glob(path,fileNames,false);
    cv::Mat im1 = cv::imread(fileNames[0]);


    omp_set_num_threads(numberOfThreads); // set number of threads in "parallel" blocks
    #pragma omp parallel for
    for (size_t k=1; k<fileNames.size(); ++k) {
        cv::Mat im2 = cv::imread(fileNames[k]);
        if (im1.empty() || im2.empty()) continue; //only proceed if sucsessful

        Mat flow = getOpticalFlowFarneback(im1, im2);
        if (saveFlows) {
            saveFlowToDisk(outputPath + "/flows/flow_" + to_string(k) + ".npy", flow);
        }
        if (saveOverlays || previewOverlays) {
            Mat grayImage = cvtColorBGR2GRAY(im2);
            Mat overlayedImage = overlayFlowLines(flow, grayImage);

            if (saveOverlays) {
                saveImageToDisk(outputPath + "/overlays/image_" + to_string(k) + ".jpg", overlayedImage);
            }
            if (previewOverlays) {
                namedWindow("OpticalFlow", WINDOW_AUTOSIZE);
                imshow("Image " + to_string(k), overlayedImage);
                waitKey();
            }
        }
        im1 = im2;
    }
    return 0;
}

void OpticalFlowService::saveFlowToDisk(string fileName, cv::Mat flow) {
    writeOpticalFlow(fileName, flow);
}

void OpticalFlowService::saveImageToDisk(string fileName, cv::Mat image) {
    imwrite(fileName, image);
}

cv::Mat OpticalFlowService::cvtColorBGR2GRAY(cv::Mat bgrMat) {
    cv::Mat grayMat;
    cv::cvtColor(bgrMat, grayMat, cv::COLOR_BGR2GRAY);
    return grayMat;
}

cv::Mat OpticalFlowService::getOpticalFlowFarneback(cv::Mat firstImage, cv::Mat secondImage) {
    cv::Mat firstGrayMat, secondGrayMat;
    cv::cvtColor(firstImage, firstGrayMat, cv::COLOR_BGR2GRAY);
    cv::cvtColor(secondImage, secondGrayMat, cv::COLOR_BGR2GRAY);
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(firstGrayMat, secondGrayMat, flow, 0.5, 7, 50, 3, 7, 1.5, 0); /// nice defaults: 0.5, 3, 15, 3, 5, 1.2, 0
    return flow;
}

cv::Mat OpticalFlowService::overlayFlowLines(cv::Mat flow, cv::Mat image) {
    cv::Mat imageWithFlowOverlay;
    cv::cvtColor(image, imageWithFlowOverlay, cv::COLOR_GRAY2BGR);
    drawOpticalFlowMap(flow, imageWithFlowOverlay, 10, 1.5, cv::Scalar(0, 255, 0));
    return imageWithFlowOverlay;
}

void OpticalFlowService::drawOpticalFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, double, const cv::Scalar& color) {
    int x;
    // auto start = high_resolution_clock::now(); 
    
    #pragma omp simd collapse(2)
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step) {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            const cv::Point roundedPoint = cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y));
            line(cflowmap, cv::Point(x,y), roundedPoint, color, 1, cv::LINE_AA);
            circle(cflowmap, cv::Point(x,y), 0.5, color, cv::FILLED);
        
        }

    // auto stop = high_resolution_clock::now(); 

    // auto duration = duration_cast<microseconds>(stop - start); 
  
    // cout << "Time taken by function: "
    //      << duration.count() << " microseconds" << endl; 
}


