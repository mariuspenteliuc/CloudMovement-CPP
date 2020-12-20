//
//  OpticalFlowService.hpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 12.11.2020.
//

#ifndef OpticalFlowService_hpp
#define OpticalFlowService_hpp

#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

class OpticalFlowService {
private:
    vector<cv::String> fileNames;
    string fileType;

    static cv::Mat cvtColorBGR2GRAY(cv::Mat bgrMat);
    static void drawOpticalFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, double, const cv::Scalar& color);
    static cv::Mat getOpticalFlowFarneback(cv::Mat firstImage, cv::Mat secondImage);
    static cv::Mat overlayFlowLines(cv::Mat flow, cv::Mat image);
    static void saveFlowToDisk(string fileName, cv::Mat flow);
    static void saveImageToDisk(string fileName, cv::Mat image);
    static cv::Mat loadImageFromDisk(string fileName);

public:
    OpticalFlowService();
    static string getOpenCVVersion();
    int computeFlowForImages(string inputPath, string outputPath, string fileType, bool saveOverlays, bool saveFlows, bool previewOverlays, int numberOfThreads);
};

#endif /* OpticalFlowService_hpp */
