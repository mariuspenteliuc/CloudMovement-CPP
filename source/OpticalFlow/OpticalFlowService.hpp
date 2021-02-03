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
#include <filesystem>
#include "Helpers/FileHelper.hpp"
#include "Helpers/MathHelper.hpp"

using namespace std;

class OpticalFlowService {
private:
    static void drawOpticalFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, double, const cv::Scalar& color);
    static cv::Mat getOpticalFlowFarneback(cv::Mat firstImage, cv::Mat secondImage);
    static cv::Mat overlayFlowLines(cv::Mat flow, cv::Mat image);
public:
    OpticalFlowService();
    static int computeFlowForImages(string inputPath, string outputPath, string fileType, bool saveOverlays, bool saveFlows, bool previewOverlays, int numberOfThreads);
    static cv::Mat averageFlows(string inputPath);
};

#endif /* OpticalFlowService_hpp */
