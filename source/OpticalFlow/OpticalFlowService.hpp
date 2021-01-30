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
#include "FileHelper.hpp"
#include "MathHelper.hpp"

using namespace std;

class OpticalFlowService {
private:
    static void drawOpticalFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, double, const cv::Scalar& color);
    static cv::Mat getOpticalFlowFarneback(cv::Mat firstImage, cv::Mat secondImage);
public:
    static cv::Mat overlayFlowLines(cv::Mat flow, cv::Mat image);
    OpticalFlowService();
    static int computeFlowForImages(string inputPath, string outputPath, string fileType, bool saveOverlays, bool saveFlows, bool previewOverlays);
    static cv::Mat averageFlows(string inputPath, size_t index = 0, size_t numberOfFlows = 0);
};

#endif /* OpticalFlowService_hpp */
