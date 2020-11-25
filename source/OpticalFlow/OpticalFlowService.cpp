//
//  OpticalFlowService.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 12.11.2020.
//

#include "OpticalFlowService.hpp"

using namespace std;
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
int OpticalFlowService::computeFlowForImages(string inputPath, string outputPath, string fileType = "jpg", bool saveOverlays = false, bool saveFlows = true, bool previewOverlays = false) {
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
//                waitKey();
            }
        }
        im1 = im2;
    }
    return 0;
}

vector<cv::Mat> OpticalFlowService::loadFlowsFromDirectory(string inputPath) {
    cv::String path(inputPath + "/*." + "npy");
    vector<string> fileNames;
    cv::glob(path, fileNames, false);
    vector<Mat> flows;
    for (size_t k=0; k<fileNames.size(); ++k) {
//        flows.push_back(imread(fileNames[k]));
        flows.push_back(readOpticalFlow(fileNames[k]));
    }
    return flows;
}

Mat OpticalFlowService::averageFlows(string inputPath) {
    vector<Mat> flows = OpticalFlowService::loadFlowsFromDirectory(inputPath);
    int cols = flows[0].cols;
    int rows = flows[0].rows;
    Mat flowAverage = flows[0];
//    Mat flowAverage = Mat::zeros(cols, rows, CV_32FC4);
//    flowAverage = cvtColorBGR2GRAY(flowAverage);
    for(int row = 0; row < rows; ++row) {
        for(int col = 0; col < cols; ++col) {
            int movementCount = 0;
            float averageX = 0.0;
            float averageY = 0.0;
            for (Mat flow : flows) {
                Point2f& fxy = flow.at<Point2f>(row, col);
                if (fxy.x > 0.001 || fxy.y > 0.001) {
                    movementCount++;
                    averageX += fxy.x;
                    averageY += fxy.y;
                }
            }
            if (movementCount == 0) {
                movementCount++;
            }
            averageX /= movementCount;
            averageY /= movementCount;
//            std::cout <<flowAverage.at<Point2f>(y, x).x << ", " << flowAverage.at<Point2f>(y, x).y << endl;
            flowAverage.at<Point2f>(row, col) = Point2f(averageX, averageY);
//            std::cout <<flowAverage.at<Point2f>(y, x).x << ", " << flowAverage.at<Point2f>(y, x).y << endl;
        }
    }
    return flowAverage;
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
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step) {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            const cv::Point roundedPoint = cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y));
            line(cflowmap, cv::Point(x,y), roundedPoint, color, 1, cv::LINE_AA);
            circle(cflowmap, cv::Point(x,y), 0.5, color, cv::FILLED);
        }
}


