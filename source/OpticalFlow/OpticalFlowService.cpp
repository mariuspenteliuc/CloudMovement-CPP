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

/// Compute Farneback Optical Flow on the images available at the specified directory.
/// @param fileType The type of images to be loaded. Can be `jpg`, `png`, and so on.
/// @param inputPath The path to the folder containing the images.
/// @param outputPath The path to the folder where results should be saved.
/// @param saveOverlays `true` if images should have flow lines overlayed on them and be saved to disk, `false` otherwise.
/// @param saveFlows `true` if flows should be saved to disk, `false` otherwise.
int OpticalFlowService::computeFlowForImages(string inputPath, string outputPath, string fileType = "jpg", bool saveOverlays = false, bool saveFlows = true, bool previewOverlays = false) {
    vector<cv::String> fileNames;
    cv::String path(inputPath + "/*." + fileType);
    cv::glob(path,fileNames,false);
    cv::Mat im1 = FileHelper::readFile(fileNames[0]);
    for (size_t k=1; k<fileNames.size(); ++k) {
         cv::Mat im2 = FileHelper::readFile(fileNames[k]);
         if (im1.empty() || im2.empty()) continue; //only proceed if sucsessful

        Mat flow = getOpticalFlowFarneback(im1, im2);
        if (saveFlows) {
            FileHelper::writeFile(outputPath + "/flows/flow_" + string(5 - to_string(k).length(), '0') + to_string(k) + ".npy", flow);
        }
        if (saveOverlays || previewOverlays) {
            Mat grayImage;// = FileHelper::convertToGray(im2);
            cv::cvtColor(im2, grayImage, cv::COLOR_BGR2GRAY);
            Mat overlayedImage = overlayFlowLines(flow, grayImage);

            if (saveOverlays) {
                FileHelper::writeFile(outputPath + "/overlays/image_" + string(5 - to_string(k).length(), '0') + to_string(k) + ".jpg", overlayedImage);
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

/**
 * Averages multiple flows found in a directory
 *
 * @param inputPath the path to the directory inside of which the flows are found.
 * @return a flow computed as the average.
 */
Mat OpticalFlowService::averageFlows(string inputPath) {
    String path(inputPath + "/*." + "npy");
    vector<Mat> flows = FileHelper::readFiles(path);
    int cols = flows[0].cols;
    int rows = flows[0].rows;
    Mat flowAverage = flows[0];
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
            flowAverage.at<Point2f>(row, col) = Point2f(averageX, averageY);
        }
    }
    return flowAverage;
}

/**
 * Computes and returns the Optical Flow Farneback variant.
 *
 * @param firstImage the image cronologically first.
 * @param secondImage the image cronologically second.
 * @return a flow containg data that represent dispalcement of pixels in the secondImage.
 */
cv::Mat OpticalFlowService::getOpticalFlowFarneback(cv::Mat firstImage, cv::Mat secondImage) {
    cv::Mat firstGrayMat, secondGrayMat;
    cv::cvtColor(firstImage, firstGrayMat, cv::COLOR_BGR2GRAY);
    cv::cvtColor(secondImage, secondGrayMat, cv::COLOR_BGR2GRAY);
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(firstGrayMat, secondGrayMat, flow, 0.5, 7, 50, 3, 7, 1.5, 0); /// nice defaults: 0.5, 3, 15, 3, 5, 1.2, 0
    return flow;
}

/**
 * Overlays a grid of lines to visualize motion vectors generated from dispalcement data.
 *
 * @param flow the motion data to be overlayed.
 * @param image the image onto the flow grid should be overlayed.
 * @return the image with flow lines overlayed on top.
 */
cv::Mat OpticalFlowService::overlayFlowLines(cv::Mat flow, cv::Mat image) {
    cv::Mat imageWithFlowOverlay;
    cv::cvtColor(image, imageWithFlowOverlay, cv::COLOR_GRAY2BGR);
    drawOpticalFlowMap(flow, imageWithFlowOverlay, 10, 1.5, cv::Scalar(0, 255, 0));
    return imageWithFlowOverlay;
}

/**
 * Draws the Optical Flow map by adding a dot grid overlay and lines representing the motion direction and length.
 *
 * @param flow the flow from which data is to be drawn.
 * @param cflowmap the image onto which data is to be drawn.
 * @param step the grid step size.
 * @param double? not sure why I have this...
 * @param color the color data should have.
 */
void OpticalFlowService::drawOpticalFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, double, const cv::Scalar& color) {
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step) {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            const cv::Point roundedPoint = cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y));
            line(cflowmap, cv::Point(x,y), roundedPoint, color, 1, cv::LINE_AA);
            circle(cflowmap, cv::Point(x,y), 0.5, color, cv::FILLED);
        }
}


