//
//  ImageHelper.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 12.11.2020.
//

#include "ImageHelper.hpp"

cv::Mat ImageHelper::readImage(std::string pathToFile) {
    cv::Mat image = cv::imread(pathToFile);
    return image;
}
