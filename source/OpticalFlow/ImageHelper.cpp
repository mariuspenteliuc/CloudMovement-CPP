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
//            if (img.data == 0) {
//                std::cerr << "Image not found!" << std::endl;
//                return -1;
//            }
//        namedWindow("image", WINDOW_AUTOSIZE);
//            imshow("image", img);
//            waitKey();
}
