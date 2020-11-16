//
//  ImageHelper.hpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 12.11.2020.
//

#ifndef ImageHelper_hpp
#define ImageHelper_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

class ImageHelper {

public:
    static cv::Mat readImage(std::string path);
};

#endif /* ImageHelper_hpp */
