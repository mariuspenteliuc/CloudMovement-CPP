//
//  FileHelper.hpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 12.11.2020.
//

#ifndef FileHelper_hpp
#define FileHelper_hpp

#include <stdio.h>
#include <filesystem>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class FileHelper {

public:
    static Mat readFile(string fileName);
    static vector<Mat> readFiles(string folderPath);
    static bool writeFile(string fileName, Mat data);
};

#endif /* FileHelper_hpp */
