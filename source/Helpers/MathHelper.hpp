//
//  MathHelper.hpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 16.11.2020.
//

#ifndef MathHelper_hpp
#define MathHelper_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

Point2f addPoints(Point2f v1, Point2f v2);
Point2f removePoints(Point2f v1, Point2f v2);
Point2f removePoints(Point2f v1, float a);
Point2f dividePoints(Point2f v1, float a);
float ruleOfThree(float part, float whole);
Point2f multiplyPoint(Point2f p, float f);

#endif /* MathHelper_hpp */
