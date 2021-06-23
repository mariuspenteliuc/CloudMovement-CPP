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
#include <math.h>
#include "Vector.hpp"

using namespace cv;

Point2f addPoints(Point2f v1, Point2f v2);
Point2f computeAPE(Point2f real, Point2f forecast);
Point2f removePoints(Point2f v1, Point2f v2);
Point2f removePoints(Point2f v1, float a);
Point2f dividePoint(Point2f v1, float a);
float ruleOfThree(float part, float whole);
Point2f multiplyPoint(Point2f p, float f);
Point2f averagePoints(std::vector<cv::Point2f> points);
Point2f averagePointsProportionally(Point2f a, Point2f b);
Point2f addPointsByPercentage(Point2f v1, float p1, Point2f v2, float p2);
Point2f averagePointsUsingWeights(Point2f p20, Point2f p80);
Point2f transformToSimpleDirection(Point2f point);

#endif /* MathHelper_hpp */
