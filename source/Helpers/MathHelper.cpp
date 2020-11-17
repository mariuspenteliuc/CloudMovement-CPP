//
//  MathHelper.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 16.11.2020.
//

#include "MathHelper.hpp"

Point2f addPoints(Point2f v1, Point2f v2) {
    float x = v1.x + v2.x;
    float y = v1.y + v2.y;
    return Point2f(x, y);
}
Point2f removePoints(Point2f v1, Point2f v2) {
    float x = v1.x - v2.x;
    float y = v1.y - v2.y;
    return Point2f(x, y);
}
Point2f removePoints(Point2f v1, float a) {
    float x = v1.x - a;
    float y = v1.y - a;
    return Point2f(x, y);
}
Point2f dividePoint(Point2f v1, float a) {
    float x = v1.x / a;
    float y = v1.y / a;
    return Point2f(x, y);
}

Point2f multiplyPoint(Point2f p, float f) {
    float x = p.x * f;
    float y = p.y * f;
    return Point2f(x, y);
}

float ruleOfThree(float part, float whole) {
    return part / whole;
}

Point2f averagePoints(std::vector<cv::Point2f> points) {
    cv::Point2f average;
    for (Point2f point: points) {
        average = addPoints(average, point);
    }
    return dividePoint(average, points.size());
}
