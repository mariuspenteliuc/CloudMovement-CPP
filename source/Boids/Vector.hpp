//
//  Vector.hpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 13.11.2020.
//

#ifndef Vector_hpp
#define Vector_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "MathHelper.hpp"

class Vector {
private:
    cv::Point origin;
    cv::Point displacement;

    /// Computes the average displacement for a set of vectors. This will return the displacement as a `Point` object, containing the average displacement on each axis.
    /// - Parameter vectors: the set of `Vectors` to be averaged
    /// - Returns: a `Point` containing the average displacement
    static void initWithOriginAndPosition(cv::Point origin, cv::Point position);
    static void initWithDisplacementAndPosition(cv::Point displacement, cv::Point position);
    static void initWithOriginAndDisplacement(cv::Point origin, cv::Point displacement);
public:
    static cv::Point2f averageVectorDisplacement(std::vector<Vector> vectors);
    Vector();
    static float getEuclidianDistance(cv::Point2f origin, cv::Point2f destination);
    Vector(cv::Point2f origin, cv::Point2f position);
    cv::Point2f getOrigin();
    cv::Point2f getDisplacement();
};

#endif /* Vector_hpp */
