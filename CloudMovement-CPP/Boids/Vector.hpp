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

class Vector {
private:
    cv::Point origin;
    cv::Point displacement;

    /// Computes the average displacement for a set of vectors. This will return the displacement as a `Point` object, containing the average displacement on each axis.
    /// - Parameter vectors: the set of `Vectors` to be averaged
    /// - Returns: a `Point` containing the average displacement
    static cv::Point averageVectorDisplacement(std::vector<Vector>);
    static void initWithOriginAndPosition(cv::Point origin, cv::Point position);
    static void initWithDisplacementAndPosition(cv::Point displacement, cv::Point position);
    static void initWithOriginAndDisplacement(cv::Point origin, cv::Point displacement);
public:
    Vector();
};

#endif /* Vector_hpp */
