//
//  Scene.hpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 13.11.2020.
//

#ifndef Scene_hpp
#define Scene_hpp

#include <stdio.h>
#include "Boid.hpp"

class Scene {
private:
    std::vector<Boid> boids;
    int sizeX, sizeY;

    bool addBoid(Boid boid);
    cv::Point2f getCenterOfMass();
    cv::Point2f getCenterOfMass(std::vector<Boid> boids);

    cv::Point2f rule1(Boid boid);
    cv::Point2f rule2(Boid boid);
    cv::Point2f rule3(Boid boid);
public:
    Scene(int sizeX, int sizeY);
    int getSizeX();
    int getSizeY();
    bool addRandomBoid();
    int getBoidsCount();
    std::vector<Boid> getNeighbors(Boid boid, float range);
    std::vector<Boid> getAllBoids();
};

#endif /* Scene_hpp */
