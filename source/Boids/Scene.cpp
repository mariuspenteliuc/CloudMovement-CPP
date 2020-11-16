//
//  Scene.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 13.11.2020.
//

#include "Scene.hpp"
#include "MathHelper.hpp"

const float Scene::FIXED_RANGE = 20;
const float Scene::COLLISION_RANGE = 5;

std::vector<Boid> Scene::getNeighbors(Boid boid, float range) {
    std::vector<Boid> neighbors;
    for (auto& potentialNeighbor : boids) {
        if (Boid::getDistanceBetween(boid, potentialNeighbor) <= range) {
            neighbors.push_back(potentialNeighbor);
        }
    }
    return neighbors;
}

Scene::Scene(int sizeX, int sizeY) {
    this->sizeX = sizeX;
    this->sizeY = sizeY;
}

bool Scene::addRandomBoid() {
    int initialSize = static_cast<int>(boids.size());
    boids.push_back(Boid::initWithinConstraint(sizeX, sizeY));
    if (initialSize < boids.size()) {
        return true;
    }
    return false;
}
int Scene::getBoidsCount() {
    return static_cast<int>(boids.size());
}

int Scene::getSizeX() {
    return sizeX;
}

int Scene::getSizeY() {
    return sizeY;
}

cv::Point2f Scene::getCenterOfMass() {
    return getCenterOfMass(boids);
}

cv::Point2f Scene::getCenterOfMass(std::vector<Boid> boids) {
    cv::Point2f centerOfMass;
    for (Boid boid : boids) {
        centerOfMass = addPoints(centerOfMass, boid.getPosition());
    }
    return dividePoints(centerOfMass, boids.size());
}

std::vector<Boid> Scene::getAllBoids() {
    return boids;
}

Point2f Scene::rule1(Boid boid) {
    Point2f perceivedCenterOfMassForBoid;
    std::vector<Boid> neighborhood = getNeighbors(boid, FIXED_RANGE);
    perceivedCenterOfMassForBoid = getCenterOfMass(neighborhood);
    return perceivedCenterOfMassForBoid;
}

Point2f Scene::rule2(Boid boid) {
    Point2f collisionDistance;
    std::vector<Boid> neighborhood = getNeighbors(boid, FIXED_RANGE);
    for (int i = 0; i < neighborhood.size(); ++i) {
        float proximity = Boid::getDistanceBetween(boid, neighborhood[i]);
        if (abs(proximity) < COLLISION_RANGE) {
            collisionDistance = removePoints(collisionDistance, proximity);
        }
    }
    return collisionDistance;
}
