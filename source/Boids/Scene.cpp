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
/**
 * Sum numbers in a vector.
 *
 * @param values Container whose values are summed.
 * @return sum of `values`, or 0.0 if `values` is empty.
 */
Point2f Scene::rule1(Boid boid) {
    Point2f perceivedCenterOfMassForBoid;
    std::vector<Boid> neighborhood = getNeighbors(boid, FIXED_RANGE);
    perceivedCenterOfMassForBoid = getCenterOfMass(neighborhood);
    return perceivedCenterOfMassForBoid;
}
/**
 * Method name: name
 * Description: returns name
 * Parameters: none
 */
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

bool Scene::updateWindMap(cv::Mat newWindMap) {
    windMap = newWindMap;
    return true;
}

cv::Mat Scene::getWindMap() {
    return windMap;
}

bool Scene::update() {
    int i = 0;
    for (Boid boid : boids) {
        std::vector<Point2f> points;
        Point2f p1 = rule1(boid);
        points.push_back(p1);
//        Point2f p2 = rule2(boid);
//        points.push_back(p2);
//        Point2f p3 = rule3(boid);
//        points.push_back(p3);
        boid.updateVelocity(points);
//        if (i%500 != 0) {
//            i++;
//            continue;
//        } else {
//            std::cout << i++ << std::endl;
//        }
    }
    return true;
}
