//
//  Scene.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 13.11.2020.
//

#include "Scene.hpp"

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
