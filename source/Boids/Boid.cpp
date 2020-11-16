//
//  Boid.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 13.11.2020.
//

#include "Boid.hpp"
#include <cstdlib>


Boid::Boid() {
    id = rand() % 4294967295;
    
}

bool Boid::operator == (const Boid &ref) const {
    return(this->id == ref.getID());
}

const long Boid::getID() const {
    return(this->id);
}

Boid Boid::initWithinConstraint(int maxX, int maxY) {
    Boid boid = Boid();
    boid.position = cv::Point2f((rand() % (maxX * 1000))/100.0f, (rand() % (maxY * 1000))/100.0f);
    return boid;
}

float Boid::getDistanceBetween(Boid firstBoid, Boid secondBoid) {
    return sqrt(pow((secondBoid.position.x - firstBoid.position.x), 2.0f) + pow((secondBoid.position.y - firstBoid.position.y), 2.0f));
}

cv::Point2f Boid::getDistanceTo(Boid boid) {
    float displacementX = boid.position.x - this->position.x;
    float displacementY = boid.position.y - this->position.y;
    return cv::Point2f(displacementX, displacementY);
}
