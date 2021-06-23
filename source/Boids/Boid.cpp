//
//  Boid.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 13.11.2020.
//

#include "Boid.hpp"

Boid::Boid() {
    id = rand() % 4294967295;
    
}

bool Boid::operator == (const Boid &ref) const {
    return(this->id == ref.getID());
}
std::string const&  Boid::to_str() const {
    static std::string value = "";
    value.append("Boid #");
    value.append(std::to_string(id).c_str());
    value.append(" (");
    value.append(std::to_string(position.x).c_str());
    value.append(", ");
    value.append(std::to_string(position.y).c_str());
    value.append(")\0");
    return value;
}

bool Boid::updateVelocity(std::vector<cv::Point2f> points) {
    cv::Point2f average = Point2f(0, 0);
    for (cv::Point2f point : points) {
        average = addPoints(average, point);
    }
    average = dividePoint(average, points.size());
    velocity = Vector(position, average);
//    if (abs(velocity.getDisplacement().x) > 1 && abs(velocity.getDisplacement().y) > 1) {
        updatePosition();
//    } else {
//
//    }
    return true;
}

bool Boid::updatePosition() {
    position = addPoints(position, velocity.getDisplacement());
    return true;
}
bool Boid::updatePosition(std::vector<cv::Point2f> points) {
    cv::Point2f pt = averagePoints(points);
    velocity = Vector(position, pt);
    position = pt;
    return true;
}

std::ostream& operator<<(std::ostream& os, const Boid& boid) {
    os << boid.to_str();
    return os;
}

const long Boid::getID() const {
    return(this->id);
}

Boid Boid::initWithinConstraint(int maxX, int maxY) {
    Boid boid = Boid();
    boid.position = cv::Point2f((rand() % (maxX * 100))/100.0f, (rand() % (maxY * 100))/100.0f);
    boid.velocity = Vector(boid.getPosition(), boid.getPosition());
    return boid;
}

Boid Boid::initWithinConstraint(int x, int y, int margin) {
    Boid boid = Boid();
    boid.position = cv::Point2f( x + (rand() %margin)/100.0f - 1/2, y + (rand() % margin)/100.0f - 1/2);
    boid.velocity = Vector(boid.getPosition(), boid.getPosition());
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

cv::Point2f Boid::getPosition() {
    return position;
}
