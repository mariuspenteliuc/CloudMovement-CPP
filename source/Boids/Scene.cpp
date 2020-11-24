//
//  Scene.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 13.11.2020.
//

#include "Scene.hpp"

const float Scene::FIXED_RANGE = 25;
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
    this->scene = Mat::zeros( sizeY, sizeX, CV_8UC3 );//Mat(sizeX, sizeY, CV_32F);
    this->framesShown = 0;
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
    return dividePoint(centerOfMass, boids.size());
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

Point2f Scene::ruleOfWind(Boid boid) {
    std::vector<Vector> winds = getWindVectors(boid.getPosition());
    Point2f averageWind = Vector::averageVectorDisplacement(winds);
    Point2f targetPoint = addPoints(boid.getPosition(), averageWind);
    return targetPoint;
}

bool Scene::updateWindMap(cv::Mat newWindMap) {
    windMap = newWindMap;
    return true;
}

cv::Mat Scene::getWindMap() {
    return windMap;
}

std::vector<Vector> Scene::getWindVectors(cv::Point2f location) {
    std::vector<Vector> closeWindVectors;
    for (int i = -FIXED_RANGE; i <= FIXED_RANGE; ++i) {
        for (int j = -FIXED_RANGE; j <= FIXED_RANGE; ++j) {
            float yAxis = location.y + j;
            float xAxis = location.x + i;
            if (yAxis < 0) {
                yAxis = 0;
            } else if (yAxis > 1079) {
                yAxis = 1919;
            }
            if (xAxis < 0) {
                xAxis = 0;
            } else if (xAxis > 1919) {
                xAxis = 1079;
            }
            Point2f origin = windMap.at<cv::Point2f>(yAxis, xAxis);
//            Vector vector = Vector(origin, location);
            Vector vector = Vector::initWithDisplacementAndPosition(origin, location);
            closeWindVectors.push_back(vector);
        }
    }
    return closeWindVectors;
}

// TODO: create a startSimulation() and a stopSimulation() function

bool Scene::update() {
//    TODO: rewrite this function to apply rules, update positions, and draw boids on scene;
    int i = 0;
//    TODO: refactor contents of this for into a function applyRules(Boid boid) which should be able to apply each rule individually
    for (Boid& boid : boids) {
        std::vector<Point2f> points;
        Point2f p1 = ruleOfWind(boid);
        points.push_back(p1);
//        Point2f p2 = rule2(boid);
//        points.push_back(p2);
//        Point2f p3 = rule3(boid);
//        points.push_back(p3);
        boid.updateVelocity(points);
    }
    drawScene();
    return true;
}

void Scene::clearScene() {
    scene = Mat::zeros( sizeY, sizeX, CV_8UC3 );
}

bool Scene::startSimulation() {
    namedWindow("OpticalFlow", WINDOW_AUTOSIZE);
    drawScene();
    return true;
}

void Scene::drawScene() {
    clearScene();
    int k = 0;
    for (Boid boid : boids) {
        const cv::Point point = cv::Point(cvRound(boid.getPosition().x), cvRound(boid.getPosition().y));
        circle(scene, point, .5, cv::Scalar(255, 255, 255, 0), cv::FILLED);
    }


//    imshow("Image " + std::to_string(framesShown++), scene);
    imshow("OpticalFlow", scene);
    OpticalFlowService::saveImageToDisk("/Users/mariuspenteliuc/Assets/PhD/debug/debug_out/boids/boids_" + std::to_string(framesSaved++) + ".jpg", scene);
//    std::cout << "press any key to continue..." << std::endl;
//    waitKey();
//    for(int y = 0; y < cflowmap.rows; y += step)
//        for(int x = 0; x < cflowmap.cols; x += step) {
//            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
//            const cv::Point roundedPoint = cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y));
//            line(cflowmap, cv::Point(x,y), roundedPoint, color, 1, cv::LINE_AA);
//            circle(cflowmap, cv::Point(x,y), 0.5, color, cv::FILLED);
//        }
}

