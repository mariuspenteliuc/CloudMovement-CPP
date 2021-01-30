//
//  Scene.cpp
//  CloudMovement-CPP
//
//  Created by Marius E. Penteliuc on 13.11.2020.
//

#include "Scene.hpp"

const float Scene::FIXED_RANGE = 25;
const float Scene::COLLISION_RANGE = 5;

/**
 * Returns the neighborhood of a boid given by a radius.
 *
 * @param boid the boid object for which the neighbors are found.
 * @param range a radius where to look for neighbors.
 * @return an array of boids that are in the neighborhood.
 */
std::vector<Boid> Scene::getNeighbors(Boid boid, float range) {
    std::vector<Boid> neighbors;
    for (auto& potentialNeighbor : boids) {
        if (Boid::getDistanceBetween(boid, potentialNeighbor) <= range) {
            neighbors.push_back(potentialNeighbor);
        }
    }
    return neighbors;
}

/**
 * Constructor that initializes a new scene to the specified parameters.
 *
 * @param sizeX the width of the scene.
 * @param sizeY the height of the scene.
 */
Scene::Scene(int sizeX, int sizeY) {
    this->sizeX = sizeX;
    this->sizeY = sizeY;
    this->scene = Mat::zeros( sizeY, sizeX, CV_8UC3 );
    this->framesShown = 0;
    this->saveSimulation = false;
}

/**
 * Creates a new boid objects and puts it in the scene at a random location.
 *
 * @return true after completion (beta)...
 */
bool Scene::addRandomBoid() {
    int initialSize = static_cast<int>(boids.size());
    boids.push_back(Boid::initWithinConstraint(sizeX, sizeY));
    if (initialSize < boids.size()) {
        return true;
    }
    return false;
}

bool Scene::addBoid(int x, int y, int margin) {
    int initialSize = static_cast<int>(boids.size());
    boids.push_back(Boid::initWithinConstraint(x, y, margin));
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

/**
 * Computes the center of mass for all the boids in the scene.
 *
 * @return a point representing the center of mass for all boids.
 */
cv::Point2f Scene::getCenterOfMass() {
    return getCenterOfMass(boids);
}

/**
 * Computes the center of mass for an array of boids.
 *
 * @param boids an array of boids for which the center of mass is to be computed.
 * @return a point representing the center of mass for given boids.
 */
cv::Point2f Scene::getCenterOfMass(std::vector<Boid> boids) {
    cv::Point2f centerOfMass;
    for (Boid boid : boids) {
        centerOfMass = addPoints(centerOfMass, boid.getPosition());
    }
    return dividePoint(centerOfMass, boids.size());
}

/**
 * Returns all the boid objects that exist in the scene.
 *
 * @return an array with all boid objects in the scene.
 */

std::vector<Boid> Scene::getAllBoids() {
    return boids;
}

/**
 * The center of mass rule influences a boid object to the center of mass of its neighboring objects.
 *
 * @param boid the boid to which the rule is applied.
 * @return a target point towards which the boid should move.
 */
Point2f Scene::rule1(Boid boid) {
    Point2f perceivedCenterOfMassForBoid;
    std::vector<Boid> neighborhood = getNeighbors(boid, FIXED_RANGE);
    perceivedCenterOfMassForBoid = getCenterOfMass(neighborhood);
    return perceivedCenterOfMassForBoid;
}

/**
 * The collision distance rule influences a boid object to not collide with neighboring objects.
 *
 * @param boid the boid to which the rule is applied.
 * @return a target point towards which the boid should move.
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

/**
 * Computes a target point towards a boid object should move based on the wind map in its neighborhood.
 * It averages the wind around its location and based.
 *
 * Probably the most important rule in this project.
 *
 * @param boid the boid to which the rule is applied.
 * @return a target point towards which the boid should move.
 */
Point2f Scene::ruleOfWind(Boid boid) {
    std::vector<Vector> winds = getWindVectors(boid.getPosition());
    Point2f averageWind = Vector::averageVectorDisplacement(winds);
    Point2f targetPoint = addPoints(boid.getPosition(), averageWind);
    return targetPoint;
}

/**
 * Updates the wind map used by the ruleOfWind() function.
 *
 * @param newWindMap an object that contains a flow of vectors.
 * @return true after completion (beta)...
 */
bool Scene::updateWindMap(cv::Mat newWindMap) {
    windMap = newWindMap;
    return true;
}

/// Updates the position of wind vectors by their velocity and direction of movement.
/// @param windMap the collection of vectors to be updated.
Mat Scene::updateWindPosition(Mat windMap) {
    Mat updatedWindMap;
    windMap.copyTo(updatedWindMap);
    for(int row = 0; row < windMap.rows; ++row) {
        for(int col = 0; col < windMap.cols; ++col) {
            Point2f& fxy = windMap.at<Point2f>(row, col);
            if (isnan(fxy.x) || isnan(fxy.y)) {
                cout << "We have NaN at (" << row << ", " << col << ")" << endl;
                if (isnan(fxy.x)) fxy.x = fxy.y;
                else fxy.y = fxy.x;
            }
            if ((col + fxy.x < 1918) && (row + fxy.y < 1078)) {
                //            cv::Point2f newPoint = cv::Point2f(cvRound(col+fxy.x), cvRound(row+fxy.y));
                cv::Point2f newPoint = cv::Point2f(ceil(col+fxy.x), ceil(row+fxy.y));
                if (newPoint.x >= 0 && newPoint.y >=0) {
                    updatedWindMap.at<Point2f>(newPoint) = fxy;
                }
                //            cout << "changing " << updatedwindmap.at<Point2f>(newPoint) << " into " << fxy << endl;
            }
        }
    }
    return updatedWindMap;
}


/// Averages the velocity and direction of neighboring groups of vectors.
/// The average is calculated between the vector and its neighbor with has the largest displacement value using weights of 20% and 80% respectively.
///
/// If radius extends beyond the boundary, missing neighbors will be ignored.
/// @param windMap the collection of vectors to be averaged.
/// @param radius the neighborhood window for the average, default is 5.
Mat Scene::averageWindMap(Mat windMap, int radius) {
    for(int row = 0; row < windMap.rows; ++row) {
        for(int col = 0; col < windMap.cols; ++col) {
            Point2f& currentPoint = windMap.at<Point2f>(row, col);
            //            following works for odd numbers
            int colStartIndex = col - (radius - 1)/2;
            if (colStartIndex < 0) colStartIndex = 0;
            int rowStartIndex = row - (radius - 1)/2;
            if (rowStartIndex < 0) rowStartIndex = 0;
            int colEndIndex = col + (radius - 1)/2;
            if (colEndIndex > 1920-1) colEndIndex = 1920-1;
            int rowEndIndex = row + (radius - 1)/2;
            if (rowEndIndex > 1080-1) rowEndIndex = 1080-1;

            Mat data = windMap.colRange(colStartIndex, colEndIndex + 1).rowRange(rowStartIndex, rowEndIndex + 1);
            Point2f fastestVector = Point2f(0,0);
            cv::Point2f average = Point2f(0, 0);
            for (int i = 0; i < data.rows; ++i) {
                for (int j = 0; j < data.cols; ++j) {
                    Point2f& fxy = data.at<Point2f>(i,j);
                    float currentDisplacement = Vector::getEuclidianDistance(Point2f(0,0), currentPoint);
                    float neighborDisplacement = Vector::getEuclidianDistance(Point2f(0,0), fxy);
                    if (neighborDisplacement >= currentDisplacement) {
                        fastestVector = fxy;
                    }
                    average = addPoints(average, fxy);
                }
            }
//            currentPoint = averagePointsUsingWeights(currentPoint, fastestVector);
            currentPoint = dividePoint(average, data.rows * data.cols);
//            50% weight on the fastest wind vector
            currentPoint = dividePoint(addPoints(currentPoint, fastestVector), 2);
        }
    }
    return windMap;
}

/**
 * Updates the wind map used by the ruleOfWind() function according to Boids Algorithm.
 *
 * @return true after completion (beta)...
 */
bool Scene::updateWindMapUsingBoids(int neighbordhoodRadius, string outputFolder) {
    updateWindPosition(getWindMap()).copyTo(this->windMap);
    averageWindMap(getWindMap(), neighbordhoodRadius);
    return true;
}

/**
 * Returns the wind map that is used by the ruleOfWind() function.
 * @return an object that contains a flow of vectors.
 */
cv::Mat Scene::getWindMap() {
    return windMap;
}

/**
 * Gets the wind vectors in the neighborhood of a location defined by FIXED_RANGE radius.
 *
 * @param location a point representing the center of the radius.
 * @return an array of Vector objects found near the location.
 */
std::vector<Vector> Scene::getWindVectors(cv::Point2f location) {
    std::vector<Vector> closeWindVectors;
    for (int i = -FIXED_RANGE; i <= FIXED_RANGE; ++i) {
        for (int j = -FIXED_RANGE; j <= FIXED_RANGE; ++j) {
            float yAxis = location.y + j;
            float xAxis = location.x + i;
            if (yAxis < 0) yAxis = 0;
            else if (yAxis > 1079) yAxis = 1919;
            if (xAxis < 0) xAxis = 0;
            else if (xAxis > 1919) xAxis = 1079;
            Point2f origin = windMap.at<cv::Point2f>(yAxis, xAxis);
            Vector vector = Vector::initWithDisplacementAndPosition(origin, location);
            closeWindVectors.push_back(vector);
        }
    }
    return closeWindVectors;
}

/**
 * Updates the simulation by applying rules to each boid's motion. These rules are combined to create target points towards each boid object will navigate.
 *
 * @return true after completion (beta)...
 */
bool Scene::updateSimulation() {
    for (Boid& boid : boids) {
        std::vector<Point2f> points;
        points.push_back(ruleOfWind(boid));
//        points.push_back(rule2(boid));
        boid.updateVelocity(points);
    }
    drawScene();
    return true;
}

/**
 * yeah... clears the scene using Mat::zeros
 */
void Scene::clearScene() {
    scene = Mat::zeros( sizeY, sizeX, CV_8UC3 );
}

/**
 * Starts the simulation by drawing the very first frame.
 *
 * @return true after completion (beta)...
 */
bool Scene::startSimulation() {
    saveSimulation = true;
    namedWindow("OpticalFlow", WINDOW_AUTOSIZE);
    drawScene();
    return true;
}

/**
 * Clears the scene of previous drawings, then redraws it using the updated boid positions.
 * After saving the scene to an image on disk, it will display the scene and wait for user key press to continue;
 */
void Scene::drawScene() {
    clearScene();
    for (Boid boid : boids) {
        const cv::Point point = cv::Point(cvRound(boid.getPosition().x), cvRound(boid.getPosition().y));
        circle(scene, point, .5, cv::Scalar(255, 255, 255, 0), cv::FILLED);
    }
//    FileHelper::writeFile("/Users/mariuspenteliuc/Assets/PhD/debug/debug_out/boids/boids_" + std::string(5 - to_string(framesSaved).length(), '0') + std::to_string(framesSaved) + ".jpg", scene);
    FileHelper::writeFile("/Volumes/Transfers/Experiments/day_window_1/test_1/simulated_clouds/boids_" + std::string(5 - to_string(framesSaved).length(), '0') + std::to_string(framesSaved) + ".jpg", scene);
    framesSaved++;
    imshow("OpticalFlow", scene);
    if (Scene::previewSimulation) {
        std::cout << "press any key to continue..." << std::endl;
        waitKey();
    }
}

/**
 * Runs the simulation by updating the scene a specified number of times.
 *
 * @param steps an integer representing how many times should the simulation run.
 * @return true after completion (beta)...
 */
bool Scene::runSimulation(int steps, bool preview) {
    this->previewSimulation = preview;
    for (int i = 0; i < steps; ++i) {
        updateSimulation();
    }
    return true;
}
