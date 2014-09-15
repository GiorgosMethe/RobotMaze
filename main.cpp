#include <iostream>
#include <geometry.h>
#include <robot.h>
#include <fstream>
#include <unistd.h>
#include <randomGenerator.h>
#include <string>
#include<opencv2/opencv.hpp>
#include "boost/algorithm/string/split.hpp"
#include "boost/algorithm/string/classification.hpp"
#include "boost/timer.hpp"


using namespace std;

void drawMaze(robot _robot, point goal, vector<lineSegment> walls)
{
    int rescale = 1000;
    cv::Mat base(rescale, rescale, CV_64FC3);
    base=cv::Scalar(255, 255, 255);
    //walls
    for (int i = 0; i < walls.size(); ++i) {
        cv::Point2d p1(walls.at(i).start.x * rescale, 1000 - walls.at(i).start.y * rescale);
        cv::Point2d p2(walls.at(i).end.x * rescale, 1000 - walls.at(i).end.y * rescale);
        cv::line(base, p1, p2, CV_RGB(0, 0, 0), 5, CV_AA);
    }

    // robot
    cv::circle(base, cv::Point(_robot.position.x * rescale, 1000 - _robot.position.y * rescale), _robot.radius * rescale, CV_RGB(200, 0, 0), -1, CV_AA);
    // robot orientation
    cv::Point2d robotCenter(_robot.position.x *rescale,  1000 - _robot.position.y * rescale);
    point orientation; orientation = _robot.position.pointToDis(_robot.radius);
    cv::Point2d robotOr(orientation.x * rescale, 1000 - orientation.y * rescale);

    cv::line(base, robotCenter, robotOr, CV_RGB(0, 0, 0), 5, CV_AA);
    // goal
    cv::circle(base, cv::Point(goal.x * rescale, 1000 - goal.y * rescale), 20, CV_RGB(0, 0, 200), -1, CV_AA);

    cv::namedWindow("maze", CV_NORMAL);
    cv::imshow("maze", base);
    cv::waitKey(10);
}

void readMaze(robot &_robot, point &goal, vector<lineSegment> &walls)
{
    string line;
    ifstream myfile ("maze.txt");
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            std::vector<std::string> tokens;
            boost::split(tokens, line, boost::is_any_of(" "));
            if(tokens[0] == "r"){
                _robot.position.x = atof(tokens[1].c_str());
                _robot.position.y = atof(tokens[2].c_str());
                _robot.position.theta = d2r(atof(tokens[3].c_str()));
                _robot.radius = atof(tokens[4].c_str());
            }else if(tokens[0] == "g"){
                goal.x = atof(tokens[1].c_str());
                goal.y = atof(tokens[2].c_str());
            }else if(tokens[0] == "w"){
                lineSegment tmp;
                tmp.start.x = atof(tokens[1].c_str());
                tmp.start.y = atof(tokens[2].c_str());
                tmp.end.x = atof(tokens[3].c_str());
                tmp.end.y = atof(tokens[4].c_str());
                walls.push_back(tmp);
            }
        }
        myfile.close();
    }
    else
    {
        cerr << "nofile" << endl;
    }
}


void driveRandom(robot &_robot, vector<lineSegment> walls)
{
    _robot.position.theta -= d2r(50 * unifRand() - 25);


    double distance = 0.01;

    double minDist = 100.0;
    double angle = (distance > 0) ? 0 : M_PI;
    for (int i = 0; i < walls.size(); ++i) {
        point a;
        if (walls.at(i).intersection(_robot.position, remainder(_robot.position.theta + angle, 2*M_PI), a))
        {
            double tmpDis = _robot.position.distance(a);
            if(tmpDis < minDist)
            {
                minDist = tmpDis;
            }
        }
    }

    distance = min(distance, (minDist - _robot.radius));

    point newPosition = _robot.position.pointToDis(distance);
    bool isValid = true;
    for (int i = 0; i < walls.size(); ++i) {
        if ((walls.at(i).pointDist(newPosition, true)) < _robot.radius) isValid = false;
    }
    if(isValid)
    {
        _robot.position.x = newPosition.x;
        _robot.position.y = newPosition.y;
    }
    else
    {
        double dx = newPosition.x - _robot.position.x;
        double dy = newPosition.y - _robot.position.y;
        point newPosition; newPosition.x = _robot.position.x + dx; newPosition.y = _robot.position.y;
        bool isValid = true;
        for (int i = 0; i < walls.size(); ++i) {
            if ((walls.at(i).pointDist(newPosition, true)) < _robot.radius) isValid = false;
        }
        if(isValid)
        {
            _robot.position.x = newPosition.x;
            _robot.position.y = newPosition.y;
        }
        else
        {
            isValid = true;
            point newPosition; newPosition.y = _robot.position.y + dy; newPosition.x = _robot.position.x;
            bool isValid = true;
            for (int i = 0; i < walls.size(); ++i) {
                if ((walls.at(i).pointDist(newPosition, true)) < _robot.radius) isValid = false;
            }
            if(isValid)
            {
                _robot.position.x = newPosition.x;
                _robot.position.y = newPosition.y;
            }
        }
    }
    _robot.position.theta = remainder(_robot.position.theta, 2*M_PI);
}

int main()
{
    seed();
    for (int var = 0; var < 100000000; ++var) {
        cout << var << endl;
        robot* _robot = new robot();
        point goal;
        vector<lineSegment> walls;
        readMaze(*_robot, goal, walls);
        boost::timer timer;
        double simulationTime = 0;
        for (int var = 0; var < 10000; ++var) {
            _robot->updateSensors(goal, walls);
            driveRandom(*_robot, walls);
//            //        drawMaze(_robot, goal, walls);
        }
        std::cout << "Simulation took: " << timer.elapsed() << " Seconds to complete.\n" << std::endl;
        delete _robot;
        walls.clear();
    }
    return 0;
}

