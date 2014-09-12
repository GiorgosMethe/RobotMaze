#include <iostream>
#include <geometry.h>
#include <fstream>
#include <unistd.h>
#include <string>
#include<opencv2/opencv.hpp>
#include "boost/algorithm/string/split.hpp"
#include "boost/algorithm/string/classification.hpp"


using namespace std;

void drawMaze(point robot, point goal, vector<lineSegment> walls)
{
    int rescale = 1000;
    cv::Mat base(rescale, rescale, CV_64FC3);
    base=cv::Scalar(255, 255, 255);
    for (int i = 0; i < walls.size(); ++i) {
        cv::Point2d p1(walls.at(i).start.x * rescale, walls.at(i).start.y * rescale);
        cv::Point2d p2(walls.at(i).end.x * rescale, walls.at(i).end.y * rescale);
        cv::line(base, p1, p2, CV_RGB(0, 0, 0), 5, CV_AA);
    }
    cv::circle(base, cv::Point(robot.x * rescale, robot.y * rescale), 10, CV_RGB(200, 0, 0), -1, CV_AA);
    cv::circle(base, cv::Point(goal.x * rescale, goal.y * rescale), 10, CV_RGB(0, 0, 200), -1, CV_AA);
    cv::namedWindow("maze", CV_NORMAL);
    cv::imshow("maze", base);
    cv::waitKey(10);
}

void readMaze(point &robot, point &goal, vector<lineSegment> &walls)
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
                robot.x = atof(tokens[1].c_str());
                robot.y = atof(tokens[2].c_str());
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

int main()
{

    point robot;
    point goal;
    vector<lineSegment> walls;

    readMaze(robot, goal, walls);

    for (int var = 0; var < 10000; ++var) {

        drawMaze(robot, goal, walls);
    }
    return 0;
}

