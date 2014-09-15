#ifndef ROBOT_H
#define ROBOT_H
#include <geometry.h>
#include <vector>
#include <iostream>

using namespace std;

enum rangeFinder
{
    rangeFinderN,
    rangeFinderNE,
    rangeFinderNW,
    rangeFinderE,
    rangeFinderW,
    rangeFinderS
};

enum radar
{
    radarN,
    radarE,
    radarS,
    radarW
};

class robot
{
public:
    pointTheta position;
    double radius;
    double rangeFinderSensor[6];
    bool radarSensor[4];
    robot()
    {

    }
    ~robot()
    {
    }

    void updateSensors(point goal, std::vector<lineSegment> &walls)
    {
        double radarValue = position.getAngleDif(goal);

        radarSensor[radarN] = (radarValue >= -M_PI_4 && radarValue < M_PI_4);
        radarSensor[radarE] = (radarValue >= M_PI_4 && radarValue < 3*M_PI_4);
        radarSensor[radarW] = (radarValue >= -3*M_PI_4 && radarValue < -M_PI_4);
        radarSensor[radarS] = ((radarValue < -3*M_PI_4 && radarValue <= 0) || (radarValue <= M_PI && radarValue >= 3*M_PI_4));
//        cout << "Radar: " << radarSensor[radarN] << "," << radarSensor[radarN] << "," << radarSensor[radarE] << "," << radarSensor[radarS] << "," << radarSensor[radarW] << endl;

//        cout << "RangeFinder: ";
        for(int range = rangeFinderN; range <= rangeFinderS; range++)
        {
            double angleRange = 0.0;
            rangeFinder foo = static_cast<rangeFinder>(range);

            switch (foo) {
            case rangeFinderN:
                break;
            case rangeFinderNE:
                angleRange = M_PI_4;
                break;
            case rangeFinderNW:
                angleRange = -M_PI_4;
                break;
            case rangeFinderE:
                angleRange = M_PI_2;
                break;
            case rangeFinderW:
                angleRange = -M_PI_2;
                break;
            case rangeFinderS:
                angleRange = M_PI;
                break;
            default:
                cerr << "No range" << endl;
                break;
            }

            double minDist = 100.0;
            point a;
            for (int i = 0; i < walls.size(); ++i) {
                if (walls.at(i).intersection(position, remainder(position.theta + angleRange,2*M_PI), a))
                {
                    double tmpDis = position.distance(a);
                    if(tmpDis < minDist)
                    {
                        minDist = tmpDis;
                    }
                }
            }
            rangeFinderSensor[foo] = minDist;
//            cout << foo << " " << rangeFinderSensor[foo] << ",   ";
        }
//        cout << endl;
    }
};



#endif
