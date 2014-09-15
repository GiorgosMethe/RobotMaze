#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <math.h>
#include <iostream>
#include <cmath>
using namespace std;

double d2r(double d)
{
    return d * M_PI / 180;
}

double r2d(double r)
{
    return r * 180/ M_PI;
}

class point
{
public:
    double x;
    double y;
    point(){}
    point(double _x, double _y) {
        x = _x;
        y = _y;
    }
    double distance(point _point)
    {
        return sqrt(pow((x - _point.x),2) + pow((y - _point.y),2));
    }

    double getAngle(point _point)
    {
        return atan2(_point.x - x, _point.y - y);
    }

    point pointToAngleDis(double distance, double angle)
    {
        std::cout << angle << std::endl;
        point tmp;
        tmp.x = x + (sin(angle) * distance);
        tmp.y = y + (cos(angle) * distance);
        return tmp;
    }
};

class pointTheta : public point
{
public:
    double theta;
    pointTheta(){}
    pointTheta(double _x, double _y, double _theta)
    {
        x = _x;
        y = _y;
        theta = _theta;
    }
    point pointToDis(double distance)
    {
        point tmp;
        tmp.x = x + (sin(theta) * distance);
        tmp.y = y + (cos(theta) * distance);
        return tmp;
    }

    double getAngleDif(point _point)
    {
        return this->getAngle(_point) - theta;
    }
};


class lineSegment
{
public:
    point start;
    point end;
    lineSegment() {}
    lineSegment(point _start, point _end) {
        start = _start;
        end = _end;
    }

    double getAngle()
    {
        return start.getAngle(end);
    }

    bool intersection(point _point, double _angle, point &ret)
    {
        point orientation;
        pointTheta tmp;
        tmp.x =_point.x;
        tmp.y =_point.y;
        tmp.theta = _angle;
        orientation = tmp.pointToDis(1.0);

        double x1 = start.x, x2 = end.x;
        double y1 = start.y, y2 = end.y;
        double x3 = _point.x, x4 = orientation.x;
        double y3 = _point.y, y4 = orientation.y;
        double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        if (d == 0) return NULL;

        double pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
        double x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
        double y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;

        if ( x < min(x1, x2) - 10e-5 || x > (max(x1, x2) + 10e-5) )return false;
        if ( y < min(y1, y2) - 10e-5 || y > (max(y1, y2) + 10e-5)  ) return false;

        ret.x = x;
        ret.y = y;

        if(abs(_point.getAngle(ret) - _angle) > 10e-5) return false;
        return true;
    }

    double pointDist(point _point, bool segment=false)
    {
        if(segment)
        {
            double A = _point.x - start.x;
            double B = _point.y - start.y;
            double C = end.x - start.x;
            double D = end.y - start.y;

            double dot = A * C + B * D;
            double len_sq = C * C + D * D;
            double param = dot / len_sq;
            double xx,yy;
            if(param < 0)
            {
                xx = start.x;
                yy = start.y;
                return 100;
            }
            else if(param > 1)
            {
                xx = end.x;
                yy = end.y;
                return 100;
            }
            else
            {
                xx = start.x + param * C;
                yy = start.y + param * D;
            }
            point tmp; tmp.x = xx; tmp.y = yy;
            return _point.distance(tmp);
        }
        double normalLength = hypot(end.x - start.x, end.y - start.y);
        return fabs((_point.x - start.x) * (end.y - start.y) - (_point.y - start.y) * (end.x - start.x)) / normalLength;
    }

};


#endif // GEOMETRY_H
