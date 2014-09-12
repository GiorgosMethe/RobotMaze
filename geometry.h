#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <math.h>


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
    float distance(point _point)
    {
        return sqrt(pow((x - _point.x),2) + pow((y - _point.y),2));
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
        double angle = atan2(end.x-start.x,end.y-start.y);
        angle = angle * (180 / M_PI);
        if( angle < 0 )
            angle += 180;
        return angle;
    }
};


#endif // GEOMETRY_H
