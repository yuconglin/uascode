#pragma once

#include "Planner/UserStructs/point2D.h"
#include "Planner/UserStructs/obstacle3D.h"
#include <vector>

namespace UasCode{

class ReachabilitySet{

public:
    //constructor
    ReachabilitySet(double _spd, double _omiga, double _r, double _hd):spd(_spd),omiga(_omiga),r(_r), hd(_hd) {
       rho = spd / omiga;
    }

    ReachabilitySet(const UserStructs::obstacle3D& _obs);

    //public functions
    void GetSet( int num, double t );
    bool InSet( double x, double y );
    bool InSet3( double x, double y, double z);
    void OutputSet( const char* filename );
private:
    //set default constructor to default
    ReachabilitySet():spd(0.),omiga(0.),r(0.){
        //no operations
    }
    //member variables
    double spd;
    double omiga;
    double rho;
    double r;
    double hr;
    double hd; //heading defined w.r.t x-axial
    double x0;
    double y0;
    double t0;
    double z0;
    double vert;
    double up_the;
    //set components
    std::vector<UserStructs::point2D> set_points;
    double h_low;
    double h_high;

    //member functions
    //x, y
    double x1(double the, double t);
    double y1(double the, double t);
    double x2(double the, double t);
    double y2(double the, double t);
    double x3(double the, double t);
    double y3(double the, double t);
    double x4(double the, double t);
    double y4(double the, double t);
    double x(double the, double t);
    double y(double the, double t);
    
};

}
