#pragma once

#include <vector>
#include "Planner/UserStructs/point2D.h"
#include "Planner/Utils/PointInPoly.h"
#include <stdexcept>
#include <fstream>
#include <iomanip>

namespace UserStructs {

struct SetPointsVh{
    std::vector< point2D > points_2d;
    double h_low;
    double h_high;

    SetPointsVh( const std::vector<point2D> _pts_2d, const double _h_low, const double _h_high):points_2d(_pts_2d),h_low(_h_low),h_high(_h_high){

    }

    bool InSetPt(double x, double y, double z){

      if( points_2d.size() < 3){
          throw std::runtime_error("No enough polygon points < 3");
      }

      return Utils::PointInPoly(points_2d,x,y) && z < h_high && z > h_low;
    }

    void PrintSet(const char* filename)
    {
        std::ofstream file( filename );
        for(int i=0; i!= points_2d.size(); ++i){
            file << std::setprecision(4) << std::fixed
                    <<  points_2d[i].x << " " << points_2d[i].y << '\n';
        }
    }

};

}
