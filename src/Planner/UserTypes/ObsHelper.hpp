#pragma once

#include "ReachabilitySet.hpp"
#include <list>
#include <vector>
#include "Planner/UserStructs/obstacle3D.h"
#include "Planner/UserStructs/point2D.h"
#include "Planner/UserTypes/ReachabilitySet.hpp"
#include "Planner/UserStructs/PlaneStateSim.h"

namespace UasCode {

class ObsHelper{

public:
    ObsHelper(const UserStructs::obstacle3D& obs3d, const double _dt);

    inline void SetDt(const double _dt){ this->dt = _dt;}

    bool InSet(const UserStructs::PlaneStateSim& state);
    bool InSet3D(double t, double x, double y, double z);
    bool NoSet(const UserStructs::PlaneStateSim& state);

private:
    uint32_t address;

    static bool if_print;
    UasCode::ReachabilitySet reach_set;

    /* ObsData */
    struct ObsData{
        double x1, x2, x3;
        double head_xy;
        double speed;
        double v_vert;
        double t;

        ObsData(UserStructs::obstacle3D& obs3d):
            x1(obs3d.x1),x2(obs3d.x2),x3(obs3d.x3),
            head_xy(obs3d.head_xy),speed(obs3d.speed),
            v_vert(obs3d.v_vert),t(obs3d.t){ }
    };
    /* ObsData end*/

    double dt;

    std::vector< UserStructs::SetPointsVh > sets;

    //functions
    void CreateSetPoints();

    UserStructs::SetPointsVh GetSetPointsVh(double _t);

};

}
