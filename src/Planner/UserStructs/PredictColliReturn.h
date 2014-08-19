#pragma once

namespace UserStructs{

struct PredictColliReturn{
    bool colli;
    int seq_colli; //predicted collision happend between (seq_colli)th and (seq_colli+1)th waypoints
    double time_colli;//time of predicted collision from now on
    double dis_colli_2d;
    double dis_colli_hgt;

    //constructor
    PredictColliReturn();
};

}
