#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include "stdlib.h"

#include "common/Utils/FindPath.h"
#include "UserStructs/obstacle3D.h"

void IntroPolate(std::vector<UserStructs::obstacle3D>& obs_vec,int cnt0,int cnt1);

//to extropolate adsb data for obstacles
int main(int argc, char** argv)
{
    std::string obs_file = Utils::FindPath()+"records/"+argv[1];
    std::string obs_file_whole = Utils::FindPath()+"records/tt_"+argv[1];
    std::ifstream f_obs(obs_file.c_str());
    std::ofstream fw_obs(obs_file_whole.c_str());
    std::vector<UserStructs::obstacle3D > obs_vec;

    int method= 0; //0 for extropolation, 1 for intropolation
    if(argc > 2)
        method = atoi(argv[2]);

    //get one obstacle
    UserStructs::obstacle3D obs_pre;
    std::string line;
    int line_count=0;

    while(getline(f_obs,line))
    {
        UserStructs::obstacle3D obs;
        std::istringstream iss(line);

        iss >> obs.address
               >> obs.x1
               >> obs.x2
               >> obs.x3
               >> obs.head_xy
               >> obs.speed
               >> obs.v_vert
               >> obs.t >> obs.r >> obs.hr;
        //obs.head_xy= obs.head_xy*M_PI/180.;
        //see if extropolate is needed
        if(line_count> 0 && obs.t - obs_pre.t > 1.0){
            int cnt = (int)(obs.t-obs_pre.t);
            for(int i=0;i!=cnt-1;++i){
                UserStructs::obstacle3D obs_sub;
                obs_sub.address= obs_pre.address;
                if(method==0){
                    double Dt= i+1;
                    obs_sub.x1= obs_pre.x1+ obs_pre.speed*cos(obs_pre.head_xy)*Dt;
                    obs_sub.x2= obs_pre.x2+ obs_pre.speed*sin(obs_pre.head_xy)*Dt;
                    obs_sub.x3= obs_pre.x3+ obs_pre.v_vert*Dt;
                    obs_sub.head_xy= obs_pre.head_xy;
                    obs_sub.speed= obs_pre.speed;
                    obs_sub.v_vert= obs_pre.v_vert;
                    obs_sub.t= obs_pre.t+Dt;
                }
                else if(method==1){
                    double lambda= 1.0*(i+1)/cnt;
                    obs_sub.x1= obs_pre.x1*(1.-lambda)+ obs.x1*lambda;
                    obs_sub.x2= obs_pre.x2*(1.-lambda)+ obs.x2*lambda;
                    obs_sub.x3= obs_pre.x3*(1.-lambda)+ obs.x3*lambda;
                    obs_sub.head_xy= obs_pre.head_xy*(1.-lambda)+ obs_pre.head_xy*lambda;
                    obs_sub.speed= obs_pre.speed*(1.-lambda)+ obs.speed*lambda;
                    if(obs_sub.speed == 0)
                       obs_sub.speed= obs_pre.speed;
                    obs_sub.v_vert= obs_pre.v_vert*(1.-lambda)+ obs.v_vert*lambda;
                    if(obs_sub.v_vert == 0)
                       obs_sub.v_vert = obs_pre.v_vert;

                    obs_sub.t= obs_pre.t+i+1;
                }
                else{}

                obs_sub.r= obs_pre.r;
                obs_sub.hr= obs_pre.hr;
                obs_vec.push_back(obs_sub);
            }
        }

        obs_vec.push_back(obs);
        obs_pre = obs;
        ++line_count;
    }

    int count=0, cnt=0, cnt1=0;
    bool if_repeat= false;
    while(count!= obs_vec.size())
    {
        if(obs_vec[count].x1 == obs_vec[count+1].x1 && obs_vec[count].x2 == obs_vec[count+1].x2 && obs_vec[count].x3 == obs_vec[count+1].x3)
        {
           if(!if_repeat){
               cnt= count;
               if_repeat= true;
           }
        }
        else
        {
            if(if_repeat){
                cnt1= count+1;
                if_repeat= false;
                //std::cout<<"cnt:"<< cnt<<","<<"cnt1:"<< cnt1<< "\n";
                IntroPolate(obs_vec,cnt,cnt1);
            }
        }
        ++count;
    }

    //size()-1 for the zero line at the bottom
    for(int i=0;i!=obs_vec.size();++i){
        UserStructs::obstacle3D obs= obs_vec[i];
        fw_obs << obs.address << " "
               << obs.x1 << " "
               << obs.x2 << " "
               << obs.x3 << " "
               << obs.head_xy  << " "
               << obs.speed << " "
               << obs.v_vert << " "
               << std::setprecision(4) << std::fixed
               << obs.t << " "
               << obs.r << " "
               << obs.hr
               << "\n";
    }

    return 0;
}

void IntroPolate(std::vector<UserStructs::obstacle3D>& obs_vec,int cnt0,int cnt1)
{
    UserStructs::obstacle3D obs0= obs_vec[cnt0];
    UserStructs::obstacle3D obs1= obs_vec[cnt1];
    int num= cnt1-cnt0;
    //double spd = std::sqrt(pow(obs1.x1-obs0.x1,2)+pow(obs1.x2-obs0.x2,2))/num;
    //double vv= (obs1.x3-obs0.x3)/num;

    for(int i=1;i!= num;++i){
       UserStructs::obstacle3D obs_sub;
       double lambda= 1.0*i/num;
       obs_sub.x1= obs0.x1*(1.-lambda)+ obs1.x1*lambda;
       obs_sub.x2= obs0.x2*(1.-lambda)+ obs1.x2*lambda;
       obs_sub.x3= obs0.x3*(1.-lambda)+ obs1.x3*lambda;
       obs_sub.head_xy= obs0.head_xy*(1.-lambda)+ obs1.head_xy*lambda;
       obs_sub.speed= obs0.speed*(1.-lambda)+ obs1.speed*lambda;
       //std::cout<<"obs0.speed:"<< obs0.speed <<" "<< obs1.speed<<"\n";
       //std::cout<<"1-lambda:"<< 1.-lambda<<" "<< "lambda:"<< lambda<<"\n";
       //std::cout<<"obs_sub.speed:"<< obs_sub.speed<<"\n";
       obs_sub.v_vert= obs0.v_vert*(1.-lambda)+ obs1.v_vert*lambda;
       obs_sub.r= obs0.r;
       obs_sub.hr= obs0.hr;
       obs_sub.t= obs0.t+ i;
       obs_vec[cnt0+i]= obs_sub;
    }
}//IntroPolate ends
