#include <map>
#include <vector>
#include <fstream>
#include <iostream>
#include <cfloat>
#include <stdint.h>
#include "common/Utils/FindPath.h"

struct TwoDis{
    double v_dis;
    double h_dis;
    TwoDis():v_dis(0.),h_dis(0.){}
    TwoDis(double _v_dis,double _h_dis):v_dis(_v_dis),h_dis(_h_dis){}
};

void MinDis(const std::vector<TwoDis>& vec_dis, std::vector<TwoDis>& vec_result);

int main(int argc,char** argv)
{
   std::string obsdis_file = Utils::FindPath()+"records/obdis_log.txt";

   std::ifstream f_dis(obsdis_file.c_str());

   std::map<uint32_t,std::vector<TwoDis> > map_dis;

   while(f_dis.good())
   {
       uint32_t addr;
       double ver_dis;
       double hor_dis;

       f_dis >> addr >> ver_dis >> hor_dis;
       //new address
       if(map_dis.find(addr)==map_dis.end())
       {
          std::vector<TwoDis> dis_vec;
          dis_vec.push_back(TwoDis(ver_dis,hor_dis));
          map_dis[addr] = dis_vec;
       }
       else{
          map_dis[addr].push_back(TwoDis(ver_dis,hor_dis));
       }
   }//while ends

   for(std::map<uint32_t,std::vector<TwoDis> >::iterator it= map_dis.begin();it!=map_dis.end();++it)
   {
      std::vector<TwoDis> vec_result;
      MinDis(it->second,vec_result);
      std::cout << (int)it->first<<":"
                <<"(" << vec_result[0].v_dis<<","<< vec_result[0].h_dis<<")"<<","
                <<"(" << vec_result[1].v_dis<<","<< vec_result[1].h_dis<<")"<<"\n";
   }

}

//TwoDis MinDis(const std::vector<TwoDis>& vec_dis)
void MinDis(const std::vector<TwoDis>& vec_dis, std::vector<TwoDis>& vec_result)
{
   TwoDis min_dis(DBL_MAX,DBL_MAX);
   vec_result.clear();
   vec_result.push_back(min_dis);
   vec_result.push_back(min_dis);

   for(int i=0;i!= vec_dis.size();++i)
   {
      if(min_dis.v_dis > vec_dis[i].v_dis){
          min_dis.v_dis = vec_dis[i].v_dis;
          vec_result[0]= vec_dis[i];
      }

      if(min_dis.h_dis > vec_dis[i].h_dis){
          min_dis.h_dis = vec_dis[i].h_dis;
          vec_result[1]= vec_dis[i];
      }
   }
   //return min_dis;
}
