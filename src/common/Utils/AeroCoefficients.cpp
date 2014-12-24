#include "AeroCoefficients.h"
namespace Utils{
  double CL_Lift(double x){
                                 //  -0.2000	-0.7500
                                 //   0.0000	0.2500
                                 //   0.2300	1.4000
                                 //   0.6000	0.7100
      double cl;
      if(x <= 0){
          cl = 5*(x+0.2)-0.75;
      }
      else if(x > 0 && x <= 0.23){
          cl = 1.15/0.23*x+0.25;
      }
      else {
          cl = -1.865*(x-0.6)+0.71;
      }
      return cl;
  }

  double CD0_Drag0Lift(double angle_attack){
      return 0.028;
  }

  double K_InducedDrag(){
      return 0.04;
  }

}
