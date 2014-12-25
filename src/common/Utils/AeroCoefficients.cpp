#include "AeroCoefficients.h"
#include <cmath>
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

  double CD0_Drag0Lift(double x){
      //-1.5700	1.5000
      //-0.2600	0.0560
      // 0.0000	0.0280
      // 0.2600	0.0560
      // 1.5700	1.5000
      double abs_x = std::abs(x);
      double cd0 = 0;
      if(abs_x >= 0 && abs_x <= 0.26){
        cd0 = -0.1077 * abs_x + 0.028;
      }
      else if(abs_x > 0.26 && abs_x <= M_PI/2){
        cd0 = (1.5-0.056) / (M_PI-0.26) * (abs_x - 0.26) + 0.056;
      }
      else{
        cd0 = 1.5;
      }
      return cd0;
  }

  double K_InducedDrag(){
      return 0.04;
  }

}
