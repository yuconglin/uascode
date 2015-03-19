#include "MathUtils.h"
namespace Utils{

  namespace math{
    float constrain(float val, float min, float max)
    {
	return (val < min) ? min : ((val > max) ? max : val);
    }

    int sgn(double num)
    {
        return (num>0) ? 1 : ((num<0) ? -1 : 0);
    }
}

}
