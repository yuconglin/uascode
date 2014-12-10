#include "FindPath.h"
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace Utils{
 std::string FindPath()
 {
     std::string path="/home/yucong/catkin_ws/src/uascode/";
     fs::path p1(path);

     if(fs::exists(p1))
        return path;
 }

}
