#include "FindPath.h"
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace Utils{
 std::string FindPath()
 {
     std::string path1="/home/yucong/ros_workspace/uascode/";
     std::string path2="/home/yucong/fuerte_workspace/sandbox/uascode/";
     //std::string path;

     fs::path p1(path1);
     fs::path p2(path2);

     if(fs::exists(p1))
         return path1;

     if(fs::exists(p2))
         return path2;
 }

}
