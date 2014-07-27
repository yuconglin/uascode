#include "YcLogger.h"

#include <boost/filesystem.hpp>
#include <log4cxx/propertyconfigurator.h>

namespace fs = boost::filesystem;

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.YcLogger.YcLogger"));
}

namespace Utils{

LogConfigurator::LogConfigurator(const std::string& propfile, const std::string& msg)
{
    fs::path p(propfile);

    if (!fs::exists(p))
    {
        fs::path p2file("/home/yucong/ros_workspace/uascode/src/common/Utils/");
        p = fs::system_complete(p2file / p);
        if (!fs::exists(p))
        {
            p = fs::system_complete(p2file / p.filename());
        }
    }

    std::string fname = p.string();
    std::ostringstream oss;
    oss << "\n\n\n"
        << "****************************************************\n"
        << "log4cxx configuration: " << fname << "\n"
        << "msg:                   " << msg << "\n"
        << "****************************************************\n"
        << std::endl;

    std::cerr << oss.str() << std::endl;

    if (fs::exists(p))
    {
        log4cxx::PropertyConfigurator::configure(fname);
        UASLOG(s_logger, LL_INFO, oss.str());
    }
    else
    {
        try {
            throw std::runtime_error ("cannot configure log4cxx because property file is missing");
        } catch (std::runtime_error &e) {
            std::cout << "Caught a runtime_error exception: " << e.what () << '\n';
        }
    }
}

log4cxx::LevelPtr getStartup()
{
    static log4cxx::LevelPtr level(new log4cxx::Level(INT_MAX - 1, LOG4CXX_STR("STARTUP"), 0));
    return level;
}

}
