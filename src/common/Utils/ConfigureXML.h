#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

namespace Utils{
    typedef boost::property_tree::ptree PropertyTree;
    typedef boost::shared_ptr<PropertyTree> PropertyTreePtr;
}
