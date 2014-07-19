#include "MavlinkRecNode.hpp"

using namespace UasCode;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MavRecNode");

    MavlinkRecNode mavrec_node;
    mavrec_node.TcpSetUp();
    mavrec_node.working();
}
