#include "BPRFD/bprfd.h"

BPRFD::BPRFD()
{
    getParams();

    allocateMemory();
}

BPRFD::~BPRFD() {}

void BPRFD::getParams()
{
    ROS_INFO_STREAM("\033[1;32m Getting params begin. \033[0m");
    std::cout << std::endl;

    nh.param<bool>("bprfd/base_control/debug_mode", debug_mode, false);
    ROS_INFO_STREAM("\033[1;32m -- whether use debug mode: " << (debug_mode ? "true" : "false") << " \033[0m");
    std::cout << std::endl;

    ROS_INFO_STREAM("\033[1;32m Getting params done. \033[0m");
    std::cout << std::endl;
}

void BPRFD::allocateMemory()
{

}

void BPRFD::run()
{
    
}