#include "BPRFD/bprfd.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bprfd");

    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    ROS_INFO("\033[1;32m BPRFD Main Begin.\033[0m");
    std::cout << std::endl;

    BPRFD BPRFD;
    BPRFD.run();

    ros::spinOnce();

    ROS_INFO("\033[1;32m BPRFD Main Done.\033[0m");

    return 0;  
}