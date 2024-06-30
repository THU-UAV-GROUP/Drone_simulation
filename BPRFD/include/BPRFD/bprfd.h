#pragma once

#include "BPRFD/utils.h"

class BPRFD
{
private:
    ros::NodeHandle nh;

    bool debug_mode;

public:
    BPRFD();
    ~BPRFD();

    void getParams();

    void allocateMemory();

    void run();
};