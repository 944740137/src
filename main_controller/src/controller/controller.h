#pragma once
// #include <algorithm/pinocchino_interactive.h>

#include <fstream>
#include <iostream>

namespace my_controller
{
    class Controller
    {

    private:
        // debug，绘图
        int time = 0;
        std::ofstream myfile;

    public:
        void startControllerManager();
    };

};

