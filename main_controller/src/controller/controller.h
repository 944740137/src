#pragma once
// #include <algorithm/pinocchino_interactive.h>

#include <fstream>
#include <iostream>
#include "robot/myrobot.h"

namespace my_controller
{
    template <int _Dofs = 7>
    class Controller
    {

    private:
        // debug，绘图
        int time = 0;
        std::ofstream myfile;

        // controllerLaw
        Eigen::Matrix<double, _Dofs, 1> jointError;
        Eigen::Matrix<double, 6, 1> cartesianError;

        Eigen::Matrix<double, _Dofs, 1> LastjointError;
        Eigen::Matrix<double, 6, 1> LastcartesianError;

        Eigen::Matrix<double, _Dofs, 1> tau_d;
        Eigen::Matrix<double, _Dofs, 1> qc;

    public:
        virtual bool setControllerLaw(my_robot::Robot<_Dofs> *robot) = 0;
        virtual bool updateError(my_robot::Robot<_Dofs> *robot) = 0;
        void startControllerManager();
    };

    template <int _Dofs = 7>
    class ComputedTorqueMethod : public Controller<_Dofs>
    {
    private:
        double Ka = 1;
        double Kv = 1;
        double Kp = 1;

    public:
        bool setControllerLaw(my_robot::Robot<_Dofs> *robot);
    };

};

const int pandaDim = 7;
typedef my_controller::Controller<pandaDim> PandaController;
typedef my_robot::Robot<pandaDim> Panda;

