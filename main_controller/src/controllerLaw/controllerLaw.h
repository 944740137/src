#pragma once
// #include <algorithm/pinocchino_interactive.h>

#include <fstream>
#include <iostream>

namespace my_controller
{
    template <int _Dofs = 7>
    class ControllerLawBase
    {
    private:
        Eigen::Matrix<double,_Dofs, 1> jointError;
        Eigen::Matrix<double,6, 1> cartesianError;

        Eigen::Matrix<double,_Dofs, 1> LastjointError;
        Eigen::Matrix<double,6, 1> LastcartesianError;

    public:
        Eigen::Matrix<double,_Dofs, 1> tau_d;
        Eigen::Matrix<double,_Dofs, 1> qc;
        virtual bool setControllerLaw() = 0;
    };



    template <int _Dofs>
    class ComputedTorqueMethod: ControllerLawBase<_Dofs>
    {
    private:
        double Ka;
        double Kv;
        double Kp;

    public:
        bool setControllerLaw();
    };



    template <int _Dofs>
    class PID : ControllerLawBase<_Dofs>
    {
    private:
        double Ka;
        double Kv;
        double Kp;

    public:
        bool setControllerLaw();
    };
};

