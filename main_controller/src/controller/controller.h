#include <franka_example_controllers/pinocchino_interactive.h>
#include "main_controller/myrobot.h"
#include <fstream>
#include <iostream>

namespace my_controller
{
    template <typename _Scalar, int _Dofs = 7>
    class ControllerLawBase
    {
    private:
        Eigen::Matrix<_Scalar, _Dofs, 1> jointError;
        Eigen::Matrix<_Scalar, 6, 1> cartesianError;

        Eigen::Matrix<_Scalar, _Dofs, 1> LastjointError;
        Eigen::Matrix<_Scalar, 6, 1> LastcartesianError;

    public:
        bool setControllerLaw();
    };

    template <typename _Scalar, int _Dofs = 7>
    class ComputedTorqueMethod : ControllerLawBase<typename _Scalar, int _Dofs = 7>
    {

    };


    template <typename RobotType>
    class Controller
    {
        int time = 0;
        std::ofstream recordData;

    public:
        bool recordData(RobotType &myRobot);
        bool cycleRun(RobotType &myRobot);

    };

};

template <typename RobotType>
bool my_controller::Controller<RobotType>::recordData(RobotType &myRobot){

};

template <typename RobotType>
bool my_controller::Controller<RobotType>::cycleRun(RobotType &myRobot)
{
    // update robot param/controller param

    myRobot.updateData();
    myRobot.setTau()
};

template <typename RobotType>
bool startController()
{
    // pin obj
    my_robot::Robot<double, 7> myrobot;
    my_controller::Controller<my_robot::Robot<double, 7>> myController;
    // con obj robot obj
}
bool cycleRun()
{
    my_robot::Robot<double, 7> *myrobot;
    my_controller::Controller<my_robot::Robot<double, 7>> *myController;
    myController->cycleRun(*myrobot);
}