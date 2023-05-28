#include "robot/myrobot.h"
#include <controllerLaw/controllerLaw.h>
namespace my_controller
{
    template <int _Dofs>
    bool ComputedTorqueMethod<_Dofs>::setControllerLaw()
    {
        // qc = ddq_d + Kp * error + Kv * derror;
        // tau_d << M * (qc) + C /* + G */; 
        return true;
    }

}