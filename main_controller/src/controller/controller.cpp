#include <algorithm/pinocchino_interactive.h>
#include "controller/controller.h"

extern pinLibInteractive *pinInteractive;

namespace my_controller
{
    void Controller::startControllerManager()
    {
        if (pinInteractive == nullptr)
            pinInteractive = new pinLibInteractive();
    }

};