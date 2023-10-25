#pragma once

#include <vector>

#include "ControllerConstants.h"

namespace Actions{
    enum Action{
        NONE = -1,
        XSTRAFE,
        YSTRAFE,
        ROTATION,
        ZERO,
        ACTION_COUNT //Just the number of actions, as it is at the end of a enum
    };

    //Different enum for POV actions because logic is different
    enum POVAction{
        NO_POV_ACTION = -1,
        TEMP,
        ACTION_COUNT_POV //Just the number of actions, as it is at the end of a enum
    };
}

namespace ControllerMapData{
    using namespace ControllerConstants;
    using namespace Actions;

    //Element in the buttonmap, mapping a single button to action
    struct ControlMapElement{
        Button button;
        Action action;
    };

    //Maps Buttons -> Actions
    //Buttons are structs in the form of {Joystick, ButtonData}
    //There are already some named ButtonData and Buttons
    const std::vector<ControlMapElement> ButtonMap = {
        {{LJOY, X_AXIS},     XSTRAFE},
        {{LJOY, Y_AXIS},     YSTRAFE},
        {{LJOY, TRIGGER},      NONE},
        {{LJOY, B_2},          NONE},
        {{LJOY, B_4},          NONE},

        {{RJOY, X_AXIS},    ROTATION},
        {{RJOY, Y_AXIS},        NONE},
        {{RJOY, TRIGGER},       ZERO},
        {{RJOY, B_2},           NONE},
        {{RJOY, B_3},          NONE},
        {{RJOY, B_4},           NONE},
        
        {XBOX_LJOY_X,           NONE},
        {XBOX_LJOY_Y,    NONE}, 
        {XBOX_LJOY_Y,  NONE},
        {XBOX_RJOY_X,           NONE},
        {XBOX_RJOY_Y,     NONE},
        {XBOX_LTRIGGER,  NONE},
        {XBOX_LTRIGGER,  NONE},
        {XBOX_RTRIGGER,  NONE},
        {XBOX_A_BUTTON ,        NONE},
        {XBOX_B_BUTTON ,      NONE},
        {XBOX_X_BUTTON ,  NONE},
        {XBOX_Y_BUTTON ,     NONE},
        {XBOX_L_BUMPER ,  NONE},
        {XBOX_R_BUMPER ,   NONE},
        {{XBOX, B_7},    NONE},
        {{XBOX, B_8},   NONE},

        {BB_UP,              NONE},
        {BB_LEFT,         NONE},
        {BB_RIGHT,       NONE}
    };

    //Allows for maps of buttons to values, such as the index of the buttonboard
    //Only for buttons and triggers currently
    //No need for default val because it's now in the controller method
    template <typename T>
    struct ValueMapElement{
        Button button;
        T value;
    };

    //Takes the range from min to max
    //if range covers over 0, like from 350->10, the larger number comes first
    struct POVRange{
        int min;
        int max;
    };

    const POVRange POV_UP = {350, 10};
    const POVRange POV_RIGHT = {80, 100};
    const POVRange POV_DOWN = {170, 190};
    const POVRange POV_LEFT = {260, 280};

    struct POVMapElement{
        Button pov;
        POVRange range;
        POVAction action;
    };

    const std::vector<POVMapElement> POVMap = {
        
    };
};