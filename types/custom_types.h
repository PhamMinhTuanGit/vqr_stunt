#ifndef CUSTOM_TYPES_H_
#define CUSTOM_TYPES_H_

#include "common_types.h"

namespace types{
    enum RobotType{
        VqrWheel,
    };

    enum RobotMotionState{
        WaitingForStand = 0,
        StandingUp      = 1,
        JointDamping    = 2,
        LieDown         = 4,
        RLControlMode   = 6,
        QPBalanceMode   = 7,
        YawTurnMode     = 8,
    };

    enum StateName{
        kInvalid      = -1,
        kIdle         = 0,
        kStandUp      = 1,
        kJointDamping = 2,
        kLieDown      = 4,
        kRLControl    = 6,
        kQPBalance    = 7,
        kQpBalance    = 7,
        kYawTurn      = 8,
    };
    

    inline std::string GetAbsPath(){
        char buffer[PATH_MAX];
        if(getcwd(buffer, sizeof(buffer)) != NULL){
            return std::string(buffer);
        }
        return "";
    }
};

#endif