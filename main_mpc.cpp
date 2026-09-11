#include "state_machine.hpp"
#include <iostream>

#ifdef BUILD_SIMULATION
    #define BACKWARD_HAS_DW 1
    #include "backward.hpp"
    namespace backward{
        backward::SignalHandling sh;
    }
#endif

using namespace types;

MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

int main(){
    std::cout << "==========================================" << std::endl;
    std::cout << "       Starting VQR Wheel MPC Deploy      " << std::endl;
    std::cout << "  (State: Idle -> StandUp -> QPBalance)   " << std::endl;
    std::cout << "==========================================" << std::endl;

    // Launch state machine with default active state set to kQPBalance
    StateMachine state_machine(RobotType::VqrWheel, StateName::kQPBalance);
    state_machine.Run();
    return 0;
}
