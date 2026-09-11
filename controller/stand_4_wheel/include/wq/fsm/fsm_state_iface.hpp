#pragma once
#include "wq/types.hpp"

namespace wq {

// ===========================================================================
//  ADAPTER LAYER
//  File nay TON TAI de module build/test doc lap. Khi cam vao SDK that:
//    - Bo StateName o day, include enum CUA BAN.
//    - Cho StateQpBalance ke thua base class CUA BAN, forward 4 ham ao.
//  Khong sua gi trong balance_qp.* / robot_model.*
// ===========================================================================

// Enum cua SDK. Gia tri 3 va 5 BI KHUYET trong ban goc -- gan nhu chac chan
// da duoc SDK dung noi bo (state an, hoac ID danh cho firmware/protocol).
// DUNG lap vao: neu upstream co state 3 khong export ra enum, ban se gap
// loi kieu "chuyen state ngau nhien" cuc kho truy.
// => kQpBalance = 7, cao hon moi gia tri dang dung.
enum class StateName : int {
    kInvalid      = -1,
    kIdle         = 0,   // WaitingForStand
    kStandUp      = 1,   // StandingUp
    kJointDamping = 2,   // JointDamping
    kLieDown      = 4,   // LieDown
    kRLControl    = 6,   // RLControlMode
    kQpBalance    = 7,   // <-- MODULE NAY
};

const char* toString(StateName s);

// Cau noi toi sim / SDK. Trien khai lop nay o phia ban.
class RobotIO {
public:
    virtual ~RobotIO() = default;

    // Dien day du: p_WB, v_WB, q_WB, omega_B, q, dq, dq_w, tau_w, tau_j.
    // q/dq/tau_j: 12 leg joints in FL,FR,HL,HR order; wheel vectors use the
    // same four-leg order. Use robotToControllerOrder() at the SDK boundary.
    virtual void readState(RobotState& s) = 0;
    // c is in controller order (12 legs + 4 wheels). Implementations connected
    // to the VQR SDK must remap all five vectors with controllerToRobotOrder().
    virtual void writeCommand(const ActuatorCommand& c) = 0;
    virtual double dt() const = 0;

    // kStandUp cua VQR de kp banh=0 va chi khoa nhot bang kd=wheel_lock_kd_;
    // tra ve kd hien tai de StateQpBalance noi suy sang damping QP.
    virtual double wheelKdNow() const { return 0.0; }

    virtual bool requestLieDown() const { return false; }
    virtual bool requestDamping() const { return false; }
};

class FSMStateBase {
public:
    FSMStateBase(StateName id, const char* name) : id_(id), name_(name) {}
    virtual ~FSMStateBase() = default;

    virtual void enter() = 0;
    virtual void run()   = 0;
    virtual void exit()  = 0;
    virtual StateName checkChange() = 0;

    StateName   id()   const { return id_; }
    const char* name() const { return name_; }

protected:
    StateName   id_;
    const char* name_;
};

} // namespace wq
