#pragma once
#include "wq/types.hpp"
#include <algorithm>

namespace wq {

// ===========================================================================
//  MOC 1: dung on dinh, BANH KHONG KHOA, khong co "y do" chuyen dong nao.
//
//  Luat motor:  tau = tau_ff + kp*(q_des - q) + kd*(dq_des - dq)
//
//  Banh:  kp = 0, dq_des = 0  =>  tau = -kd * dq
//         Day la MA SAT NHOT thuan tuy, khong phai khoa vi tri.
//         Banh khong co diem neo: day thi lan, buong thi tu tat dan.
//         Neu kd = 0 hoan toan, robot thanh xe day trung tinh: san nghieng
//         1 do la troi mai khong dung.
// ===========================================================================
struct IdleHoldConfig {
    double kp_leg_max = 80.0;
    double kd_leg     = 3.0;
    double kd_wheel   = 0.8;    // chinh: day tay thay nang nhung lan duoc,
                                // buong ra dung trong ~0.5 s
    double t_ramp     = 2.0;    // chi ramp kp; kd bat ngay tu ms dau (luon an toan)
    double tau_leg_max = 60.0;
    double tau_wheel_max = 20.0;
};

class IdleHold {
public:
    explicit IdleHold(const IdleHoldConfig& cfg = IdleHoldConfig{}) : cfg_(cfg) {}

    // BAY SO 1: doc q MOT LAN DUY NHAT roi dong bang.
    // Neu gan q_des = state.q moi vong lap thi sai so luon = 0 => thanh phan kp
    // luon = 0 => chan KHONG co do cung => robot tu tu sup xuong.
    // Loi nay rat hay gap vi code trong co ve hop ly.
    void onEnter(const RobotState& s) {
        q_hold_ = s.q;
        t_      = 0.0;
    }

    ActuatorCommand update(const RobotState& s, double dt) {
        t_ += dt;
        const double kp = cfg_.kp_leg_max *
                          std::min(t_ / std::max(cfg_.t_ramp, 1e-3), 1.0);

        ActuatorCommand c;
        c.q_des.head<kLegJoints>() = q_hold_;
        c.dq_des.setZero();
        c.kp.head<kLegJoints>().setConstant(kp);
        c.kd.head<kLegJoints>().setConstant(cfg_.kd_leg);

        // banh: kp = 0 TUYET DOI
        c.kp.tail<kNumLegs>().setZero();
        c.kd.tail<kNumLegs>().setConstant(cfg_.kd_wheel);
        limitCommandTorque(c, s, cfg_.tau_leg_max, cfg_.tau_wheel_max);
        return c;
    }

    const Vec12& held() const { return q_hold_; }
    bool rampDone() const { return t_ >= cfg_.t_ramp; }

private:
    IdleHoldConfig cfg_;
    Vec12  q_hold_ = Vec12::Zero();
    double t_      = 0.0;
};

} // namespace wq
