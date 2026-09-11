#include "wq/fsm/state_qp_balance.hpp"
#include <algorithm>
#include <cstdio>

namespace wq {

const char* toString(StateName s) {
    switch (s) {
        case StateName::kIdle:         return "kIdle";
        case StateName::kStandUp:      return "kStandUp";
        case StateName::kJointDamping: return "kJointDamping";
        case StateName::kLieDown:      return "kLieDown";
        case StateName::kRLControl:    return "kRLControl";
        case StateName::kQpBalance:    return "kQpBalance";
        default:                       return "kInvalid";
    }
}

StateQpBalance::StateQpBalance(RobotModel& model, RobotIO& io,
                               const BalanceQpConfig& qp_cfg,
                               const QpBalanceStateConfig& st_cfg)
    : FSMStateBase(StateName::kQpBalance, "qp_balance"),
      model_(model), io_(io), qp_(model, qp_cfg), cfg_(st_cfg) {}

void StateQpBalance::enter() {
    io_.readState(s_);
    model_.update(s_);

    // LATCH toan bo tai thoi diem vao -- dung cai bay o moc 1.
    q_hold_ = s_.q;
    qp_.setNominalJoint(s_.q);
    qp_.setTargetHeight(s_.p_WB.z());     // giu nguyen chieu cao kStandUp de lai
    qp_.setTargetXY(s_.p_WB.head<2>());
    qp_.onEnter(s_);

    // VQR kStandUp giu kp banh=0 va khoa nhot bang kd. Ke thua kd de chuyen
    // lien tuc sang damping cua QP; khong bao gio tao wheel position target.
    kd_wheel0_ = io_.wheelKdNow();

    lambda_   = 0.0;
    fail_cnt_ = 0;
    fallen_   = false;

    if (cfg_.enable_log) log_.open(cfg_.log_path);

    std::printf("[kQpBalance] enter: z=%.3f  gz=%.2f  kd_wheel0=%.1f\n",
                s_.p_WB.z(), s_.g_B.z(), kd_wheel0_);
}

ActuatorCommand StateQpBalance::blendWithHoldPd(const ActuatorCommand& qp_cmd,
                                                double dt) {
    lambda_ = std::min(lambda_ + dt / std::max(cfg_.blend_time, 1e-3), 1.0);
    const double w = 1.0 - lambda_;

    ActuatorCommand out;
    out.q_des.head<kLegJoints>() = q_hold_;   // PD giu tu the cuoi standup
    out.dq_des.setZero();

    // PD thuan khong co feedforward -> chi scale phan QP
    out.tau_ff = lambda_ * qp_cmd.tau_ff;

    out.kp.head<kLegJoints>() =
        Vec12::Constant(w * cfg_.pd_kp_hold) + lambda_ * qp_cmd.kp.head<kLegJoints>();
    out.kd.head<kLegJoints>() =
        Vec12::Constant(w * cfg_.pd_kd_hold) + lambda_ * qp_cmd.kd.head<kLegJoints>();

    // Banh luon kp=0; noi suy damping kStandUp -> damping QP.
    out.kp.tail<kNumLegs>().setZero();
    out.kd.tail<kNumLegs>() =
        Vec4::Constant(w * kd_wheel0_) + lambda_ * qp_cmd.kd.tail<kNumLegs>();
    return out;
}

void StateQpBalance::run() {
    const double dt = io_.dt();

    io_.readState(s_);
    model_.update(s_);   // FK + Jacobian + g(q): MOT lan / chu ky

    // Moc hien tai: 4 diem do co dinh.
    // Moc nhac 2 chan cheo: doi thanh {true,false,false,true} -- KHONG viet
    // controller moi; QP tu phan bo luc len 2 diem con lai.
    s_.contact = {true, true, true, true};

    ActuatorCommand qp_cmd = qp_.update(s_, dt);

    if (qp_.lastSolveOk()) fail_cnt_ = 0;
    else                   ++fail_cnt_;

    ActuatorCommand out = (lambda_ < 1.0) ? blendWithHoldPd(qp_cmd, dt) : qp_cmd;
    limitCommandTorque(out, s_, qp_.legTorqueLimit(), qp_.wheelTorqueLimit());

    fallen_ = (s_.g_B.z() > cfg_.fall_gz_ratio * kGravity);

    if (!out.isFinite()) {          // chan tuyet doi: NaN xuong motor la hong phan cung
        std::printf("[kQpBalance] ** NaN trong lenh -> DAMPING **\n");
        out = ActuatorCommand::Damping();
        fail_cnt_ = cfg_.max_qp_fail + 1;
    }

    io_.writeCommand(out);

    if (cfg_.enable_log)
        log_.write(s_, out, qp_.lastForces(),
                   static_cast<int>(id()), qp_.lastSolveOk());
}

void StateQpBalance::exit() {
    log_.close();
    std::printf("[kQpBalance] exit: lambda=%.2f fail=%d fallen=%d\n",
                lambda_, fail_cnt_, fallen_ ? 1 : 0);
}

StateName StateQpBalance::checkChange() {
    if (fallen_)                         return StateName::kJointDamping;
    if (fail_cnt_ > cfg_.max_qp_fail)    return StateName::kJointDamping;
    if (io_.requestDamping())            return StateName::kJointDamping;
    if (io_.requestLieDown())            return StateName::kLieDown;
    return StateName::kQpBalance;
}

} // namespace wq
