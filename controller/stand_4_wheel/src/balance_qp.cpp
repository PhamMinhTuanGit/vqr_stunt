#include "wq/balance_qp.hpp"
#include "wq/math_utils.hpp"
#include <chrono>

namespace wq {

// moi chan: 2 (chan f_z) + 4 (kim tu thap ma sat) + 2 (momen banh) = 8
static constexpr int kIneqPerLeg = 8;
static constexpr int kNumIneq    = kIneqPerLeg * kNumLegs;   // 32
static constexpr int kNumVar     = 3 * kNumLegs;             // 12

BalanceQP::BalanceQP(RobotModel& model, const BalanceQpConfig& cfg)
    : model_(model), cfg_(cfg), qp_(kNumVar, kNumIneq) {
    A_.setZero(6, kNumVar);
    b_.setZero(6);
    G_.setZero(kNumVar, kNumVar);
    g0_.setZero(kNumVar);
    CI_.setZero(kNumIneq, kNumVar);
    ci0_.setZero(kNumIneq);

    f_prev_.setZero();
    q_nominal_.setZero();
    p_des_ = v_des_ = omega_des_ = Vec3::Zero();
    p_nominal_ = Vec3(0.0, 0.0, 0.35);
    R_des_ = R_nominal_ = Mat3::Identity();
}

void BalanceQP::onEnter(const RobotState& s) {
    // LATCH: bat dau tu CHINH trang thai hien tai -> sai so ban dau = 0.
    // Neu khong, sai so luc chuyen thuong 3-5 cm, nhan Kp cao => robot NHAY DUNG.
    p_des_ = s.p_WB;
    R_des_ = s.q_WB.toRotationMatrix();
    v_des_.setZero();
    omega_des_.setZero();

    p_nominal_.head<2>() = s.p_WB.head<2>();   // x,y nominal = noi dang dung

    // Khoa Yaw nominal theo huong hien tai cua robot (roll=0, pitch=0, giu nguyen yaw).
    // Dieu nay ngan chan robot bi giat xoay than do goc yaw IMU khac 0:
    const Mat3 R_current = s.q_WB.toRotationMatrix();
    const double current_yaw = std::atan2(R_current(1, 0), R_current(0, 0));
    R_nominal_ = Eigen::AngleAxisd(current_yaw, Vec3::UnitZ()).toRotationMatrix();

    // Moi f_prev bang phan bo tinh: neu de = 0, so hang beta*||f - f_prev||^2
    // se keo nghiem dau tien ve 0 va robot sut mot nhip.
    f_prev_ = staticDistribution(s);
    solve_ok_ = true;
}

Vec12 BalanceQP::staticDistribution(const RobotState& s) const {
    Vec12 f = Vec12::Zero();
    const int n = s.numContacts();
    if (n == 0) return f;
    const double fz = s.mass * kGravity / static_cast<double>(n);
    for (int i = 0; i < kNumLegs; ++i)
        if (s.contact[i]) f.segment<3>(3 * i) = Vec3(0.0, 0.0, fz);
    return f;
}

void BalanceQP::buildConstraints(const RobotState& s) {
    CI_.setZero();
    ci0_.setZero();

    const double fmax      = cfg_.f_max_scale * s.mass * kGravity;
    const double f_tan_max = cfg_.tau_wheel_max / model_.wheelRadius();

    for (int i = 0; i < kNumLegs; ++i) {
        const int  c  = 3 * i;
        const int  r  = kIneqPerLeg * i;
        const bool on = s.contact[i];

        // Chan swing: ep f_z in [0,0]; kim tu thap tu ep f_x = f_y = 0
        const double lo = on ? cfg_.f_min : 0.0;
        const double hi = on ? fmax       : 0.0;

        CI_(r + 0, c + 2) =  1.0;  ci0_(r + 0) = -lo;   //  f_z - lo >= 0
        CI_(r + 1, c + 2) = -1.0;  ci0_(r + 1) =  hi;   // -f_z + hi >= 0

        // kim tu thap ma sat tuyen tinh hoa: |f_x| <= mu*f_z, |f_y| <= mu*f_z
        CI_(r + 2, c + 0) = -1.0;  CI_(r + 2, c + 2) = cfg_.mu;
        CI_(r + 3, c + 0) =  1.0;  CI_(r + 3, c + 2) = cfg_.mu;
        CI_(r + 4, c + 1) = -1.0;  CI_(r + 4, c + 2) = cfg_.mu;
        CI_(r + 5, c + 1) =  1.0;  CI_(r + 5, c + 2) = cfg_.mu;

        // gioi han momen banh: |f . e_roll| <= tau_max / r_w
        // (momen banh = ban kinh x luc tiep tuyen -- J^T tu lo phan nay)
        const Vec3 e = model_.contactJacobian(i).e_roll_W;
        CI_.block<1, 3>(r + 6, c) = -e.transpose();  ci0_(r + 6) = f_tan_max;
        CI_.block<1, 3>(r + 7, c) =  e.transpose();  ci0_(r + 7) = f_tan_max;
    }
}

ActuatorCommand BalanceQP::update(const RobotState& s, double dt) {
    const auto t0 = std::chrono::steady_clock::now();
    const Mat3 R  = s.q_WB.toRotationMatrix();

    // ---------- 1. Setpoint troi dan ve nominal ----------
    const double a = std::min(dt / std::max(cfg_.drift_tau, 1e-3), 1.0);
    p_des_ += a * (p_nominal_ - p_des_);

    if (std::abs(omega_des_.z()) > 1e-4) {
        // Tich luy van toc goc yaw tu tay dieu khien (turning rate):
        const Eigen::AngleAxisd dyaw(omega_des_.z() * dt, Vec3::UnitZ());
        R_nominal_ = (dyaw * R_nominal_).eval();
        R_des_     = (dyaw * R_des_).eval();
    } else {
        // Khong co lenh xoay: keo R_des ve R_nominal (roll=0, pitch=0, giu yaw danh dinh)
        R_des_ = slerpR(R_des_, R_nominal_, a);
    }

    // ---------- 2. Gia toc mong muon ----------
    Vec3 e_p = p_des_ - s.p_WB;
    // x,y: bao hoa vi p_WB(x,y) khong quan sat duoc tuyet doi (estimator troi)
    e_p.head<2>() = satVec(e_p.head<2>().eval(), cfg_.xy_pos_sat);

    const Vec3 a_cmd = cfg_.Kp_pos.cwiseProduct(e_p)
                     + cfg_.Kd_pos.cwiseProduct(v_des_ - s.v_WB);

    // omega_B la he THAN -> phai xoay sang world TRUOC khi tru. Loi dau rat hay gap.
    const Vec3 omega_W = R * s.omega_B;
    const Vec3 e_R     = so3Log(R_des_ * R.transpose());
    const Vec3 dw_cmd  = cfg_.Kp_rot.cwiseProduct(e_R)
                       + cfg_.Kd_rot.cwiseProduct(omega_des_ - omega_W);

    // ---------- 3. Wrench mong muon (SRBD) ----------
    b_.head<3>() = s.mass * (a_cmd + Vec3(0.0, 0.0, kGravity));
    b_.tail<3>() = s.I_G_W * dw_cmd + omega_W.cross(s.I_G_W * omega_W);

    // ---------- 4. Ma tran anh xa luc -> wrench ----------
    A_.setZero();
    for (int i = 0; i < kNumLegs; ++i) {
        const Vec3 r = s.p_c_W[i] - s.p_CoM_W;
        A_.block<3, 3>(0, 3 * i) = Mat3::Identity();
        A_.block<3, 3>(3, 3 * i) = skew(r);
    }

    // ---------- 5. Hessian & gradient ----------
    // min ||Af - b||_S^2 + alpha||f||^2 + beta||f - f_prev||^2
    const Vec6 Sw = (s.numContacts() <= 2) ? cfg_.S_two_contact : cfg_.S;
    const Eigen::DiagonalMatrix<double, 6> S(Sw);

    G_.noalias() = 2.0 * (A_.transpose() * S * A_);
    G_.diagonal().array() += 2.0 * (cfg_.alpha + cfg_.beta);
    g0_.noalias() = -2.0 * (A_.transpose() * (S * b_) + cfg_.beta * f_prev_);

    buildConstraints(s);

    // ---------- 6. Giai ----------
    VecXd f = f_prev_;
    solve_ok_ = qp_.solve(G_, g0_, CI_, ci0_, f);
    if (!solve_ok_) {
        // Fallback: loc dan ve phan bo tinh thay vi nhay hoac dung nghiem rac
        f = 0.9 * f_prev_ + 0.1 * staticDistribution(s);
    }
    f_prev_ = f.head<kNumVar>();

    // ---------- 7. tau = g(q) - sum J^T f ----------
    // CO dau tru va CO g(q) -- quy uoc Pinocchio.
    // Loi ich kem theo: g(q) bu luon trong luong ban than chan, thu ma SRBD bo qua.
    VecXd tau_full = model_.generalizedGravity();
    for (int i = 0; i < kNumLegs; ++i) {
        if (!s.contact[i]) continue;
        tau_full.noalias() -= model_.contactJacobian(i).J.transpose()
                            * f_prev_.segment<3>(3 * i);
    }

    ActuatorCommand cmd;
    cmd.tau_ff = model_.toActuated(tau_full);
    cmd.tau_ff.head<kLegJoints>() =
        cmd.tau_ff.head<kLegJoints>().cwiseMax(-cfg_.tau_leg_max)
                                           .cwiseMin(cfg_.tau_leg_max);
    cmd.tau_ff.tail<kNumLegs>() =
        cmd.tau_ff.tail<kNumLegs>().cwiseMax(-cfg_.tau_wheel_max)
                                        .cwiseMin(cfg_.tau_wheel_max);

    // Lop PD khop PHU: gain THAP, chi chong nhieu tan so cao
    cmd.q_des.head<kLegJoints>() = q_nominal_;
    cmd.dq_des.setZero();
    cmd.kp.head<kLegJoints>().setConstant(cfg_.kp_joint);
    cmd.kd.head<kLegJoints>().setConstant(cfg_.kd_joint);

    // Banh: kp = 0 TUYET DOI (khong khoa vi tri), chi damping nhe.
    // Dat kp != 0 cho banh la loi kinh dien gay giat vang.
    cmd.kp.tail<kNumLegs>().setZero();
    cmd.kd.tail<kNumLegs>().setConstant(cfg_.kd_wheel);

    limitCommandTorque(cmd, s, cfg_.tau_leg_max, cfg_.tau_wheel_max);

    const auto t1 = std::chrono::steady_clock::now();
    solve_us_ = std::chrono::duration<double, std::micro>(t1 - t0).count();
    return cmd;
}

} // namespace wq
