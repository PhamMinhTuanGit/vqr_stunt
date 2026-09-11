#pragma once
#include "wq/types.hpp"
#include "wq/robot_model.hpp"
#include "wq/qp_solver.hpp"

namespace wq {

struct BalanceQpConfig {
    // --- vong ngoai PD ---
    // KHAC BIET COT LOI giua robot chan va robot BANH:
    //   z, roll, pitch : duoi/co chan -> ON DINH TINH  -> gain CAO
    //   x, y, yaw      : momen banh   -> KHAU TICH PHAN THUAN, troi tu do
    //                    + p_WB(x,y) khong quan sat duoc tuyet doi
    //                    -> muc tieu CHINH la v_des = 0, vi tri chi hieu chinh cham
    Vec3 Kp_pos = Vec3( 20.0,  20.0, 400.0);
    Vec3 Kd_pos = Vec3( 15.0,  15.0,  40.0);
    Vec3 Kp_rot = Vec3(500.0, 500.0, 120.0);
    Vec3 Kd_rot = Vec3( 30.0,  30.0,  20.0);
    double xy_pos_sat = 0.10;      // bao hoa sai so x,y [m]
    double drift_tau  = 1.5;       // hang so thoi gian keo setpoint ve nominal [s]

    // --- trong so QP ---
    // angular (20) > linear (1) vi LAT la khong cuu duoc, con lech vi tri thi cuu duoc
    Vec6 S = (Vec6() << 1, 1, 1, 20, 20, 5).finished();
    // Cau hinh 2 diem do (nhac 2 chan cheo): bai toan DU AM BAC -> phai NOI S,
    // giam trong so cac truc khong kiem soat duoc. Tu kich hoat khi numContacts <= 2.
    Vec6 S_two_contact = (Vec6() << 1, 1, 1, 20, 5, 1).finished();

    double alpha = 1e-3;   // regularization, giu QP luon co nghiem
    double beta  = 1e-2;   // lam muot theo thoi gian -- CHONG CHATTERING.
                           // Thieu no, khi 4 diem do (du 6 bac) nghiem QP nhay
                           // giua cac cau hinh tuong duong moi chu ky -> motor keu.

    // --- rang buoc ---
    double mu            = 0.4;    // bao thu; banh cao su/san cung thuc te 0.6-0.8
    double f_min         = 5.0;    // ep chan luon ti nhe, tranh mat tiep xuc
    double f_max_scale   = 1.5;    // f_z <= 1.5 * m * g
    double tau_wheel_max = 20.0;   // [N.m], matches MJCF actuator ctrlrange

    // --- lop PD khop PHU, chi chong nhieu tan so cao ---
    double kp_joint = 5.0;         // dung vuot 15, se chong lai QP
    double kd_joint = 1.0;
    double kd_wheel = 0.5;         // banh: kp = 0 tuyet doi

    double tau_leg_max = 60.0;     // [N.m], matches URDF/MJCF leg limit
};

class BalanceQP {
public:
    BalanceQP(RobotModel& model, const BalanceQpConfig& cfg = BalanceQpConfig{});

    // GOI khi vao state: latch setpoint = trang thai hien tai (sai so ban dau = 0)
    // + moi f_prev bang phan bo tinh.
    void onEnter(const RobotState& s);

    ActuatorCommand update(const RobotState& s, double dt);

    void setTargetHeight(double z)     { p_nominal_.z() = z; }
    void setTargetXY(const Vec2& xy)   { p_nominal_.head<2>() = xy; }
    void setTargetYawRate(double wz)   { omega_des_.z() = wz; }
    void setNominalJoint(const Vec12& q) { q_nominal_ = q; }
    void setNominalOrientation(const Mat3& R) { R_nominal_ = R; }

    const Vec12& lastForces() const { return f_prev_; }
    bool lastSolveOk() const { return solve_ok_; }
    double lastSolveTimeUs() const { return solve_us_; }
    double legTorqueLimit() const { return cfg_.tau_leg_max; }
    double wheelTorqueLimit() const { return cfg_.tau_wheel_max; }

private:
    void  buildConstraints(const RobotState& s);
    Vec12 staticDistribution(const RobotState& s) const;

    RobotModel&     model_;
    BalanceQpConfig cfg_;
    QpSolver        qp_;

    Vec3 p_des_, v_des_, omega_des_, p_nominal_;
    Mat3 R_des_, R_nominal_;
    Vec12 f_prev_, q_nominal_;

    bool   solve_ok_ = true;
    double solve_us_ = 0.0;

    MatXd A_, G_, CI_;
    VecXd b_, g0_, ci0_;
};

} // namespace wq
