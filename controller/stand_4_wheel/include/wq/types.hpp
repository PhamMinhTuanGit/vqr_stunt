#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>

namespace wq {

using Vec2  = Eigen::Vector2d;
using Vec3  = Eigen::Vector3d;
using Vec4  = Eigen::Vector4d;
using Vec6  = Eigen::Matrix<double, 6, 1>;
using Vec12 = Eigen::Matrix<double, 12, 1>;
using Vec16 = Eigen::Matrix<double, 16, 1>;
using Mat3  = Eigen::Matrix3d;
using Quat  = Eigen::Quaterniond;
using MatXd = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

constexpr int    kNumLegs   = 4;
constexpr int    kLegJoints = 12;   // 3 khop / chan
constexpr int    kNumAct    = 16;   // 12 chan + 4 banh
constexpr double kGravity   = 9.81;

// Canonical VQR order used by this controller and by policy_order.
enum LegId { FL = 0, FR = 1, HL = 2, HR = 3 };

// Controller order: 12 leg joints, then FL/FR/HL/HR wheels.
// Wire/robot order: FL/FR/HL/HR, with each wheel after its three leg joints.
// These helpers make the boundary remap explicit and prevent commanding the
// wrong actuator when this module is connected to the existing SDK.
inline Vec16 controllerToRobotOrder(const Vec16& in) {
    Vec16 out;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        out.segment<3>(4 * leg) = in.segment<3>(3 * leg);
        out[4 * leg + 3] = in[kLegJoints + leg];
    }
    return out;
}

inline Vec16 robotToControllerOrder(const Vec16& in) {
    Vec16 out;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        out.segment<3>(3 * leg) = in.segment<3>(4 * leg);
        out[kLegJoints + leg] = in[4 * leg + 3];
    }
    return out;
}

// ---------------------------------------------------------------- state ----
struct RobotState {
    // --- base: lay TRUC TIEP tu sim ---
    Vec3 p_WB    = Vec3::Zero();      // vi tri base, he WORLD
    Vec3 v_WB    = Vec3::Zero();      // van toc dai base, he WORLD
    Quat q_WB    = Quat::Identity();  // huong base
    Vec3 omega_B = Vec3::Zero();      // van toc goc, he THAN

    // --- khop ---
    Vec12 q     = Vec12::Zero();
    Vec12 dq    = Vec12::Zero();
    Vec4  dq_w  = Vec4::Zero();       // toc do 4 banh (goc banh KHONG can)
    Vec4  tau_w = Vec4::Zero();       // momen banh do duoc
    Vec12 tau_j = Vec12::Zero();      // momen khop chan do duoc

    // --- tiep xuc ---
    std::array<bool, kNumLegs> contact = {true, true, true, true};

    // --- dan xuat: RobotModel::update() dien ---
    std::array<Vec3, kNumLegs> p_c_W{};   // diem cham san, world
    std::array<Vec3, kNumLegs> axis_W{};  // truc quay banh, world
    Vec3   p_CoM_W = Vec3::Zero();
    Mat3   I_G_W   = Mat3::Identity();    // quan tinh tam khoi, world-aligned
    double mass    = 0.0;
    Vec3   g_B     = Vec3(0, 0, -kGravity);  // trong luc trong he THAN

    double t = 0.0;

    int numContacts() const {
        int n = 0;
        for (bool c : contact) if (c) ++n;
        return n;
    }
};

// -------------------------------------------------------------- command ----
struct ActuatorCommand {
    // All fields use controller order (12 leg joints + 4 wheels). RobotIO must
    // call controllerToRobotOrder() for every field before writing to the SDK.
    Vec16 q_des  = Vec16::Zero();
    Vec16 dq_des = Vec16::Zero();
    Vec16 tau_ff = Vec16::Zero();
    Vec16 kp     = Vec16::Zero();
    Vec16 kd     = Vec16::Zero();

    // Lenh an toan van nang: tha long co damping.
    // LUU Y: khong ton tai trang thai "khong gui lenh" -- motor co watchdog,
    // ngung publish la driver tu chuyen sang limp va robot sup.
    static ActuatorCommand Damping(double kd_leg = 2.0, double kd_wheel = 1.0) {
        ActuatorCommand c;
        c.kd.head<kLegJoints>().setConstant(kd_leg);
        c.kd.tail<kNumLegs>().setConstant(kd_wheel);
        return c;
    }

    bool isFinite() const {
        return q_des.allFinite() && dq_des.allFinite() && tau_ff.allFinite()
            && kp.allFinite() && kd.allFinite();
    }
};

// Limit the instantaneous hybrid-PD torque at the state used to form a command.
// The hardware interface does not apply these limits itself, so clamping only
// tau_ff would still allow the kp/kd terms to exceed the model's actuator caps.
inline void limitCommandTorque(ActuatorCommand& c, const RobotState& s,
                               double leg_limit, double wheel_limit) {
    Vec16 q = Vec16::Zero();
    Vec16 dq = Vec16::Zero();
    q.head<kLegJoints>() = s.q;
    dq.head<kLegJoints>() = s.dq;
    dq.tail<kNumLegs>() = s.dq_w;

    const Vec16 feedback = c.kp.cwiseProduct(c.q_des - q)
                         + c.kd.cwiseProduct(c.dq_des - dq);
    Vec16 total = c.tau_ff + feedback;
    total.head<kLegJoints>() = total.head<kLegJoints>()
        .cwiseMax(-leg_limit).cwiseMin(leg_limit);
    total.tail<kNumLegs>() = total.tail<kNumLegs>()
        .cwiseMax(-wheel_limit).cwiseMin(wheel_limit);
    c.tau_ff = total - feedback;
}

} // namespace wq
