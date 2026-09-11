#include "wq/robot_model.hpp"
#include "wq/math_utils.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <cstdio>
#include <cmath>
#include <stdexcept>

namespace wq {
namespace pin = pinocchio;

RobotModel::RobotModel(const ModelConfig& cfg) : cfg_(cfg) {
    if (!(cfg_.wheel_radius > 0.0) || !std::isfinite(cfg_.wheel_radius))
        throw std::invalid_argument("[RobotModel] wheel_radius phai huu han va > 0");
    if (cfg_.wheel_axis_local.norm() < 1e-9 || cfg_.ground_normal.norm() < 1e-9)
        throw std::invalid_argument("[RobotModel] wheel axis/ground normal khong hop le");

    pin::urdf::buildModel(cfg.urdf_path, pin::JointModelFreeFlyer(), model_);
    data_    = pin::Data(model_);
    data_fd_ = pin::Data(model_);

    // neutral() BAT BUOC: khop continuous can (cos,sin) = (1,0), khong phai 0
    q_   = pin::neutral(model_);
    v_   = VecXd::Zero(model_.nv);
    g_q_ = VecXd::Zero(model_.nv);
    J_tmp_.resize(6, model_.nv);

    auto findJoint = [&](const std::string& name, int& iq, int& iv, int& nq) {
        if (!model_.existJointName(name))
            throw std::runtime_error("[RobotModel] khong co khop: " + name);
        const auto jid = model_.getJointId(name);
        iq = model_.idx_qs[jid];
        iv = model_.idx_vs[jid];
        nq = model_.nqs[jid];
    };

    int dummy_nq = 0;
    for (int i = 0; i < kLegJoints; ++i)
        findJoint(cfg.leg_joints[i], leg_q_[i], leg_v_[i], dummy_nq);

    for (int i = 0; i < kNumLegs; ++i) {
        findJoint(cfg.wheel_joints[i], wheel_q_[i], wheel_v_[i], wheel_nq_[i]);
        if (!model_.existFrame(cfg.wheel_frames[i]))
            throw std::runtime_error("[RobotModel] khong co frame: "
                                     + cfg.wheel_frames[i]);
        wheel_fid_[i] = static_cast<int>(
            model_.getFrameId(cfg.wheel_frames[i], pin::BODY));
        Jc_[i].J.setZero(3, model_.nv);
    }

    mass_ = pin::computeTotalMass(model_);

    if (cfg_.verbose) {
        std::printf("[RobotModel] nq=%d nv=%d mass=%.3f kg r_wheel=%.4f m\n",
                    model_.nq, model_.nv, mass_, cfg_.wheel_radius);
        std::printf("[RobotModel] ky vong: nq = 7 + 12 + 4*nq_wheel, nv = 6 + 16 = 22\n");
        for (int i = 0; i < kNumLegs; ++i)
            std::printf("[RobotModel]   wheel[%d] '%s' nq=%d %s\n",
                        i, cfg_.wheel_joints[i].c_str(), wheel_nq_[i],
                        wheel_nq_[i] == 2 ? "(continuous -> cos/sin)" : "(revolute)");
        if (model_.nv != 22)
            std::printf("[RobotModel] ** CANH BAO: nv=%d != 22, kiem tra URDF **\n",
                        model_.nv);
    }
}

void RobotModel::packConfiguration(const RobotState& s) {
    q_.head<3>()     = s.p_WB;
    q_.segment<4>(3) = s.q_WB.normalized().coeffs();   // Pinocchio: (x,y,z,w)
    for (int i = 0; i < kLegJoints; ++i) q_[leg_q_[i]] = s.q[i];

    // Goc banh KHONG anh huong contact Jacobian: tam banh nam tren truc quay
    // nen banh tu xoay khong lam tam dich chuyen, truc quay cung khong doi huong.
    // => giu neutral vinh vien. Day la ly do RobotState khong can q_w.
    for (int i = 0; i < kNumLegs; ++i) {
        if (wheel_nq_[i] == 2) {
            q_[wheel_q_[i]]     = 1.0;   // cos
            q_[wheel_q_[i] + 1] = 0.0;   // sin
        } else {
            q_[wheel_q_[i]] = 0.0;
        }
    }
}

void RobotModel::packVelocity(const RobotState& s) {
    const Mat3 R = s.q_WB.toRotationMatrix();
    // !!! v cua JointModelFreeFlyer la he LOCAL, khong phai world !!!
    v_.head<3>()     = R.transpose() * s.v_WB;
    v_.segment<3>(3) = s.omega_B;
    for (int i = 0; i < kLegJoints; ++i) v_[leg_v_[i]]   = s.dq[i];
    for (int i = 0; i < kNumLegs; ++i)   v_[wheel_v_[i]] = s.dq_w[i];
}

void RobotModel::update(RobotState& s) {
    packConfiguration(s);
    packVelocity(s);

    pin::forwardKinematics(model_, data_, q_, v_);
    pin::updateFramePlacements(model_, data_);
    pin::computeJointJacobians(model_, data_, q_);
    pin::centerOfMass(model_, data_, q_, v_);
    pin::ccrba(model_, data_, q_, v_);                  // -> data_.Ig tai CoM
    pin::computeGeneralizedGravity(model_, data_, q_);
    g_q_ = data_.g;

    const Mat3   R  = s.q_WB.toRotationMatrix();
    const Vec3   n  = cfg_.ground_normal.normalized();
    const double rw = cfg_.wheel_radius;

    s.mass    = mass_;
    s.p_CoM_W = data_.com[0];
    s.I_G_W   = data_.Ig.inertia().matrix();            // da world-aligned
    s.g_B     = R.transpose() * Vec3(0.0, 0.0, -kGravity);

    for (int i = 0; i < kNumLegs; ++i) {
        J_tmp_.setZero();
        pin::getFrameJacobian(model_, data_, wheel_fid_[i],
                              pin::LOCAL_WORLD_ALIGNED, J_tmp_);

        const Vec3 p_w = data_.oMf[wheel_fid_[i]].translation();
        const Mat3 R_w = data_.oMf[wheel_fid_[i]].rotation();
        const Vec3 a_W = (R_w * cfg_.wheel_axis_local).normalized();

        // Diem cham san = tam banh dich xuong r_w theo phap tuyen.
        // Bo sot phan chenh r_w nay se gay SAI MOMEN PITCH CO HE THONG.
        const Vec3 p_c = p_w - rw * n;

        // Doi Jacobian tu tam banh -> diem cham:
        //   v_P = v_O + w x d,   d = p_c - p_w = -rw*n
        //   J_P = J_v + [d]x^T J_w = J_v - [d]x J_w = J_v + rw*[n]x J_w
        Jc_[i].J.noalias() = J_tmp_.topRows<3>()
                           + rw * skew(n) * J_tmp_.bottomRows<3>();
        Jc_[i].p_c_W    = p_c;
        const Vec3 roll = a_W.cross(n);
        Jc_[i].axis_W   = a_W;
        Jc_[i].e_roll_W = (roll.norm() > 1e-9)
                        ? roll.normalized() : Vec3::Zero(); // tranh NaN khi lat 90 do
        Jc_[i].offset_local = R_w.transpose() * (-rw * n);

        s.p_c_W[i]  = p_c;
        s.axis_W[i] = a_W;
    }
}

Vec3 RobotModel::contactMaterialPointAt(const VecXd& q, int leg,
                                        const Vec3& offset_local) const {
    pin::forwardKinematics(model_, data_fd_, q);
    pin::updateFramePlacements(model_, data_fd_);
    const auto& oMf = data_fd_.oMf[wheel_fid_[leg]];
    return oMf.translation() + oMf.rotation() * offset_local;
}

Vec16 RobotModel::toActuated(const VecXd& tau_full) const {
    Vec16 out = Vec16::Zero();
    for (int i = 0; i < kLegJoints; ++i) out[i] = tau_full[leg_v_[i]];
    for (int i = 0; i < kNumLegs; ++i)   out[kLegJoints + i] = tau_full[wheel_v_[i]];
    return out;
}

} // namespace wq
