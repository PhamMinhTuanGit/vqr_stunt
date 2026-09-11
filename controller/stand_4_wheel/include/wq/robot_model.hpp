#pragma once
#include "wq/types.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <string>

namespace wq {

struct ModelConfig {
    std::string urdf_path;
    std::array<std::string, kNumLegs>   wheel_frames;   // FL, FR, HL, HR
    std::array<std::string, kNumLegs>   wheel_joints;
    std::array<std::string, kLegJoints> leg_joints;
    double wheel_radius     = 0.091;
    Vec3   wheel_axis_local = -Vec3::UnitY();
    Vec3   ground_normal    = Vec3::UnitZ();
    bool   verbose          = true;
};

// Jacobian tiep xuc 3 x nv cua mot chan, tai DIEM CHAM SAN (khong phai truc banh)
struct ContactJacobian {
    MatXd J;          // 3 x nv
    Vec3  p_c_W;      // diem cham, world
    Vec3  axis_W;     // truc quay banh, world
    Vec3  e_roll_W;   // huong lan = axis x normal
    Vec3  offset_local; // vector tam banh -> diem vat lieu dang cham san
};

class RobotModel {
public:
    explicit RobotModel(const ModelConfig& cfg);

    // FK + Jacobian + g(q) + CoM + Ig. GOI DUNG MOT LAN moi chu ky.
    void update(RobotState& s);

    const ContactJacobian& contactJacobian(int leg) const { return Jc_[leg]; }
    const VecXd& generalizedGravity() const { return g_q_; }   // nv chieu

    // Trich 16 phan tu actuated tu vector nv chieu
    Vec16 toActuated(const VecXd& tau_full) const;

    int    nq() const { return model_.nq; }
    int    nv() const { return model_.nv; }
    double mass() const { return mass_; }
    double wheelRadius() const { return cfg_.wheel_radius; }
    int    legJointVIdx(int i)   const { return leg_v_[i]; }
    int    wheelJointVIdx(int i) const { return wheel_v_[i]; }

    // --- phuc vu test sai phan so ---
    const VecXd& configuration() const { return q_; }
    const VecXd& velocity()      const { return v_; }
    const pinocchio::Model& pinModel() const { return model_; }
    Vec3 contactMaterialPointAt(const VecXd& q, int leg,
                                const Vec3& offset_local) const;

private:
    void packConfiguration(const RobotState& s);
    void packVelocity(const RobotState& s);

    ModelConfig      cfg_;
    pinocchio::Model model_;
    pinocchio::Data  data_;
    mutable pinocchio::Data data_fd_;   // ban sao cho test, khong dung runtime

    VecXd  q_, v_, g_q_;
    double mass_ = 0.0;

    std::array<int, kLegJoints> leg_q_{}, leg_v_{};
    std::array<int, kNumLegs>   wheel_q_{}, wheel_v_{}, wheel_nq_{}, wheel_fid_{};
    std::array<ContactJacobian, kNumLegs> Jc_;
    MatXd J_tmp_;
};

} // namespace wq
