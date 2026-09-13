#pragma once

#include "stand_wheel/pinocchio_model.hpp"

#include <Eigen/Core>

#include <array>
#include <vector>

namespace stand_wheel {

using ActiveContacts = std::vector<LegId>;

struct ContactStack {
  // Block i is the active_contacts[i] contact. All rows and forces are
  // expressed on world-aligned axes: [x, y, z] and [fx, fy, fz].
  ActiveContacts active_contacts;
  std::vector<Eigen::Vector3d> positions_world;
  Eigen::MatrixXd jacobian_world;
  Eigen::VectorXd jdot_v_world;
};

struct FullDynamicsTerms {
  Eigen::MatrixXd mass_matrix;
  Eigen::VectorXd nonlinear_effects;
  ContactStack contacts;
};

class FullDynamics {
 public:
  explicit FullDynamics(PinocchioModel& robot);

  // S is 16 x nv. generalizedActuation(tau) computes S.transpose() * tau.
  const Eigen::MatrixXd& selectionMatrix() const { return selection_matrix_; }
  const std::array<JointMapping, kNumActuatedJoints>& actuatorOrdering() const {
    return actuator_ordering_;
  }

  Eigen::VectorXd generalizedActuation(
      const Eigen::Ref<const Eigen::VectorXd>& tau_actuated) const;

  ContactStack contactStack(const Eigen::VectorXd& q,
                            const Eigen::VectorXd& v,
                            const ActiveContacts& active_contacts);

  Eigen::VectorXd generalizedContactForce(
      const ContactStack& contacts,
      const Eigen::Ref<const Eigen::VectorXd>& lambda_world) const;

  FullDynamicsTerms compute(const Eigen::VectorXd& q,
                            const Eigen::VectorXd& v,
                            const ActiveContacts& active_contacts);

  Eigen::VectorXd forwardDynamics(
      const Eigen::VectorXd& q, const Eigen::VectorXd& v,
      const Eigen::Ref<const Eigen::VectorXd>& tau_actuated,
      const ActiveContacts& active_contacts,
      const Eigen::Ref<const Eigen::VectorXd>& lambda_world);

 private:
  static void validateActiveContacts(const ActiveContacts& active_contacts);
  void validateVelocity(const Eigen::VectorXd& v) const;
  ContactStack makeContactStack(const Eigen::VectorXd& q,
                                const Eigen::VectorXd& v,
                                const ActiveContacts& active_contacts,
                                const ModelSnapshot& snapshot);

  PinocchioModel& robot_;
  pinocchio::Data dynamics_data_;
  Eigen::MatrixXd selection_matrix_;
  std::array<JointMapping, kNumActuatedJoints> actuator_ordering_;
};

}  // namespace stand_wheel
