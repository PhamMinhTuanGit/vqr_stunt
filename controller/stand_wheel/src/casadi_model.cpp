#include "stand_wheel/casadi_model.hpp"

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <Eigen/Core>

#include <array>
#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

namespace stand_wheel {
namespace pin = pinocchio;

namespace {

using ADScalar = casadi::SX;
using ADModel = pin::ModelTpl<ADScalar>;
using ADData = ADModel::Data;
using ADVector = ADModel::ConfigVectorType;
using ADTangent = ADModel::TangentVectorType;
using ADMatrix = ADData::MatrixXs;
using ADVector3 = Eigen::Matrix<ADScalar, 3, 1>;

template <typename Derived>
casadi::SX toCasadi(const Eigen::MatrixBase<Derived>& matrix) {
  casadi::SX result(
      casadi::Sparsity::dense(matrix.rows(), matrix.cols()));
  pin::casadi::copy(matrix.derived(), result);
  return result;
}

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> skew(
    const Eigen::Matrix<Scalar, 3, 1>& value) {
  Eigen::Matrix<Scalar, 3, 3> result;
  result << Scalar(0), -value.z(), value.y(), value.z(), Scalar(0),
      -value.x(), -value.y(), value.x(), Scalar(0);
  return result;
}

std::string contactTag(const ActiveContacts& active_contacts) {
  if (active_contacts.empty()) {
    return "none";
  }
  std::string tag;
  for (LegId leg : active_contacts) {
    if (!tag.empty()) {
      tag += "_";
    }
    std::string label = legLabel(leg);
    for (char& value : label) {
      value = static_cast<char>(std::tolower(static_cast<unsigned char>(value)));
    }
    tag += label;
  }
  return tag;
}

struct SymbolicWheel {
  LegId leg = LegId::FL;
  int wheel_velocity_index = -1;
  ADVector3 center;
  ADVector3 contact;
  ADVector3 axle;
  ADVector3 rolling;
  ADVector3 lateral;
  ADVector3 normal;
  ADMatrix force_jacobian;
  ADMatrix carrier_jacobian;
};

}  // namespace

void CasadiModel::validateActiveContacts(
    const ActiveContacts& active_contacts) {
  std::array<bool, kNumLegs> seen = {false, false, false, false};
  for (LegId leg : active_contacts) {
    const int index = static_cast<int>(leg);
    if (index < 0 || index >= kNumLegs) {
      throw std::invalid_argument("symbolic contacts contain an invalid LegId");
    }
    if (seen[index]) {
      throw std::invalid_argument(std::string("duplicate symbolic contact: ") +
                                  legLabel(leg));
    }
    seen[index] = true;
  }
}

CasadiFunctions CasadiModel::build(
    const ActiveContacts& active_contacts) const {
  validateActiveContacts(active_contacts);
  const pin::Model& model = robot_.model();
  const int nq = model.nq;
  const int nv = model.nv;
  const int num_contact_rows =
      3 * static_cast<int>(active_contacts.size());
  const std::string tag = contactTag(active_contacts);

  ADModel ad_model = model.cast<ADScalar>();

  const casadi::SX q = casadi::SX::sym("q", nq);
  const casadi::SX v = casadi::SX::sym("v", nv);
  const casadi::SX tau =
      casadi::SX::sym("tau", kNumActuatedJoints);
  const casadi::SX lambda =
      casadi::SX::sym("lambda", num_contact_rows);
  const casadi::SX lambda_wheel =
      casadi::SX::sym("lambda_wheel", num_contact_rows);
  const casadi::SX mu_roll = casadi::SX::sym("mu_roll");
  const casadi::SX mu_lat = casadi::SX::sym("mu_lat");
  const casadi::SX dq = casadi::SX::sym("dq", nv);
  const casadi::SX q_other = casadi::SX::sym("q_other", nq);

  ADVector q_ad(nq);
  ADTangent v_ad(nv);
  ADTangent dq_ad(nv);
  ADVector q_other_ad(nq);
  pin::casadi::copy(q, q_ad);
  pin::casadi::copy(v, v_ad);
  pin::casadi::copy(dq, dq_ad);
  pin::casadi::copy(q_other, q_other_ad);

  const auto normalizeConfig = [&](ADVector& config_ad) {
    const int quat_idx = 3;
    ADScalar quat_norm = Eigen::numext::sqrt(
        config_ad.template segment<4>(quat_idx).squaredNorm() + ADScalar(1e-16));
    config_ad.template segment<4>(quat_idx) /= quat_norm;
    for (const WheelContactMapping& wheel : robot_.wheels()) {
      const int idx = wheel.wheel_joint.idx_q;
      ADScalar w_norm = Eigen::numext::sqrt(
          config_ad.template segment<2>(idx).squaredNorm() + ADScalar(1e-16));
      config_ad.template segment<2>(idx) /= w_norm;
    }
  };
  normalizeConfig(q_ad);
  normalizeConfig(q_other_ad);

  ADData mass_data(ad_model);
  pin::crba(ad_model, mass_data, q_ad);
  mass_data.M.template triangularView<Eigen::StrictlyLower>() =
      mass_data.M.transpose().template triangularView<Eigen::StrictlyLower>();
  const casadi::SX mass_matrix = toCasadi(mass_data.M);

  ADData nonlinear_data(ad_model);
  const ADTangent nonlinear_ad =
      pin::nonLinearEffects(ad_model, nonlinear_data, q_ad, v_ad);
  const casadi::SX nonlinear_effects = toCasadi(nonlinear_ad);

  ADData kinematics_data(ad_model);
  pin::forwardKinematics(ad_model, kinematics_data, q_ad);
  pin::updateFramePlacements(ad_model, kinematics_data);
  pin::computeJointJacobians(ad_model, kinematics_data, q_ad);
  const ADVector3 com_ad =
      pin::centerOfMass(ad_model, kinematics_data, q_ad);

  const ADVector3 ground_normal =
      robot_.groundNormalWorld().cast<ADScalar>();
  const ADVector3 configured_wheel_axis =
      robot_.wheelAxisLocal().cast<ADScalar>();

  std::vector<SymbolicWheel> symbolic_wheels;
  symbolic_wheels.reserve(kNumLegs);
  ADVector centers_ad(3 * kNumLegs);
  ADVector contacts_ad(3 * kNumLegs);
  ADMatrix all_contact_jacobians(3 * kNumLegs, nv);
  all_contact_jacobians.setZero();

  for (const WheelContactMapping& mapping : robot_.wheels()) {
    const int leg_index = static_cast<int>(mapping.leg);
    SymbolicWheel wheel;
    wheel.leg = mapping.leg;
    wheel.wheel_velocity_index = mapping.wheel_joint.idx_v;
    const auto& placement = kinematics_data.oMf[mapping.frame_id];
    wheel.center = placement.translation();
    wheel.normal = ground_normal;
    wheel.axle = placement.rotation() * configured_wheel_axis;
    wheel.axle /= Eigen::numext::sqrt(wheel.axle.squaredNorm());

    ADVector3 radial_down =
        wheel.normal - wheel.axle.dot(wheel.normal) * wheel.axle;
    radial_down /= Eigen::numext::sqrt(radial_down.squaredNorm());
    const ADVector3 offset = -ADScalar(robot_.wheelRadius()) * radial_down;
    wheel.contact = wheel.center + offset;

    ADVector3 axle_cross_normal = wheel.axle.cross(wheel.normal);
    wheel.rolling = axle_cross_normal /
                    Eigen::numext::sqrt(axle_cross_normal.squaredNorm());
    wheel.lateral = wheel.normal.cross(wheel.rolling);
    wheel.lateral /= Eigen::numext::sqrt(wheel.lateral.squaredNorm());

    ADMatrix frame_jacobian(6, nv);
    frame_jacobian.setZero();
    pin::getFrameJacobian(ad_model, kinematics_data, mapping.frame_id,
                          pin::LOCAL_WORLD_ALIGNED, frame_jacobian);
    wheel.force_jacobian =
        frame_jacobian.topRows(3) -
        skew(offset) * frame_jacobian.bottomRows(3);
    wheel.carrier_jacobian = wheel.force_jacobian;
    wheel.carrier_jacobian.col(wheel.wheel_velocity_index).setZero();

    centers_ad.template segment<3>(3 * leg_index) = wheel.center;
    contacts_ad.template segment<3>(3 * leg_index) = wheel.contact;
    all_contact_jacobians.middleRows(3 * leg_index, 3) =
        wheel.force_jacobian;

    if (static_cast<int>(symbolic_wheels.size()) != leg_index) {
      throw std::runtime_error(
          "Step 1 wheel mappings are not in canonical FL/FR/HL/HR order");
    }
    symbolic_wheels.push_back(wheel);
  }

  ADMatrix active_contact_jacobian(num_contact_rows, nv);
  ADMatrix rolling_matrix_ad(num_contact_rows, nv);
  ADMatrix hard_matrix_ad(2 * static_cast<int>(active_contacts.size()), nv);
  ADMatrix lateral_matrix_ad(static_cast<int>(active_contacts.size()), nv);
  ADMatrix force_transform_ad(num_contact_rows, num_contact_rows);
  active_contact_jacobian.setZero();
  rolling_matrix_ad.setZero();
  hard_matrix_ad.setZero();
  lateral_matrix_ad.setZero();
  force_transform_ad.setZero();
  for (std::size_t i = 0; i < active_contacts.size(); ++i) {
    const SymbolicWheel& wheel =
        symbolic_wheels.at(static_cast<int>(active_contacts[i]));
    const int row = 3 * static_cast<int>(i);
    active_contact_jacobian.middleRows(row, 3) = wheel.force_jacobian;
    rolling_matrix_ad.row(row) =
        wheel.normal.transpose() * wheel.carrier_jacobian;
    rolling_matrix_ad.row(row + 1) =
        wheel.lateral.transpose() * wheel.carrier_jacobian;
    rolling_matrix_ad.row(row + 2) =
        wheel.rolling.transpose() * wheel.carrier_jacobian;
    rolling_matrix_ad(row + 2, wheel.wheel_velocity_index) -=
        ADScalar(robot_.wheelRadius());

    const int hard_row = 2 * static_cast<int>(i);
    hard_matrix_ad.row(hard_row) = rolling_matrix_ad.row(row);
    hard_matrix_ad.row(hard_row + 1) = rolling_matrix_ad.row(row + 2);
    lateral_matrix_ad.row(static_cast<int>(i)) =
        rolling_matrix_ad.row(row + 1);
    force_transform_ad.template block<3, 1>(row, row) = wheel.rolling;
    force_transform_ad.template block<3, 1>(row, row + 1) = wheel.lateral;
    force_transform_ad.template block<3, 1>(row, row + 2) = wheel.normal;
  }
  const casadi::SX contact_jacobian =
      toCasadi(active_contact_jacobian);
  const casadi::SX rolling_matrix = toCasadi(rolling_matrix_ad);
  const casadi::SX rolling_residual =
      casadi::SX::mtimes(rolling_matrix, v);
  const casadi::SX hard_matrix = toCasadi(hard_matrix_ad);
  const casadi::SX hard_residual =
      casadi::SX::mtimes(hard_matrix, v);
  const casadi::SX lateral_slip =
      casadi::SX::mtimes(toCasadi(lateral_matrix_ad), v);
  const casadi::SX force_transform = toCasadi(force_transform_ad);
  const casadi::SX lambda_world_from_wheel =
      casadi::SX::mtimes(force_transform, lambda_wheel);
  const casadi::SX lambda_wheel_from_world =
      casadi::SX::mtimes(force_transform.T(), lambda);

  casadi::SX friction_margins =
      casadi::SX::zeros(5 * static_cast<int>(active_contacts.size()), 1);
  for (std::size_t i = 0; i < active_contacts.size(); ++i) {
    const int force_row = 3 * static_cast<int>(i);
    const int margin_row = 5 * static_cast<int>(i);
    const casadi::SX force_roll = lambda_wheel(force_row);
    const casadi::SX force_lateral = lambda_wheel(force_row + 1);
    const casadi::SX force_normal = lambda_wheel(force_row + 2);
    friction_margins(margin_row) = force_normal;
    friction_margins(margin_row + 1) =
        mu_roll * force_normal - force_roll;
    friction_margins(margin_row + 2) =
        mu_roll * force_normal + force_roll;
    friction_margins(margin_row + 3) =
        mu_lat * force_normal - force_lateral;
    friction_margins(margin_row + 4) =
        mu_lat * force_normal + force_lateral;
  }

  casadi::SX selection_transpose =
      casadi::SX::zeros(nv, kNumActuatedJoints);
  const Eigen::MatrixXd& selection = dynamics_.selectionMatrix();
  for (int row = 0; row < selection.rows(); ++row) {
    for (int col = 0; col < selection.cols(); ++col) {
      if (selection(row, col) != 0.0) {
        selection_transpose(col, row) = selection(row, col);
      }
    }
  }
  const casadi::SX generalized_actuation =
      casadi::SX::mtimes(selection_transpose, tau);
  const casadi::SX generalized_contact_force =
      casadi::SX::mtimes(contact_jacobian.T(), lambda);
  const casadi::SX generalized_force =
      generalized_actuation + generalized_contact_force;

  ADTangent generalized_force_ad(nv);
  pin::casadi::copy(generalized_force, generalized_force_ad);
  ADData aba_data(ad_model);
  const ADTangent vdot_ad =
      pin::aba(ad_model, aba_data, q_ad, v_ad, generalized_force_ad);
  const casadi::SX vdot = toCasadi(vdot_ad);

  const ADVector3 support_start =
      symbolic_wheels.at(static_cast<int>(LegId::FL)).contact;
  const ADVector3 support_end =
      symbolic_wheels.at(static_cast<int>(LegId::HR)).contact;
  ADVector3 support_delta = support_end - support_start;
  support_delta -= ground_normal.dot(support_delta) * ground_normal;
  const ADVector3 support_direction =
      support_delta / Eigen::numext::sqrt(support_delta.squaredNorm());
  ADVector3 com_horizontal = com_ad - support_start;
  com_horizontal -= ground_normal.dot(com_horizontal) * ground_normal;
  const ADVector3 com_perpendicular =
      com_horizontal - support_direction.dot(com_horizontal) * support_direction;
  ADVector support_geometry_ad(13);
  support_geometry_ad.template segment<3>(0) = support_start;
  support_geometry_ad.template segment<3>(3) = support_end;
  support_geometry_ad.template segment<3>(6) = support_direction;
  support_geometry_ad.template segment<3>(9) = com_perpendicular;
  support_geometry_ad[12] = com_perpendicular.squaredNorm();

  const ADVector q_next_ad = pin::integrate(ad_model, q_ad, dq_ad);
  const casadi::SX q_next = toCasadi(q_next_ad);
  const ADTangent difference_ad =
      pin::difference(ad_model, q_ad, q_other_ad);
  const casadi::SX difference = toCasadi(difference_ad);

  CasadiFunctions functions;
  functions.active_contacts = active_contacts;
  functions.mass_matrix = casadi::Function(
      "sw_mass_" + tag, {q}, {mass_matrix});
  functions.nonlinear_effects = casadi::Function(
      "sw_nle_" + tag, {q, v}, {nonlinear_effects});
  functions.generalized_actuation = casadi::Function(
      "sw_actuation_" + tag, {tau}, {generalized_actuation});
  functions.contact_jacobian = casadi::Function(
      "sw_contact_jacobian_" + tag, {q}, {contact_jacobian});
  functions.generalized_contact_force = casadi::Function(
      "sw_contact_force_" + tag, {q, lambda},
      {generalized_contact_force});
  functions.forward_dynamics = casadi::Function(
      "sw_vdot_" + tag, {q, v, tau, lambda}, {vdot});
  functions.center_of_mass = casadi::Function(
      "sw_com_" + tag, {q}, {toCasadi(com_ad)});
  functions.wheel_centers = casadi::Function(
      "sw_wheel_centers_" + tag, {q}, {toCasadi(centers_ad)});
  functions.contact_positions = casadi::Function(
      "sw_contact_positions_" + tag, {q}, {toCasadi(contacts_ad)});
  functions.contact_jacobians_all = casadi::Function(
      "sw_contact_jacobians_all_" + tag, {q},
      {toCasadi(all_contact_jacobians)});
  functions.support_line = casadi::Function(
      "sw_support_line_" + tag, {q}, {toCasadi(support_geometry_ad)});
  functions.rolling_matrix = casadi::Function(
      "sw_rolling_matrix_" + tag, {q}, {rolling_matrix});
  functions.rolling_residual = casadi::Function(
      "sw_rolling_residual_" + tag, {q, v}, {rolling_residual});
  functions.hard_constraint_matrix = casadi::Function(
      "sw_hard_matrix_" + tag, {q}, {hard_matrix});
  functions.hard_constraint_residual = casadi::Function(
      "sw_hard_residual_" + tag, {q, v}, {hard_residual});
  functions.lateral_slip = casadi::Function(
      "sw_lateral_slip_" + tag, {q, v}, {lateral_slip});
  functions.force_world_from_wheel = casadi::Function(
      "sw_force_world_from_wheel_" + tag, {q, lambda_wheel},
      {lambda_world_from_wheel});
  functions.force_wheel_from_world = casadi::Function(
      "sw_force_wheel_from_world_" + tag, {q, lambda},
      {lambda_wheel_from_world});
  functions.friction_margins = casadi::Function(
      "sw_friction_margins_" + tag, {lambda_wheel, mu_roll, mu_lat},
      {friction_margins});
  functions.integrate = casadi::Function(
      "sw_integrate_" + tag, {q, dq}, {q_next});
  functions.difference = casadi::Function(
      "sw_difference_" + tag, {q, q_other}, {difference});

  const casadi::SX vdot_at_integrated_q =
      functions.forward_dynamics(
          casadi::SXVector{q_next, v, tau, lambda})
          .at(0);
  functions.vdot_derivatives = casadi::Function(
      "sw_vdot_derivatives_" + tag, {q, v, tau, lambda, dq},
      {casadi::SX::jacobian(vdot_at_integrated_q, dq),
       casadi::SX::jacobian(vdot, v), casadi::SX::jacobian(vdot, tau),
       casadi::SX::jacobian(vdot, lambda)});

  const casadi::SX rolling_at_integrated_q =
      functions.rolling_residual(casadi::SXVector{q_next, v}).at(0);
  functions.rolling_derivatives = casadi::Function(
      "sw_rolling_derivatives_" + tag, {q, v, dq},
      {casadi::SX::jacobian(rolling_at_integrated_q, dq),
       casadi::SX::jacobian(rolling_residual, v)});

  const casadi::SX com_at_integrated_q =
      functions.center_of_mass(casadi::SXVector{q_next}).at(0);
  functions.com_derivative = casadi::Function(
      "sw_com_derivative_" + tag, {q, dq},
      {casadi::SX::jacobian(com_at_integrated_q, dq)});

  return functions;
}

}  // namespace stand_wheel
