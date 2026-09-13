#pragma once

#include <pinocchio/autodiff/casadi.hpp>

#include "stand_wheel/full_dynamics.hpp"

namespace stand_wheel {

struct CasadiFunctions {
  ActiveContacts active_contacts;

  casadi::Function mass_matrix;               // q -> M (nv x nv)
  casadi::Function nonlinear_effects;          // q,v -> h (nv)
  casadi::Function generalized_actuation;      // tau(16) -> S^T tau (nv)
  casadi::Function contact_jacobian;           // q -> J_force (3*Nc x nv)
  casadi::Function generalized_contact_force;  // q,lambda -> J^T lambda
  casadi::Function forward_dynamics;           // q,v,tau,lambda -> vdot

  casadi::Function center_of_mass;        // q -> CoM (3)
  casadi::Function wheel_centers;         // q -> [FL;FR;HL;HR] (12)
  casadi::Function contact_positions;     // q -> [FL;FR;HL;HR] (12)
  casadi::Function contact_jacobians_all; // q -> [J_FL;J_FR;J_HL;J_HR]
  casadi::Function support_line;          // q -> 13-value FL-HR geometry

  casadi::Function rolling_matrix;    // q -> A_roll (3*Nc x nv)
  casadi::Function rolling_residual;  // q,v -> A_roll*v (3*Nc)
  casadi::Function hard_constraint_matrix;    // q -> A_hard (2*Nc x nv)
  casadi::Function hard_constraint_residual;  // q,v -> A_hard*v (2*Nc)
  casadi::Function lateral_slip;               // q,v -> s_lateral (Nc)
  casadi::Function force_world_from_wheel;     // q,lambda_WC -> lambda_world
  casadi::Function force_wheel_from_world;     // q,lambda_world -> lambda_WC
  casadi::Function friction_margins;  // lambda_WC,mu_roll,mu_lat -> 5*Nc
  casadi::Function integrate;         // q,dv -> Pinocchio integrate(q,dv)
  casadi::Function difference;        // q0,q1 -> Pinocchio difference(q0,q1)

  // Manifold-aware q derivatives use dq=0 when evaluated. The last input is a
  // tangent perturbation passed through Pinocchio integrate(q,dq).
  casadi::Function vdot_derivatives;     // q,v,tau,lambda,dq -> Jq,Jv,Jtau,Jlambda
  casadi::Function rolling_derivatives;  // q,v,dq -> Jq,Jv
  casadi::Function com_derivative;       // q,dq -> Jq
};

class CasadiModel {
 public:
  CasadiModel(PinocchioModel& robot, const FullDynamics& dynamics)
      : robot_(robot), dynamics_(dynamics) {}

  CasadiFunctions build(const ActiveContacts& active_contacts) const;

 private:
  static void validateActiveContacts(const ActiveContacts& active_contacts);

  PinocchioModel& robot_;
  const FullDynamics& dynamics_;
};

}  // namespace stand_wheel
