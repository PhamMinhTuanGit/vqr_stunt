/**
 * @file support_line_lqr.hpp
 * @brief Support-line frame and the single-input stabilizer for FL+HR.
 *
 * The controlled direction is defined by the actual FL-HR contact geometry:
 * tangent to the support line, lateral in the ground plane, and ground
 * normal. No world roll/pitch convention is used here.
 */
#pragma once

#include "wq/math_utils.hpp"
#include "wq/robot_model.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace wq {

struct SupportLineFrame {
    bool valid = false;
    Vec3 origin = Vec3::Zero();
    Vec3 tangent = Vec3::UnitX();  // FL -> HR, projected on the ground
    Vec3 lateral = Vec3::UnitY();  // tangent x normal
    Vec3 normal = Vec3::UnitZ();   // upward ground normal

    static SupportLineFrame fromContacts(const Vec3& p_fl, const Vec3& p_hr,
                                         const Vec3& ground_normal) {
        SupportLineFrame frame;
        if (!ground_normal.allFinite() || ground_normal.norm() < 1e-9)
            return frame;

        frame.normal = ground_normal.normalized();
        // Keep the contact normal upward for a deterministic sign convention.
        if (frame.normal.dot(Vec3::UnitZ()) < 0.0) frame.normal = -frame.normal;

        Vec3 line = p_hr - p_fl;
        line -= frame.normal * line.dot(frame.normal);
        if (!line.allFinite() || line.norm() < 1e-6) return frame;

        frame.tangent = line.normalized();
        frame.lateral = frame.tangent.cross(frame.normal);
        if (frame.lateral.norm() < 1e-9) return SupportLineFrame{};
        frame.lateral.normalize();
        // Project the origin onto the support plane. Only the lateral
        // coordinate is used for the CoM-line error.
        frame.origin = p_fl - frame.normal * p_fl.dot(frame.normal);
        frame.valid = frame.origin.allFinite();
        return frame;
    }

    Mat6 wrenchTransform() const {
        Mat6 T = Mat6::Zero();
        Mat3 basis = Mat3::Zero();
        basis.row(0) = tangent.transpose();
        basis.row(1) = lateral.transpose();
        basis.row(2) = normal.transpose();
        T.block<3, 3>(0, 0) = basis;
        T.block<3, 3>(3, 3) = basis;
        return T;
    }
};

struct SupportLineMetrics {
    bool valid = false;
    double com_line_error = 0.0;  // CoM displacement normal to FL-HR [m]
    double com_line_rate = 0.0;   // estimated rate normal to FL-HR [m/s]
    double angle = 0.0;           // base lean about the support line [rad]
    double rate = 0.0;            // angular rate about the support line [rad/s]
};

class SupportLineLqr {
public:
    void configure(double mass, double com_height, double dt) {
        using Mat4 = Eigen::Matrix4d;
        using Row4 = Eigen::Matrix<double, 1, 4>;
        const double h = std::max(com_height, 0.15);
        const double m = std::max(mass, 1.0);
        const double step = std::clamp(dt, 1e-4, 0.02);

        // x = [angle, angle_rate, CoM_lateral_error, CoM_lateral_rate].
        // A positive lateral force gives a positive moment about tangent for
        // the frame convention above.
        Mat4 Ac = Mat4::Zero();
        Ac(0, 1) = 1.0;
        Ac(1, 0) = kGravity / h;
        Ac(2, 3) = 1.0;
        const Eigen::Vector4d Bc(0.0, 1.0 / (m * h), 0.0, 1.0 / m);
        A_ = Mat4::Identity() + step * Ac;
        B_ = step * Bc;

        const Mat4 Q = (Eigen::Vector4d() << 800.0, 50.0, 40.0, 10.0)
                           .finished().asDiagonal();
        constexpr double R = 1.0;
        Mat4 P = Q;
        for (int i = 0; i < 1000; ++i) {
            const double denom = R + (B_.transpose() * P * B_)(0, 0);
            const Row4 gain = (B_.transpose() * P * A_) / denom;
            const Mat4 next = A_.transpose() * P * A_
                            - A_.transpose() * P * B_ * gain + Q;
            if ((next - P).norm() < 1e-10) {
                P = next;
                break;
            }
            P = next;
        }
        K_ = (B_.transpose() * P * A_)
           / (R + (B_.transpose() * P * B_)(0, 0));
        spectral_radius_ = (A_ - B_ * K_).eigenvalues().cwiseAbs().maxCoeff();
        stable_ = std::isfinite(spectral_radius_) && spectral_radius_ < 1.0;
    }

    SupportLineFrame frame(const RobotState& state,
                           const RobotModel& model) const {
        return SupportLineFrame::fromContacts(
            state.p_c_W[FL], state.p_c_W[HR], model.groundNormal());
    }

    SupportLineMetrics metrics(const RobotState& state,
                               const RobotModel& model) const {
        SupportLineMetrics result;
        const SupportLineFrame support = frame(state, model);
        if (!support.valid) return result;

        const Mat3 R = state.q_WB.toRotationMatrix();
        const Vec3 z_body = R.col(2);
        result.com_line_error =
            (state.p_CoM_W - support.origin).dot(support.lateral);
        result.com_line_rate = state.v_WB.dot(support.lateral);
        result.angle = std::atan2(z_body.dot(support.lateral),
                                  z_body.dot(support.normal));
        result.rate = (R * state.omega_B).dot(support.tangent);
        result.valid = std::isfinite(result.com_line_error)
                    && std::isfinite(result.com_line_rate)
                    && std::isfinite(result.angle)
                    && std::isfinite(result.rate);
        return result;
    }

    // Returns a scalar force in the support.lateral direction.
    double force(const RobotState& state, const RobotModel& model) const {
        const SupportLineFrame support = frame(state, model);
        const SupportLineMetrics e = metrics(state, model);
        if (!support.valid || !e.valid || !stable_) return 0.0;

        const Eigen::Vector4d x(e.angle, e.rate,
                                e.com_line_error, e.com_line_rate);
        const double max_force = 0.35 * state.mass * kGravity;
        // The finite-horizon cart-pole model intentionally supplies the
        // angular feedback. Add a small direct line-error term so a static
        // CoM offset is removed before UNLOAD even when the base angle is
        // momentarily close to zero.
        constexpr double kCom = 120.0;
        constexpr double kComRate = 20.0;
        const double u = -(K_ * x)(0)
                       - kCom * e.com_line_error
                       - kComRate * e.com_line_rate;
        return std::clamp(u, -max_force, max_force);
    }

    bool stable() const { return stable_; }
    double spectralRadius() const { return spectral_radius_; }
    const Eigen::Matrix<double, 1, 4>& gain() const { return K_; }

private:
    Eigen::Matrix4d A_ = Eigen::Matrix4d::Identity();
    Eigen::Vector4d B_ = Eigen::Vector4d::Zero();
    Eigen::Matrix<double, 1, 4> K_ = Eigen::Matrix<double, 1, 4>::Zero();
    double spectral_radius_ = 1.0;
    bool stable_ = false;
};

} // namespace wq
