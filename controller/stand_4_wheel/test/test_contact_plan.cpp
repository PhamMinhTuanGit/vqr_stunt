#include "vqr_names.hpp"
#include "wq/balance_qp.hpp"
#include <cstdio>

#ifndef WQ_DEFAULT_URDF_PATH
#define WQ_DEFAULT_URDF_PATH "vqr_description/vqr_urdf/urdf/VQRWheel.urdf"
#endif

int main(int argc, char** argv) {
    using namespace wq;
    const std::string urdf = argc > 1 ? argv[1] : WQ_DEFAULT_URDF_PATH;
    auto cfg = makeVqrConfig(urdf);
    cfg.verbose = false;
    RobotModel model(cfg);
    RobotState s;
    s.p_WB = Vec3(0.0, 0.0, 0.49);
    for (int leg = 0; leg < 4; ++leg)
        s.q.segment<3>(3 * leg) = Vec3(0.0, -0.65, 1.30);
    model.update(s);

    const double mg = s.mass * kGravity;
    ContactPlan plan;
    plan.active = {true, false, false, true};
    plan.fz_min << 5.0, 0.0, 0.0, 5.0;
    plan.fz_max << 1.5 * mg, 0.0, 0.0, 1.5 * mg;
    plan.fz_ref << 0.5 * mg, 0.0, 0.0, 0.5 * mg;
    s.contact = plan.active;

    BalanceQP qp(model);
    qp.setNominalJoint(s.q);
    qp.setTargetHeight(s.p_WB.z());
    qp.onEnter(s, plan);
    const auto cmd = qp.update(s, plan, 0.002);
    const auto& f = qp.lastForces();
    const bool inactive_zero = f.segment<3>(3 * FR).norm() < 1e-8
                            && f.segment<3>(3 * HL).norm() < 1e-8;
    const Vec3 normal = model.groundNormal();
    const bool support_bounded = f.segment<3>(3 * FL).dot(normal) >= 5.0 - 1e-8
                              && f.segment<3>(3 * HR).dot(normal) >= 5.0 - 1e-8;
    const bool qp_residual_finite = std::isfinite(qp.lastQpResidual());
    const bool pass = qp.lastSolveOk() && cmd.isFinite() && inactive_zero
                   && support_bounded && qp_residual_finite
                   && qp.wrenchRank() > 0
                   && qp.wrenchRank() <= 6 && std::isfinite(qp.wrenchCondition());
    std::printf("QP=%d inactive=%d bounds=%d rank=%d cond=%.3e projection=%.3e qp_residual=%.3e\n",
                qp.lastSolveOk(), inactive_zero, support_bounded, qp.wrenchRank(),
                qp.wrenchCondition(), qp.wrenchResidual(), qp.lastQpResidual());

    // UNLOAD keeps FR/HL as active contacts with a positive normal-force
    // floor, and BalanceQP still returns nonzero leg impedance for them.
    ContactPlan unload;
    unload.active = {true, true, true, true};
    unload.fz_min << 5.0, 2.0, 2.0, 5.0;
    unload.fz_max.setConstant(1.5 * mg);
    unload.fz_ref << 0.48 * mg, 0.02 * mg, 0.02 * mg, 0.48 * mg;
    unload.support_line_mode = true;
    qp.onEnter(s, unload);
    const auto unload_cmd = qp.update(s, unload, 0.002);
    const auto& unload_f = qp.lastForces();
    const bool unload_floor = unload_f.segment<3>(3 * FR).dot(normal) >= 2.0 - 1e-8
                           && unload_f.segment<3>(3 * HL).dot(normal) >= 2.0 - 1e-8;
    const bool impedance_kept = unload_cmd.kp.segment<3>(3 * FR).minCoeff() > 0.0
                             && unload_cmd.kp.segment<3>(3 * HL).minCoeff() > 0.0
                             && unload_cmd.kd.segment<3>(3 * FR).minCoeff() > 0.0
                             && unload_cmd.kd.segment<3>(3 * HL).minCoeff() > 0.0;
    std::printf("UNLOAD floor=%d impedance=%d qp=%d\n",
                unload_floor, impedance_kept, qp.lastSolveOk());
    return (pass && unload_floor && impedance_kept) ? 0 : 1;
}
