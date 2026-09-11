// ===========================================================================
//  Kiem chung contact Jacobian bang SAI PHAN SO.
//  Test RE NHAT va bat duoc gan het loi: sai truc banh, sai ban kinh,
//  sai frame, sai quy uoc LOCAL / LOCAL_WORLD_ALIGNED, sai phep doi diem cham.
//
//  CHAY TRUOC KHI CAM VAO ROBOT.
// ===========================================================================
#include "vqr_names.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <cstdio>
#include <cmath>

using namespace wq;

#ifndef WQ_DEFAULT_URDF_PATH
#define WQ_DEFAULT_URDF_PATH "vqr_description/vqr_urdf/urdf/VQRWheel.urdf"
#endif

int main(int argc, char** argv) {
    // Verify the controller-order <-> SDK robot-order boundary mapping.
    Vec16 seq;
    for (int i = 0; i < kNumAct; ++i) seq[i] = static_cast<double>(i);
    const Vec16 wire = controllerToRobotOrder(seq);
    const bool map_ok = robotToControllerOrder(wire).isApprox(seq, 0.0)
                     && wire[3] == 12.0 && wire[7] == 13.0
                     && wire[11] == 14.0 && wire[15] == 15.0;
    if (!map_ok) {
        std::fprintf(stderr, "[FAIL] actuator order remap\n");
        return 1;
    }

    RobotState limit_state;
    ActuatorCommand limit_cmd;
    limit_cmd.q_des.head<kLegJoints>().setConstant(10.0);
    limit_cmd.kp.head<kLegJoints>().setConstant(100.0);
    limit_cmd.dq_des.tail<kNumLegs>().setConstant(100.0);
    limit_cmd.kd.tail<kNumLegs>().setConstant(10.0);
    limitCommandTorque(limit_cmd, limit_state, 60.0, 20.0);
    const Vec16 limited_total = limit_cmd.tau_ff
        + limit_cmd.kp.cwiseProduct(limit_cmd.q_des)
        + limit_cmd.kd.cwiseProduct(limit_cmd.dq_des);
    if (limited_total.head<kLegJoints>().cwiseAbs().maxCoeff() > 60.0 + 1e-12
        || limited_total.tail<kNumLegs>().cwiseAbs().maxCoeff() > 20.0 + 1e-12) {
        std::fprintf(stderr, "[FAIL] hybrid command torque limit\n");
        return 1;
    }

    const std::string urdf = (argc > 1) ? argv[1] : WQ_DEFAULT_URDF_PATH;
    ModelConfig mc = makeVqrConfig(urdf);
    mc.verbose = true;
    RobotModel model(mc);

    // Tu the thu nghiem BAT KY (tranh cau hinh ky di / doi xung hoan hao)
    RobotState s;
    s.p_WB = Vec3(0.13, -0.07, 0.34);
    s.q_WB = Quat(Eigen::AngleAxisd(0.21, Vec3(0.3, -0.5, 0.8).normalized()));
    for (int i = 0; i < kNumLegs; ++i)
        s.q.segment<3>(3 * i) = Vec3(0.05 + 0.03 * i,
                                     -0.82 + 0.05 * i,
                                      1.61 - 0.04 * i);

    model.update(s);

    const auto&  pm  = model.pinModel();
    const VecXd  q0  = model.configuration();
    const double eps = 1e-6;

    std::printf("\n--- kiem chung Jacobian (eps=%.0e) ---\n", eps);
    bool all_pass = true;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const MatXd& J   = model.contactJacobian(leg).J;
        const Vec3 offset_local = model.contactJacobian(leg).offset_local;
        const Vec3 p0 = model.contactMaterialPointAt(q0, leg, offset_local);
        double max_err   = 0.0;
        int    worst_dof = -1;

        for (int k = 0; k < model.nv(); ++k) {
            VecXd dv = VecXd::Zero(model.nv());
            dv[k] = eps;

            // integrate: xu ly dung ca freeflyer lan khop continuous
            const VecXd qp = pinocchio::integrate(pm, q0,  dv);
            const VecXd qm = pinocchio::integrate(pm, q0, -dv);

            // Keep the same wheel material point selected at q0. Recomputing
            // center-r*world_normal at qp/qm would select a different material
            // point and is not the derivative represented by the contact J.
            const Vec3 num =
                (model.contactMaterialPointAt(qp, leg, offset_local)
               - model.contactMaterialPointAt(qm, leg, offset_local))
                / (2.0 * eps);
            const Vec3 ana = J.col(k);

            const double e = (num - ana).norm();
            if (e > max_err) { max_err = e; worst_dof = k; }
        }

        // LUU Y: cot ung voi khop BANH phai xap xi 0 theo phuong thang dung
        // va khac 0 theo huong lan -- do la dac trung cua robot co banh.
        const Vec3 e_roll = model.contactJacobian(leg).e_roll_W;
        const int  wv     = model.wheelJointVIdx(leg);
        const double roll_gain = J.col(wv).dot(e_roll);

        // With a tilted wheel axis, only its component tangent to the ground
        // contributes: J_wheel.e_roll = -r*|axis x normal|.
        const double expected_roll = -model.wheelRadius()
                                   * s.axis_W[leg].cross(Vec3::UnitZ()).norm();
        const bool rolling_pass = std::abs(roll_gain - expected_roll) < 1e-6;
        const bool pass = (max_err < 1e-4) && rolling_pass;
        all_pass &= pass;

        std::printf("%s leg[%d] p_c=(%.4f %.4f %.4f) err_max=%.3e @dof=%d  "
                    "J_wheel.e_roll=%+.4f (ky vong %+.4f)\n",
                    pass ? "[PASS]" : "[FAIL]", leg,
                    p0.x(), p0.y(), p0.z(), max_err, worst_dof,
                    roll_gain, expected_roll);
    }

    // Truc banh khi robot gan thang dung phai xap xi (0, +-1, 0)
    std::printf("\n--- truc quay banh (world) ---\n");
    for (int i = 0; i < kNumLegs; ++i)
        std::printf("  axis_W[%d] = (%+.3f %+.3f %+.3f)\n",
                    i, s.axis_W[i].x(), s.axis_W[i].y(), s.axis_W[i].z());

    std::printf("\n%s\n", all_pass ? "=== TAT CA PASS ==="
                                   : "=== CO LOI: xem lai wheel_axis_local / "
                                     "wheel_radius / ten frame ===");
    return all_pass ? 0 : 1;
}
