// ===========================================================================
//  Demo doc lap: vong lap 500 Hz, IdleHold (moc 1) -> kQpBalance.
//  SimIO la STUB -- noi vao sim/SDK cua ban o 3 cho danh dau TODO.
// ===========================================================================
#include "vqr_names.hpp"
#include "wq/idle_hold.hpp"
#include "wq/fsm/state_qp_balance.hpp"

#include <cstdio>
#include <string>

using namespace wq;

#ifndef WQ_DEFAULT_URDF_PATH
#define WQ_DEFAULT_URDF_PATH "vqr_description/vqr_urdf/urdf/VQRWheel.urdf"
#endif

class SimIO : public RobotIO {
public:
    explicit SimIO(double dt) : dt_(dt) {}

    void readState(RobotState& s) override {
        // TODO(1): doc tu sim
        //   s.p_WB, s.v_WB  (he WORLD)
        //   s.q_WB, s.omega_B (he THAN)
        //   s.q, s.dq, s.dq_w, s.tau_w, s.tau_j
        s.t = t_;
    }

    void writeCommand(const ActuatorCommand& c) override {
        // TODO(2): gui xuong sim/SDK
        // Remap each command vector with controllerToRobotOrder() before SDK I/O.
        // tau_i = tau_ff[i] + kp[i]*(q_des[i]-q[i]) + kd[i]*(dq_des[i]-dq[i])
        (void)c;
    }

    double dt() const override { return dt_; }

    // kStandUp hien tai dung wheel_lock_kd_=0.8 va wheel kp=0.
    double wheelKdNow() const override { return 0.8; }

    void tick() { t_ += dt_; }

private:
    double dt_, t_ = 0.0;
};

int main(int argc, char** argv) {
    const std::string urdf = (argc > 1) ? argv[1] : WQ_DEFAULT_URDF_PATH;
    const double dt = 0.002;                 // 500 Hz

    RobotModel model(makeVqrConfig(urdf));
    SimIO io(dt);

    BalanceQpConfig qp_cfg;
    QpBalanceStateConfig st_cfg;
    st_cfg.enable_log = true;

    StateQpBalance st_balance(model, io, qp_cfg, st_cfg);

    IdleHold idle;
    RobotState s;

    // ---------------- MOC 1: idle hold, banh KHONG khoa ----------------
    io.readState(s);
    model.update(s);
    idle.onEnter(s);                          // LATCH mot lan duy nhat

    const int kIdleSteps = static_cast<int>(3.0 / dt);
    for (int k = 0; k < kIdleSteps; ++k) {
        io.readState(s);
        model.update(s);
        io.writeCommand(idle.update(s, dt));
        io.tick();
    }
    std::printf("[demo] IdleHold xong, chuyen sang kQpBalance\n");

    // ---------------- kQpBalance ----------------
    st_balance.enter();
    StateName next = StateName::kQpBalance;

    for (int k = 0; k < static_cast<int>(60.0 / dt); ++k) {
        st_balance.run();
        io.tick();

        next = st_balance.checkChange();
        if (next != StateName::kQpBalance) {
            std::printf("[demo] doi state -> %s\n", toString(next));
            st_balance.exit();
            break;
        }

        if (k % 250 == 0)
            std::printf("t=%6.2f  z=%.3f  gz=%+.2f  lam=%.2f  qp=%.1fus ok=%d\n",
                        s.t, s.p_WB.z(), s.g_B.z(), st_balance.blendLambda(),
                        st_balance.qp().lastSolveTimeUs(),
                        st_balance.qp().lastSolveOk());
    }
    return 0;
}
