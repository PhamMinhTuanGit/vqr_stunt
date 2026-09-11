#pragma once
#include "wq/balance_qp.hpp"
#include "wq/logger.hpp"
#include "wq/fsm/fsm_state_iface.hpp"

namespace wq {

struct QpBalanceStateConfig {
    double blend_time    = 1.0;   // pha BLEND noi bo, KHONG tao state rieng
    int    max_qp_fail   = 25;    // 50 ms @ 500 Hz -> bo cuoc
    double fall_gz_ratio = -0.5;  // g_B.z > -0.5*g  => da nga
    double pd_kp_hold    = 60.0;  // PD giu tu the cuoi kStandUp trong luc blend
    double pd_kd_hold    = 3.0;
    bool   enable_log    = false;
    std::string log_path = "qp_balance.csv";
};

// Cam vao FSM san co. Khi tich hop that:
//   class StateQpBalance : public <BaseCuaBan> { ... }  va forward 4 ham ao.
class StateQpBalance : public FSMStateBase {
public:
    StateQpBalance(RobotModel& model, RobotIO& io,
                   const BalanceQpConfig& qp_cfg = BalanceQpConfig{},
                   const QpBalanceStateConfig& st_cfg = QpBalanceStateConfig{});

    void enter() override;
    void run()   override;
    void exit()  override;
    StateName checkChange() override;

    const BalanceQP& qp() const { return qp_; }
    double blendLambda() const { return lambda_; }

private:
    ActuatorCommand blendWithHoldPd(const ActuatorCommand& qp_cmd, double dt);

    RobotModel& model_;
    RobotIO&    io_;
    BalanceQP   qp_;
    QpBalanceStateConfig cfg_;

    RobotState s_;
    CsvLogger  log_;

    double lambda_    = 0.0;   // he so blend 0 -> 1
    double kd_wheel0_ = 0.0;   // damping banh ke thua tu kStandUp
    Vec12  q_hold_    = Vec12::Zero();
    int    fail_cnt_  = 0;
    bool   fallen_    = false;
};

} // namespace wq
