/**
 * @file two_wheel_states.hpp
 * @brief FL+HR diagonal-support transition and balance state.
 *
 * The transition is deliberately explicit:
 * SHIFT_COM -> UNLOAD_FR_HL -> LIFT_FR_HL -> TWO_WHEEL_BALANCE.
 * QP remains the contact-force allocator in every phase.
 */
#pragma once

#include "state_base.h"
#include "vqr_names.hpp"
#include "wq/balance_qp.hpp"
#include "wq/logger.hpp"
#include "wq/math_utils.hpp"
#include "wq/support_line_lqr.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <limits.h>
#include <memory>
#include <stdexcept>
#include <unistd.h>

namespace two_wheel_detail {

constexpr std::array<int, 2> kSupport = {wq::FL, wq::HR};
constexpr std::array<int, 2> kSwing = {wq::FR, wq::HL};

enum class Phase {
    SHIFT_COM = 0,
    UNLOAD_FR_HL = 1,
    LIFT_FR_HL = 2,
    TWO_WHEEL_BALANCE = 3
};

inline const char* phaseName(Phase phase) {
    switch (phase) {
        case Phase::SHIFT_COM:       return "SHIFT_COM";
        case Phase::UNLOAD_FR_HL:    return "UNLOAD_FR_HL";
        case Phase::LIFT_FR_HL:      return "LIFT_FR_HL";
        case Phase::TWO_WHEEL_BALANCE:return "TWO_WHEEL_BALANCE";
    }
    return "UNKNOWN";
}

struct Config {
    double shift_time = 1.5;
    double unload_time = 1.5;
    double lift_time = 1.0;
    double phase_timeout = 4.0;
    double stable_dwell = 0.15;

    double swing_clearance = 0.05;
    double unload_force_fraction = 0.03;
    double unload_floor_fraction = 0.008;
    double unload_min_force = 1.0;
    double support_force_min = 5.0;
    double contact_threshold = 3.0;

    double line_error_limit = 0.025;
    double line_angle_limit = 8.0 * M_PI / 180.0;
    double line_rate_limit = 0.60;
    double stable_omega_limit = 0.80;
    double qp_residual_limit = 100.0;
    double soft_angle = 15.0 * M_PI / 180.0;
    double hard_angle = 30.0 * M_PI / 180.0;
    double hard_omega = 3.0;
    double support_loss_timeout = 0.10;

    double swing_kp = 250.0;
    double swing_kd = 20.0;
    bool enable_log = true;
    std::string transition_log = "two_wheel_transition.csv";
    std::string balance_log = "two_wheel_balance.csv";
};

inline std::string findUrdfPath() {
    const std::string cwd = GetAbsPath();
    std::array<std::string, 7> candidates = {
        "", "", "",
        cwd + "/vqr_description/vqr_urdf/urdf/VQRWheel.urdf",
        cwd + "/../vqr_description/vqr_urdf/urdf/VQRWheel.urdf",
        "vqr_description/vqr_urdf/urdf/VQRWheel.urdf",
        "../vqr_description/vqr_urdf/urdf/VQRWheel.urdf"
    };
    char exe_buf[PATH_MAX];
    const ssize_t len = readlink("/proc/self/exe", exe_buf, sizeof(exe_buf) - 1);
    if (len > 0) {
        exe_buf[len] = '\0';
        const std::string exe_path(exe_buf);
        const std::size_t slash = exe_path.find_last_of('/');
        if (slash != std::string::npos) {
            const std::string bin_dir = exe_path.substr(0, slash);
            candidates[0] = bin_dir + "/../vqr_description/vqr_urdf/urdf/VQRWheel.urdf";
            candidates[1] = bin_dir + "/vqr_description/vqr_urdf/urdf/VQRWheel.urdf";
            candidates[2] = bin_dir + "/../third_party/deep_robotics_model/VQRWheel/VQRWheel_urdf/urdf/VQRWheel.urdf";
        }
    }
    for (const auto& path : candidates)
        if (!path.empty() && access(path.c_str(), F_OK) == 0) return path;
    return "";
}

inline double smooth(double x) {
    double value = 0.0;
    double derivative = 0.0;
    wq::quintic(x, value, derivative);
    return value;
}

class Runtime {
public:
    Runtime(std::shared_ptr<RobotInterface> robot, const Config& cfg = Config{})
        : robot_(std::move(robot)), cfg_(cfg) {
        const std::string urdf = findUrdfPath();
        if (urdf.empty()) throw std::runtime_error("TwoWheel: VQRWheel.urdf not found");
        auto model_cfg = wq::makeVqrConfig(urdf);
        model_cfg.verbose = false;
        model_ = std::make_unique<wq::RobotModel>(model_cfg);
        qp_ = std::make_unique<wq::BalanceQP>(*model_);
    }

    bool read() {
        BaseContactState base;
        state_valid_ = robot_->GetBaseContactState(base) && base.valid;
        const VecXf q_robot = robot_->GetJointPosition();
        const VecXf dq_robot = robot_->GetJointVelocity();
        const VecXf tau_robot = robot_->GetJointTorque();
        if (!state_valid_ || q_robot.size() != 16 || dq_robot.size() != 16
            || tau_robot.size() != 16) {
            state_valid_ = false;
            return false;
        }
        if (!base.position_world.allFinite() || !base.linear_velocity_world.allFinite()
            || !base.quaternion_wxyz.allFinite()
            || !base.wheel_normal_force.allFinite()
            || base.quaternion_wxyz.norm() < 1e-6) {
            state_valid_ = false;
            return false;
        }

        const wq::Vec16 q = wq::robotToControllerOrder(q_robot.cast<double>());
        const wq::Vec16 dq = wq::robotToControllerOrder(dq_robot.cast<double>());
        const wq::Vec16 tau = wq::robotToControllerOrder(tau_robot.cast<double>());
        s_.q = q.head<12>();
        s_.dq = dq.head<12>();
        s_.dq_w = dq.tail<4>();
        s_.tau_j = tau.head<12>();
        s_.tau_w = tau.tail<4>();
        s_.p_WB = base.position_world.cast<double>();
        s_.v_WB = base.linear_velocity_world.cast<double>();
        s_.q_WB = wq::Quat(base.quaternion_wxyz[0], base.quaternion_wxyz[1],
                           base.quaternion_wxyz[2], base.quaternion_wxyz[3]).normalized();
        s_.omega_B = robot_->GetImuOmega().cast<double>();
        normal_force_ = base.wheel_normal_force.cast<double>().cwiseMax(0.0);
        s_.t = robot_->GetInterfaceTimeStamp();
        for (int i = 0; i < wq::kNumLegs; ++i)
            s_.contact[i] = normal_force_[i] > cfg_.contact_threshold;

        model_->update(s_);
        state_valid_ = s_.q_WB.coeffs().allFinite() && s_.p_WB.allFinite()
                    && s_.v_WB.allFinite() && s_.p_CoM_W.allFinite();
        return state_valid_;
    }

    void begin(const wq::ContactPlan& plan) {
        nominal_q_ = s_.q;
        qp_->setNominalJoint(nominal_q_);
        qp_->setTargetHeight(s_.p_WB.dot(model_->groundNormal()));
        qp_->setTargetXY(s_.p_WB.head<2>());
        qp_->onEnter(s_, plan);

        const wq::SupportLineFrame frame = lqr_.frame(s_, *model_);
        const double height = frame.valid
            ? (s_.p_CoM_W - frame.origin).dot(frame.normal) : 0.30;
        lqr_.configure(s_.mass, height, 0.002);
        configured_ = lqr_.stable();
        for (int leg = 0; leg < wq::kNumLegs; ++leg)
            wheel_anchor_[leg] = model_->wheelCenterPosition(leg);
    }

    wq::ContactPlan fourPlan() const {
        wq::ContactPlan p;
        const double mg = s_.mass * wq::kGravity;
        p.active = {true, true, true, true};
        p.fz_min.setConstant(cfg_.support_force_min);
        p.fz_max.setConstant(1.5 * mg);
        p.fz_ref.setConstant(0.25 * mg);
        p.support_line_mode = true;
        return p;
    }

    wq::ContactPlan unloadPlan(double progress) const {
        const double a = std::clamp(progress, 0.0, 1.0);
        const double mg = s_.mass * wq::kGravity;
        const double swing_floor = std::max(cfg_.unload_min_force,
                                            cfg_.unload_floor_fraction * mg);
        const double swing_ref = (1.0 - a) * 0.25 * mg + a * swing_floor;
        const double support_ref = 0.5 * (mg - 2.0 * swing_ref);
        const double swing_hi = std::max(swing_floor,
            (1.0 - a) * 0.35 * mg + a * 0.05 * mg);

        wq::ContactPlan p;
        p.active = {true, true, true, true};
        p.fz_min << cfg_.support_force_min, swing_floor,
                    swing_floor, cfg_.support_force_min;
        p.fz_max << 1.5 * mg, swing_hi, swing_hi, 1.5 * mg;
        p.fz_ref << support_ref, swing_ref, swing_ref, support_ref;
        p.support_line_mode = true;
        return p;
    }

    wq::ContactPlan twoPlan() const {
        const double mg = s_.mass * wq::kGravity;
        wq::ContactPlan p;
        p.active = {true, false, false, true};
        p.fz_min << cfg_.support_force_min, 0.0, 0.0, cfg_.support_force_min;
        p.fz_max << 1.5 * mg, 0.0, 0.0, 1.5 * mg;
        p.fz_ref << 0.5 * mg, 0.0, 0.0, 0.5 * mg;
        p.support_line_mode = true;
        return p;
    }

    wq::ActuatorCommand control(const wq::ContactPlan& plan, double dt) {
        wq::Vec6 external = wq::Vec6::Zero();
        if (configured_) {
            const wq::SupportLineFrame frame = lqr_.frame(s_, *model_);
            if (frame.valid)
                external.head<3>() = lqr_.force(s_, *model_) * frame.lateral;
        }
        // No yaw/turn request is passed into the two-wheel controller.
        qp_->setTargetYawRate(0.0);
        qp_->setExternalWrench(external);
        return qp_->update(s_, plan, dt);
    }

    void addSwingImpedance(wq::ActuatorCommand& cmd,
                           double lift_fraction) const {
        const wq::SupportLineFrame frame = lqr_.frame(s_, *model_);
        if (!frame.valid) return;
        const double lift = cfg_.swing_clearance
                          * std::clamp(lift_fraction, 0.0, 1.0);
        for (int leg : kSwing) {
            const wq::Vec3 target = wheel_anchor_[leg] + lift * frame.normal;
            const wq::Vec3 velocity = model_->wheelCenterJacobian(leg)
                                    * model_->velocity();
            const wq::Vec3 force = cfg_.swing_kp
                                 * (target - model_->wheelCenterPosition(leg))
                                 - cfg_.swing_kd * velocity;
            const wq::Vec16 tau = model_->toActuated(
                model_->wheelCenterJacobian(leg).transpose() * force);
            cmd.tau_ff.segment<3>(3 * leg) += tau.segment<3>(3 * leg);
            // Deliberately preserve the QP joint impedance. In particular,
            // UNLOAD never calls this function, and LIFT never zeros kp/kd.
        }
    }

    void send(wq::ActuatorCommand cmd) {
        wq::limitCommandTorque(cmd, s_, qp_->legTorqueLimit(),
                               qp_->wheelTorqueLimit());
        last_wheel_torque_ = cmd.tau_ff.tail<4>();
        MatXf out = MatXf::Zero(16, 5);
        out.col(0) = wq::controllerToRobotOrder(cmd.kp).cast<float>();
        out.col(1) = wq::controllerToRobotOrder(cmd.q_des).cast<float>();
        out.col(2) = wq::controllerToRobotOrder(cmd.kd).cast<float>();
        out.col(3) = wq::controllerToRobotOrder(cmd.dq_des).cast<float>();
        out.col(4) = wq::controllerToRobotOrder(cmd.tau_ff).cast<float>();
        if (out.allFinite()) robot_->SetJointCommand(out);
        else hard_failure_ = true;
    }

    wq::SupportLineMetrics metrics() const {
        return lqr_.metrics(s_, *model_);
    }

    bool supportLoaded() const {
        return normal_force_[wq::FL] > cfg_.contact_threshold
            && normal_force_[wq::HR] > cfg_.contact_threshold;
    }

    bool swingUnloaded() const {
        const double threshold = std::max(cfg_.contact_threshold,
            cfg_.unload_force_fraction * s_.mass * wq::kGravity);
        return normal_force_[wq::FR] < threshold
            && normal_force_[wq::HL] < threshold;
    }

    double swingClearance() const {
        const wq::SupportLineFrame frame = lqr_.frame(s_, *model_);
        if (!frame.valid) return -std::numeric_limits<double>::infinity();
        return std::min(
            (model_->wheelCenterPosition(wq::FR) - wheel_anchor_[wq::FR])
                .dot(frame.normal),
            (model_->wheelCenterPosition(wq::HL) - wheel_anchor_[wq::HL])
                .dot(frame.normal));
    }

    bool supportLineStable() const {
        const wq::SupportLineMetrics e = metrics();
        return configured_ && e.valid && supportLoaded()
            && qp_->lastSolveOk()
            && std::isfinite(qp_->lastQpResidual())
            && qp_->lastQpResidual() <= cfg_.qp_residual_limit
            && std::abs(e.com_line_error) <= cfg_.line_error_limit
            && std::abs(e.angle) <= cfg_.line_angle_limit
            && std::abs(e.rate) <= cfg_.line_rate_limit
            && s_.omega_B.norm() <= cfg_.stable_omega_limit;
    }

    bool liftReady() const {
        return supportLineStable() && swingUnloaded()
            && qp_->lastQpResidual() <= cfg_.qp_residual_limit;
    }

    void updateQpHealth() {
        qp_fail_count_ = qp_->lastSolveOk() ? 0 : qp_fail_count_ + 1;
    }

    bool softFailure() const {
        const wq::SupportLineMetrics e = metrics();
        return !e.valid || std::abs(e.angle) > cfg_.soft_angle;
    }

    bool hardFailure() const {
        const wq::SupportLineMetrics e = metrics();
        return hard_failure_ || !state_valid_ || !configured_ || !e.valid
            || std::abs(e.angle) > cfg_.hard_angle
            || s_.omega_B.norm() > cfg_.hard_omega
            || qp_fail_count_ > 25
            || (qp_->lastSolveOk() && std::isfinite(qp_->lastQpResidual())
                && qp_->lastQpResidual() > cfg_.qp_residual_limit);
    }

    void startLog(const std::string& path) {
        if (cfg_.enable_log) log_.open(path);
    }

    void closeLog() { log_.close(); }

    void log(int phase, const wq::ActuatorCommand& cmd) {
        const wq::SupportLineMetrics e = metrics();
        log_.writeExtended(s_, cmd, qp_->lastForces(), normal_force_,
                           e.com_line_error, e.angle, e.rate,
                           qp_->lastQpResidual(), qp_->wrenchResidual(),
                           qp_->wrenchRank(), qp_->wrenchCondition(), phase,
                           0, qp_->lastSolveOk());
    }

    void reportFailure(const char* state_name,
                       double support_lost_time = 0.0) const {
        const wq::SupportLineMetrics e = metrics();
        std::cerr << "[" << state_name << "] safety stop: valid="
                  << state_valid_ << " command_valid=" << !hard_failure_
                  << " com_line=" << e.com_line_error
                  << " angle_deg=" << e.angle * 180.0 / M_PI
                  << " rate=" << e.rate
                  << " qp_fail_count=" << qp_fail_count_
                  << " qp_residual=" << qp_->lastQpResidual()
                  << " wheel_tau=" << last_wheel_torque_.transpose()
                  << " support_lost_ms=" << 1000.0 * support_lost_time
                  << " normal_force=" << normal_force_.transpose()
                  << std::endl;
    }

    const Config& config() const { return cfg_; }
    const wq::Vec4& normalForce() const { return normal_force_; }
    double massGravity() const { return s_.mass * wq::kGravity; }
    bool lqrStable() const { return configured_; }

private:
    std::shared_ptr<RobotInterface> robot_;
    Config cfg_;
    std::unique_ptr<wq::RobotModel> model_;
    std::unique_ptr<wq::BalanceQP> qp_;
    wq::RobotState s_;
    wq::Vec4 normal_force_ = wq::Vec4::Zero();
    wq::Vec12 nominal_q_ = wq::Vec12::Zero();
    std::array<wq::Vec3, 4> wheel_anchor_{};
    wq::SupportLineLqr lqr_;
    wq::CsvLogger log_;
    int qp_fail_count_ = 0;
    bool state_valid_ = false;
    bool configured_ = false;
    bool hard_failure_ = false;
    wq::Vec4 last_wheel_torque_ = wq::Vec4::Zero();
};

inline double safeDt(double now, double& previous) {
    double dt = now - previous;
    previous = now;
    return (dt > 0.0 && dt < 0.05) ? dt : 0.002;
}

} // namespace two_wheel_detail

class DiagonalTransitionState : public StateBase {
public:
    DiagonalTransitionState(const RobotType& robot_type, const std::string& name,
                            std::shared_ptr<ControllerData> data)
        : StateBase(robot_type, name, data), runtime_(ri_ptr_) {}

    void OnEnter() override {
        phase_ = two_wheel_detail::Phase::SHIFT_COM;
        phase_start_ = previous_time_ = ri_ptr_->GetInterfaceTimeStamp();
        stable_time_ = support_lost_time_ = 0.0;
        abort_ = hard_failure_ = failure_reported_ = false;
        if (!runtime_.read()) {
            hard_failure_ = true;
        } else {
            runtime_.begin(runtime_.fourPlan());
            runtime_.startLog(runtime_.config().transition_log);
        }
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::DiagonalTransitionMode);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    }

    void OnExit() override { runtime_.closeLog(); }

    void Run() override {
        const double now = ri_ptr_->GetInterfaceTimeStamp();
        const double dt = two_wheel_detail::safeDt(now, previous_time_);
        if (!runtime_.read()) {
            hard_failure_ = true;
            return;
        }

        if (uc_ptr_->GetUserCommand().target_mode
            == int(RobotMotionState::QPBalanceMode)) {
            abort_ = true;
        }

        const double elapsed = now - phase_start_;
        wq::ContactPlan plan = runtime_.fourPlan();
        double lift_fraction = 0.0;

        switch (phase_) {
            case two_wheel_detail::Phase::SHIFT_COM:
                if (runtime_.supportLineStable()) stable_time_ += dt;
                else stable_time_ = 0.0;
                if (elapsed >= runtime_.config().shift_time
                    && stable_time_ >= runtime_.config().stable_dwell) {
                    phase_ = two_wheel_detail::Phase::UNLOAD_FR_HL;
                    phase_start_ = now;
                    stable_time_ = 0.0;
                } else if (elapsed > runtime_.config().phase_timeout) {
                    abort_ = true;
                }
                break;

            case two_wheel_detail::Phase::UNLOAD_FR_HL: {
                const double progress = two_wheel_detail::smooth(
                    elapsed / std::max(runtime_.config().unload_time, 1e-3));
                plan = runtime_.unloadPlan(progress);
                const bool stable = progress > 0.85
                    && runtime_.supportLineStable()
                    && runtime_.swingUnloaded();
                stable_time_ = stable ? stable_time_ + dt : 0.0;
                if (stable_time_ >= runtime_.config().stable_dwell) {
                    // The lift gate is evaluated from measured contact force,
                    // support-line stability, and a successful QP cycle.
                    if (runtime_.liftReady()) {
                        phase_ = two_wheel_detail::Phase::LIFT_FR_HL;
                        phase_start_ = now;
                        stable_time_ = 0.0;
                    }
                } else if (elapsed > runtime_.config().phase_timeout) {
                    abort_ = true;
                }
                break;
            }

            case two_wheel_detail::Phase::LIFT_FR_HL: {
                const double progress = two_wheel_detail::smooth(
                    elapsed / std::max(runtime_.config().lift_time, 1e-3));
                plan = runtime_.twoPlan();
                lift_fraction = progress;
                const bool stable = progress >= 1.0
                    && runtime_.supportLineStable()
                    && runtime_.swingClearance() >= 0.04;
                stable_time_ = stable ? stable_time_ + dt : 0.0;
                if (stable_time_ >= runtime_.config().stable_dwell) {
                    phase_ = two_wheel_detail::Phase::TWO_WHEEL_BALANCE;
                    phase_start_ = now;
                    stable_time_ = 0.0;
                } else if (elapsed > runtime_.config().phase_timeout) {
                    hard_failure_ = true;
                }
                break;
            }

            case two_wheel_detail::Phase::TWO_WHEEL_BALANCE:
                plan = runtime_.twoPlan();
                lift_fraction = 1.0;
                break;
        }

        if (abort_) {
            // Abort before LIFT: keep all four contact constraints and return
            // to the ordinary QP state on the next FSM tick.
            phase_ = two_wheel_detail::Phase::SHIFT_COM;
            plan = runtime_.fourPlan();
            lift_fraction = 0.0;
        }

        auto cmd = runtime_.control(plan, dt);
        if (phase_ == two_wheel_detail::Phase::LIFT_FR_HL
            || phase_ == two_wheel_detail::Phase::TWO_WHEEL_BALANCE) {
            runtime_.addSwingImpedance(cmd, lift_fraction);
        }
        runtime_.send(cmd);
        runtime_.updateQpHealth();
        support_lost_time_ = runtime_.supportLoaded()
            ? 0.0 : support_lost_time_ + dt;
        runtime_.log(static_cast<int>(phase_), cmd);

        if (support_lost_time_ > runtime_.config().support_loss_timeout)
            hard_failure_ = true;
        if (runtime_.hardFailure()) hard_failure_ = true;
    }

    bool LoseControlJudge() override {
        if (hard_failure_ && !failure_reported_) {
            runtime_.reportFailure("DiagonalTransition", support_lost_time_);
            failure_reported_ = true;
        }
        return hard_failure_;
    }

    StateName GetNextStateName() override {
        if (abort_ && !hard_failure_) return StateName::kQPBalance;
        if (phase_ == two_wheel_detail::Phase::TWO_WHEEL_BALANCE)
            return StateName::kTwoWheelBalance;
        return StateName::kDiagonalTransition;
    }

private:
    two_wheel_detail::Runtime runtime_;
    two_wheel_detail::Phase phase_ = two_wheel_detail::Phase::SHIFT_COM;
    double phase_start_ = 0.0;
    double previous_time_ = 0.0;
    double stable_time_ = 0.0;
    double support_lost_time_ = 0.0;
    bool abort_ = false;
    bool hard_failure_ = false;
    bool failure_reported_ = false;
};

class TwoWheelBalanceState : public StateBase {
public:
    TwoWheelBalanceState(const RobotType& robot_type, const std::string& name,
                         std::shared_ptr<ControllerData> data)
        : StateBase(robot_type, name, data), runtime_(ri_ptr_) {}

    void OnEnter() override {
        previous_time_ = ri_ptr_->GetInterfaceTimeStamp();
        support_lost_time_ = 0.0;
        valid_ = runtime_.read();
        if (valid_) {
            runtime_.begin(runtime_.twoPlan());
            runtime_.startLog(runtime_.config().balance_log);
            valid_ = runtime_.lqrStable();
        }
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::TwoWheelBalanceMode);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    }

    void OnExit() override { runtime_.closeLog(); }

    void Run() override {
        const double now = ri_ptr_->GetInterfaceTimeStamp();
        const double dt = two_wheel_detail::safeDt(now, previous_time_);
        if (!runtime_.read()) {
            valid_ = false;
            return;
        }
        auto cmd = runtime_.control(runtime_.twoPlan(), dt);
        // Keep a nonzero swing task and the QP's joint impedance while holding
        // the lifted FR/HL legs clear of the ground.
        runtime_.addSwingImpedance(cmd, 1.0);
        runtime_.send(cmd);
        runtime_.updateQpHealth();
        support_lost_time_ = runtime_.supportLoaded()
            ? 0.0 : support_lost_time_ + dt;
        runtime_.log(static_cast<int>(two_wheel_detail::Phase::TWO_WHEEL_BALANCE),
                     cmd);
    }

    bool LoseControlJudge() override {
        const bool failed = !valid_ || runtime_.hardFailure()
                         || support_lost_time_ > runtime_.config().support_loss_timeout;
        if (failed && !failure_reported_) {
            runtime_.reportFailure("TwoWheelBalance", support_lost_time_);
            failure_reported_ = true;
        }
        return failed;
    }

    StateName GetNextStateName() override {
        const int target = uc_ptr_->GetUserCommand().target_mode;
        if (target == int(RobotMotionState::LieDown)) return StateName::kLieDown;
        if (target == int(RobotMotionState::QPBalanceMode))
            return StateName::kQPBalance;
        return StateName::kTwoWheelBalance;
    }

private:
    two_wheel_detail::Runtime runtime_;
    double previous_time_ = 0.0;
    double support_lost_time_ = 0.0;
    bool valid_ = false;
    bool failure_reported_ = false;
};
