#pragma once
#include "wq/types.hpp"
#include <cstdio>
#include <string>

namespace wq {

// CSV logger toi gian. Ha tang nay ban se can o MOI moc sau,
// nen lam ngay tu moc 1 (re khi robot dang dung yen, rat dat khi phai debug
// giua luc no dang xoay tren 2 banh).
class CsvLogger {
public:
    ~CsvLogger() { close(); }

    bool open(const std::string& path) {
        close();
        f_ = std::fopen(path.c_str(), "w");
        if (!f_) return false;
        std::fprintf(f_, "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,gbz");
        for (int i = 0; i < kLegJoints; ++i) std::fprintf(f_, ",q%d", i);
        for (int i = 0; i < kLegJoints; ++i) std::fprintf(f_, ",dq%d", i);
        for (int i = 0; i < kNumLegs; ++i)   std::fprintf(f_, ",dqw%d", i);
        for (int i = 0; i < kNumAct; ++i)    std::fprintf(f_, ",tau%d", i);
        for (int i = 0; i < 3 * kNumLegs; ++i) std::fprintf(f_, ",f%d", i);
        std::fprintf(f_, ",mode,qp_ok\n");
        return true;
    }

    void write(const RobotState& s, const ActuatorCommand& c,
               const Vec12& f, int mode, bool qp_ok) {
        if (!f_) return;
        std::fprintf(f_, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                     s.t, s.p_WB.x(), s.p_WB.y(), s.p_WB.z(),
                     s.v_WB.x(), s.v_WB.y(), s.v_WB.z());
        std::fprintf(f_, ",%.5f,%.5f,%.5f,%.5f",
                     s.q_WB.w(), s.q_WB.x(), s.q_WB.y(), s.q_WB.z());
        std::fprintf(f_, ",%.4f,%.4f,%.4f,%.4f",
                     s.omega_B.x(), s.omega_B.y(), s.omega_B.z(), s.g_B.z());
        for (int i = 0; i < kLegJoints; ++i) std::fprintf(f_, ",%.5f", s.q[i]);
        for (int i = 0; i < kLegJoints; ++i) std::fprintf(f_, ",%.5f", s.dq[i]);
        for (int i = 0; i < kNumLegs; ++i)   std::fprintf(f_, ",%.5f", s.dq_w[i]);
        for (int i = 0; i < kNumAct; ++i)    std::fprintf(f_, ",%.4f", c.tau_ff[i]);
        for (int i = 0; i < 3 * kNumLegs; ++i) std::fprintf(f_, ",%.3f", f[i]);
        std::fprintf(f_, ",%d,%d\n", mode, qp_ok ? 1 : 0);
    }

    void close() {
        if (f_) { std::fclose(f_); f_ = nullptr; }
    }

private:
    std::FILE* f_ = nullptr;
};

} // namespace wq
