#pragma once
#include "wq/types.hpp"
#include "eiquadprog.hpp"
#include <cmath>

namespace wq {

class QpSolver {
public:
    QpSolver(int n_var, int n_ineq) : n_(n_var), m_(n_ineq) {
        CE_.resize(n_var, 0);
        ce0_.resize(0);
        x_.setZero(n_var);
        CI_t_.resize(n_var, n_ineq);
        G_work_.resize(n_var, n_var);
        g0_work_.resize(n_var);
    }

    bool solve(const MatXd& G, const VecXd& g0,
               const MatXd& CI, const VecXd& ci0, VecXd& x) {
        G_work_ = G;
        g0_work_ = g0;
        CI_t_ = CI.transpose(); // convert (m x n) to (n x m) column-wise constraints
        double cost = Eigen::solve_quadprog(G_work_, g0_work_, CE_, ce0_, CI_t_, ci0, x_);
        if (std::isinf(cost) || std::isnan(cost)) return false;
        if (!x_.allFinite()) return false;
        x = x_;
        return true;
    }

    int numVar()  const { return n_; }
    int numIneq() const { return m_; }

private:
    int n_, m_;
    MatXd CE_;
    VecXd ce0_, x_;
    MatXd CI_t_, G_work_;
    VecXd g0_work_;
};

} // namespace wq
