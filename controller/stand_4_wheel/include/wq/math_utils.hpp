#pragma once
#include "wq/types.hpp"
#include <cmath>
#include <algorithm>

namespace wq {

inline Mat3 skew(const Vec3& v) {
    Mat3 S;
    S <<     0.0, -v.z(),  v.y(),
          v.z(),    0.0, -v.x(),
         -v.y(),  v.x(),    0.0;
    return S;
}

inline Vec3 vee(const Mat3& S) { return Vec3(S(2, 1), S(0, 2), S(1, 0)); }

// log map SO(3) -- dung cho sai so huong.
// KHONG dung hieu Euler angle: hong khi pitch lon va co wrap-around.
inline Vec3 so3Log(const Mat3& R) {
    const double c  = std::clamp(0.5 * (R.trace() - 1.0), -1.0, 1.0);
    const double th = std::acos(c);
    if (th < 1e-7) return 0.5 * vee(R - R.transpose());
    if (th > M_PI - 1e-7) {             // gan 180 do: cong thuc tren mat on dinh
        Eigen::AngleAxisd aa(R);
        return aa.angle() * aa.axis();
    }
    return (th / (2.0 * std::sin(th))) * vee(R - R.transpose());
}

// Da thuc quintic: s(0)=0, s(1)=1, s'=s''=0 o hai dau.
// Lien tuc toi GIA TOC (cosine chi lien tuc toi van toc -> van giat momen).
inline void quintic(double xi, double& s, double& ds) {
    xi = std::clamp(xi, 0.0, 1.0);
    const double x2 = xi * xi, x3 = x2 * xi, x4 = x3 * xi, x5 = x4 * xi;
    s  = 10.0 * x3 - 15.0 * x4 + 6.0 * x5;
    ds = 30.0 * x2 - 60.0 * x3 + 30.0 * x4;
}

template <typename Derived>
inline Eigen::Matrix<double, Derived::RowsAtCompileTime, 1>
satVec(const Eigen::MatrixBase<Derived>& v, double lim) {
    return v.cwiseMax(-lim).cwiseMin(lim);
}

inline double clampd(double v, double lo, double hi) {
    return std::clamp(v, lo, hi);
}

// Noi suy huong theo slerp, tra ve ma tran xoay.
inline Mat3 slerpR(const Mat3& A, const Mat3& B, double a) {
    Quat qa(A), qb(B);
    if (qa.dot(qb) < 0.0) qb.coeffs() *= -1.0;   // chon duong ngan
    return qa.slerp(clampd(a, 0.0, 1.0), qb).normalized().toRotationMatrix();
}

} // namespace wq
