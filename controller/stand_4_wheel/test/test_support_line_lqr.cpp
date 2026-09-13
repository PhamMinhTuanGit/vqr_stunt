#include "wq/support_line_lqr.hpp"

#include <cmath>
#include <iostream>

int main() {
    wq::SupportLineLqr lqr;
    lqr.configure(32.86, 0.49, 0.002);
    if (!lqr.stable() || !std::isfinite(lqr.spectralRadius())
        || lqr.spectralRadius() >= 1.0 || !lqr.gain().allFinite()) {
        std::cerr << "unstable support-line LQR, spectral radius="
                  << lqr.spectralRadius() << '\n';
        return 1;
    }
    std::cout << "support-line LQR spectral radius=" << lqr.spectralRadius()
              << ", gain=" << lqr.gain() << '\n';
    return 0;
}
