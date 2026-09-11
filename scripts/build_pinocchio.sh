#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/third_party/pinocchio"
BUILD="$ROOT/third_party/_build/pinocchio"
PREFIX="$ROOT/third_party/install"
JOBS="${JOBS:-$(nproc)}"

# --- kiểm tra trước, báo lỗi rõ ràng ---
[ -f "$SRC/CMakeLists.txt" ] || { echo "!! Thieu source: $SRC"; exit 1; }
[ -f "$SRC/cmake/base.cmake" ] || {
  echo "!! Thieu submodule jrl-cmakemodules."
  echo "   Chay: git submodule update --init --recursive third_party/pinocchio"
  exit 1; }

cmake -B "$BUILD" -S "$SRC" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_INSTALL_LIBDIR=lib \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_WITH_LIBPYTHON=OFF \
  -DGENERATE_PYTHON_STUBS=OFF \
  \
  -DBUILD_TESTING=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_BENCHMARK=OFF \
  -DBUILD_UTILS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DINSTALL_DOCUMENTATION=OFF \
  \
  -DBUILD_WITH_URDF_SUPPORT=ON \
  -DBUILD_WITH_SDF_SUPPORT=OFF \
  -DBUILD_WITH_COLLISION_SUPPORT=OFF \
  -DBUILD_WITH_AUTODIFF_SUPPORT=OFF \
  -DBUILD_WITH_CASADI_SUPPORT=OFF \
  -DBUILD_WITH_CODEGEN_SUPPORT=OFF \
  -DBUILD_WITH_EXTRA_SUPPORT=OFF \
  -DBUILD_WITH_OPENMP_SUPPORT=OFF \
  -DBUILD_WITH_ACCELERATE_SUPPORT=OFF

echo "=== Rà soát option con BUILD_* còn ON ==="
cmake -LAH "$BUILD" 2>/dev/null | grep -E "^BUILD_[A-Z_]*:BOOL=ON" || true

cmake --build "$BUILD" -j"$JOBS"
cmake --install "$BUILD"

echo "=== XONG ==="
find "$PREFIX/lib" -maxdepth 1 -name 'libpinocchio*' -printf '%f  %s bytes\n'
echo "Config: $PREFIX/lib/cmake/pinocchio/"