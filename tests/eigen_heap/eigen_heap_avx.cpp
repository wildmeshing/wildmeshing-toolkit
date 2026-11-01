// A unit of wmtk_test_eigen_heap, built with -mavx2 -mfma, as every unit is where the compiler
// targets x86-64-v3 (see tests/CMakeLists.txt; only with x86 compilers that take the flags).
#define WMTK_EIGEN_HEAP_UNIT avx_unit
#define WMTK_EIGEN_HEAP_UNIT_NAME "avx"
#include "eigen_heap_unit.inc"
