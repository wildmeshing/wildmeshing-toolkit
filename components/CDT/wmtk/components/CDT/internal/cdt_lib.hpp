#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace cdt {

class inputPLC;
class TetMesh;
} // namespace cdt

namespace cdt_lib {

void cdt_to_string(
    const std::vector<double>& V,
    const uint32_t npts,
    const std::vector<uint32_t>& F,
    const uint32_t ntri,
    std::vector<std::array<bool, 4>>& local_f_on_input,
    std::vector<std::array<int64_t, 4>>& T_final,
    std::vector<std::array<std::string, 3>>& V_final,
    bool inner_only);


} // namespace cdt_lib