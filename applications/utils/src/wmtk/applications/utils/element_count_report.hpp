#pragma once
#include <nlohmann/json.hpp>
#include <vector>

namespace wmtk {
    class Mesh;
}

namespace wmtk::applications::utils {

    std::vector<int64_t> element_count_report(const Mesh& m);
    nlohmann::json element_count_report_named(const Mesh& m);
}
