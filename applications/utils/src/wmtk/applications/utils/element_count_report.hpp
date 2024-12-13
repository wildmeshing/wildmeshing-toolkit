#pragma once
#include <nlohmann/json.hpp>
#include <vector>

namespace wmtk {
    class Mesh;
    namespace components::multimesh {
        class MeshCollection;
    }
}

namespace wmtk::applications::utils {


// Generates size statistics for each mesh in a mesh collection. Useful for "reports" used in integratino tests
// Something like:
// {"pos": {"vertices": 3, "edges": 24}, "pos.uv": {"vertices": 0, "edges": 23}}
    nlohmann::json element_count_report_named(const components::multimesh::MeshCollection& m);

// produces a json file corresponding colloquial simplex type names to the counts 
// Soemthing like: {"vertices": 3, "edges": 24}
    nlohmann::json element_count_report_named(const Mesh& m);


// produces the number of simplices of each type in a mesh - useful for generating json reports on the elements in a mesh
    std::vector<int64_t> element_count_report(const Mesh& m);
}
