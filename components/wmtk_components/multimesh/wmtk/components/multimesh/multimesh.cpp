#include "multimesh.hpp"

#include "MultimeshOptions.hpp"


namespace wmtk::components {
void multimesh(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    MultimeshOptions options = j.get<MultimeshOptions>();

    std::shared_ptr<Mesh> parent = cache.read_mesh(options.parent);
    std::shared_ptr<Mesh> child = cache.read_mesh(options.child);

    if (parent->top_simplex_type() == child->top_simplex_type() &&
        parent->capacity(parent->top_simplex_type()) ==
            child->capacity(child->top_simplex_type())) {
        std::vector<int64_t> parent_simplices(parent->capacity(parent->top_simplex_type()));
        std::iota(parent_simplices.begin(), parent_simplices.end(), 0);

        auto child_map =
            multimesh::same_simplex_dimension_surjection(*parent, *child, parent_simplices);

        parent->register_child_mesh(child, child_map);
    } else {
        throw std::runtime_error("unsupported multimesh mapping");
    }


    // output
    cache.write_mesh(*parent, options.name);
}

} // namespace wmtk::components