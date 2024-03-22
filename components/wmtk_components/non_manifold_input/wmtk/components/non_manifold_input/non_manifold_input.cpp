#include "non_manifold_input.hpp"

#include <wmtk/components/input/input.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>

#include "NonManifoldInputOptions.hpp"

void wmtk::components::non_manifold_input(
    const base::Paths& paths,
    const nlohmann::json& j,
    io::Cache& cache)
{
    NonManifoldInputOptions options = j.get<NonManifoldInputOptions>();

    assert(options.tetrahedron_attributes
               .empty()); // tetrahedron attributes might be deleted in this component

    // call input component
    nlohmann::json input_j = j;
    input_j.erase("non_manifold_vertex_label");
    input_j.erase("non_manifold_edge_label");
    input_j.erase("non_manifold_tag_value");
    input(paths, j, cache);

    auto mesh_in = cache.read_mesh(options.name);

    auto& m = *mesh_in;

    if (m.top_simplex_type() == PrimitiveType::Vertex) {
        logger().warn("Unnecessary use of non-manifold input component. Point meshes cannot be "
                      "non-manifold.");
        return;
    }

    if (m.top_simplex_type() == PrimitiveType::Tetrahedron) {
        logger().warn("This component was never tested for tet meshes. It should work but proceed "
                      "with care (and probably add a unit test for tet meshes).");
    }

    //////////////////////////
    // find non-manifoldness

    auto vertex_top_cofaces =
        m.register_attribute<int64_t>("vertex_top_cofaces", PrimitiveType::Vertex, 1);
    auto edge_top_cofaces =
        m.register_attribute<int64_t>("edge_top_cofaces", PrimitiveType::Edge, 1);

    auto v_tc_acc = m.create_accessor<int64_t>(vertex_top_cofaces);
    auto e_tc_acc = m.create_accessor<int64_t>(edge_top_cofaces);

    for (const Tuple& t : m.get_all(m.top_simplex_type())) {
        const auto vertices = simplex::faces_single_dimension_tuples(
            m,
            simplex::Simplex(m.top_simplex_type(), t),
            PrimitiveType::Vertex);

        for (const Tuple& v : vertices) {
            v_tc_acc.scalar_attribute(v)++;
        }

        const auto edges = simplex::faces_single_dimension_tuples(
            m,
            simplex::Simplex(m.top_simplex_type(), t),
            PrimitiveType::Edge);

        for (const Tuple& e : edges) {
            e_tc_acc.scalar_attribute(e)++;
        }
    }

    auto non_manifold_vertex_tag =
        m.register_attribute<int64_t>(options.non_manifold_vertex_label, PrimitiveType::Vertex, 1);
    auto non_manifold_edge_tag =
        m.register_attribute<int64_t>(options.non_manifold_edge_label, PrimitiveType::Edge, 1);

    auto nmv_acc = m.create_accessor<int64_t>(non_manifold_vertex_tag);
    auto nme_acc = m.create_accessor<int64_t>(non_manifold_edge_tag);

    // check if the stored number of incident top simplices is the same as the one that can be
    // reached by navigation on the mesh

    size_t nmv_counter = 0; // number of non-manifold vertices
    for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
        const size_t n_top_cofaces =
            simplex::top_dimension_cofaces_tuples(m, simplex::Simplex::vertex(v)).size();

        if (n_top_cofaces != v_tc_acc.const_scalar_attribute(v)) {
            nmv_acc.scalar_attribute(v) = options.non_manifold_tag_value;
            nmv_counter++;
        }
    }

    size_t nme_counter = 0; // number of non-manifold edges
    for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
        const size_t n_top_cofaces =
            simplex::top_dimension_cofaces_tuples(m, simplex::Simplex::edge(e)).size();

        if (n_top_cofaces != e_tc_acc.const_scalar_attribute(e)) {
            nme_acc.scalar_attribute(e) = options.non_manifold_tag_value;
            nme_counter++;
        }
    }

    logger().info(
        "The mesh {} contains {} non-manifold vertices and {} non-manifold edges",
        options.name,
        nmv_counter,
        nme_counter);

    if (nmv_counter > 0 || nme_counter > 0) {
        logger().info("Updating mesh {} in cache.", options.name);

        auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

        m.clear_attributes({pos_handle, non_manifold_vertex_tag, non_manifold_edge_tag});

        cache.write_mesh(m, options.name);
    } else {
        logger().info(
            "Mesh {} does not contain any non-manfoldness. No tags are added.",
            options.name);
    }
}
