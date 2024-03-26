#include "triangle_insertion.hpp"

#include "TriInsOptions.hpp"

#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/VolumeRemesherTriangleInsertion.hpp>


#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/Logger.hpp>


// this should change! make a util?
#include <wmtk/components/multimesh_from_tag/internal/MultiMeshFromTag.hpp>

namespace wmtk::components {

void triangle_insertion(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    TriInsOptions options = j.get<TriInsOptions>();

    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);
    std::shared_ptr<Mesh> bg_mesh = cache.read_mesh(options.background);

    if (mesh_in->top_simplex_type() != PrimitiveType::Triangle)
        log_and_throw_error("triangle_insertion supports only triangle meshes");
    if (bg_mesh->top_simplex_type() != PrimitiveType::Tetrahedron)
        log_and_throw_error("triangle_insertion supports only bg tet meshes");


    wmtk::utils::EigenMatrixWriter writer;
    mesh_in->serialize(writer);


    wmtk::utils::EigenMatrixWriter writer_bg;
    bg_mesh->serialize(writer_bg);

    Eigen::MatrixXd V;
    writer.get_double_matrix(options.input_position, PrimitiveType::Vertex, V);

    Eigen::MatrixX<int64_t> F;
    writer.get_FV_matrix(F);


    Eigen::MatrixXd Vbg;
    writer_bg.get_double_matrix(options.background_position, PrimitiveType::Vertex, Vbg);

    Eigen::MatrixX<int64_t> Fbg;
    writer_bg.get_TV_matrix(Fbg);


    /* new code for nonmanifold substructure and open boundaries*/

    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

    auto [tetmesh, tet_face_on_input_surface] =
        utils::generate_raw_tetmesh_from_input_surface(V, F, 0.1, Vbg, Fbg);

    /* -----------input surface--------- */

    auto surface_handle =
        tetmesh->register_attribute<int64_t>("surface", PrimitiveType::Triangle, 1);
    auto surface_accessor = tetmesh->create_accessor<int64_t>(surface_handle);

    // register surface tag

    const auto& tets = tetmesh->get_all(PrimitiveType::Tetrahedron);
    assert(tets.size() == tet_face_on_input_surface.size());

    for (int64_t i = 0; i < tets.size(); ++i) {
        const auto& t = tets[i]; // local face 2
        std::array<Tuple, 4> fs = {
            {tetmesh->switch_tuples(t, {PV, PE, PF}),
             tetmesh->switch_tuples(t, {PE, PF}),
             t,
             tetmesh->switch_tuples(t, {PF})}};

        for (int64_t k = 0; k < 4; ++k) {
            if (tet_face_on_input_surface[i][k]) {
                surface_accessor.scalar_attribute(fs[k]) = 1;
            } else {
                surface_accessor.scalar_attribute(fs[k]) = 0;
            }
        }
    }

    // get multimesh from tag
    internal::MultiMeshFromTag SurfaceMeshFromTag(*tetmesh, surface_handle, 1);
    SurfaceMeshFromTag.compute_substructure_mesh();

    std::shared_ptr<Mesh> surface_mesh = tetmesh->get_child_meshes().back();

    // SurfaceMeshFromTag.remove_soup();

    /* -----------open boundary and nonmanifold edges in input surface--------- */

    auto open_boundary_handle =
        tetmesh->register_attribute<int64_t>("open_boundary", PrimitiveType::Edge, 1);
    auto open_boundary_accessor = tetmesh->create_accessor<int64_t>(open_boundary_handle);

    auto nonmanifold_edge_handle =
        tetmesh->register_attribute<int64_t>("nonmanifold_edge", PrimitiveType::Edge, 1);
    auto nonmanifold_edge_accessor = tetmesh->create_accessor<int64_t>(nonmanifold_edge_handle);

    // register edge tags

    for (const auto& e : surface_mesh->get_all(PrimitiveType::Edge)) {
        const auto surface_edge = simplex::Simplex::edge(e);
        if (!surface_mesh->is_boundary(surface_edge)) continue;

        const auto& parent_e = surface_mesh->map_to_parent(surface_edge);
        const auto& child_e = tetmesh->map_to_child_tuples(*surface_mesh, parent_e);

        assert(child_e.size() > 0);

        if (child_e.size() == 1) {
            // if edge is a boundary on the surface mesh and only appears once,
            // then it is an open boundary
            open_boundary_accessor.scalar_attribute(parent_e.tuple()) = 1;
        } else {
            // else it is a nonmanifold edge in the input surface
            nonmanifold_edge_accessor.scalar_attribute(parent_e.tuple()) = 1;
        }
    }

    // get multimeshes from tag
    internal::MultiMeshFromTag OpenBoundaryFromTag(*tetmesh, open_boundary_handle, 1);
    OpenBoundaryFromTag.compute_substructure_mesh();

    std::shared_ptr<Mesh> open_boundary_mesh = tetmesh->get_child_meshes().back();

    // OpenBoundaryFromTag.remove_soup();

    internal::MultiMeshFromTag NonmanifoldEdgeFromTag(*tetmesh, nonmanifold_edge_handle, 1);
    NonmanifoldEdgeFromTag.compute_substructure_mesh();

    std::shared_ptr<Mesh> nonmanifold_edge_mesh = tetmesh->get_child_meshes().back();

    // NonmanifoldEdgeFromTag.remove_soup();

    /* -----------nonmanifold vertices in input surface--------- */

    auto nonmanifold_vertex_handle =
        tetmesh->register_attribute<int64_t>("nonmanifold_vertex", PrimitiveType::Vertex, 1);
    auto nonmanifold_vertex_accessor = tetmesh->create_accessor<int64_t>(nonmanifold_vertex_handle);

    for (const auto& v : tetmesh->get_all(PrimitiveType::Vertex)) {
        const auto& open_boundary_v =
            tetmesh->map_to_child(*open_boundary_mesh, simplex::Simplex::vertex(v));
        const auto& nonmanifold_edge_v =
            tetmesh->map_to_child(*nonmanifold_edge_mesh, simplex::Simplex::vertex(v));
        if (open_boundary_v.size() + nonmanifold_edge_v.size() > 1) {
            // if on the edgemeshes and more than 1 copy
            nonmanifold_vertex_accessor.scalar_attribute(v) = 1;
        } else if (
            // not on the edgemeshes and more than 1 copy on the surface mesh
            open_boundary_v.size() + nonmanifold_edge_v.size() == 0 &&
            tetmesh->map_to_child(*surface_mesh, simplex::Simplex::vertex(v)).size() > 1) {
            nonmanifold_vertex_accessor.scalar_attribute(v) = 1;
        }
    }

    // TODO: register as child mesh

    // remove all soups
    NonmanifoldEdgeFromTag.remove_soup();
    OpenBoundaryFromTag.remove_soup();
    SurfaceMeshFromTag.remove_soup();


    /* ------------------ post processing -------------------*/

    // propagate position to all child meshes
    auto pt_attribute =
        tetmesh->get_attribute_handle<double>(options.input_position, PrimitiveType::Vertex);

    for (auto child : tetmesh->get_child_meshes()) {
        auto child_position_handle = child->register_attribute<double>(
            options.input_position,
            PrimitiveType::Vertex,
            tetmesh->get_attribute_dimension(pt_attribute.as<double>()));

        auto propagate_to_child_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
            return P;
        };
        auto update_child_positon =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                child_position_handle,
                pt_attribute,
                propagate_to_child_position);
        update_child_positon->run_on_all();
    }

    // send to cache

    std::map<std::string, std::vector<int64_t>> names;

    names["tetmesh"] = tetmesh->absolute_multi_mesh_id();
    names["surface_mesh"] = surface_mesh->absolute_multi_mesh_id();
    names["open_boundary"] = open_boundary_mesh->absolute_multi_mesh_id();
    names["nonmanifold_edges"] = nonmanifold_edge_mesh->absolute_multi_mesh_id();

    cache.write_mesh(*tetmesh, options.name, names);

    /* -------------------- new code end ----------------------- */
    /* ----don't forget to comment out/ delete the old code----- */

    // auto [tetmesh, facemesh] =
    //     utils::generate_raw_tetmesh_with_surface_from_input(V, F, 0.1, Vbg, Fbg);

    // auto pt_attribute =
    //     tetmesh->get_attribute_handle<double>(options.input_position, PrimitiveType::Vertex);
    // auto child_position_handle = facemesh->register_attribute<double>(
    //     options.input_position,
    //     PrimitiveType::Vertex,
    //     tetmesh->get_attribute_dimension(pt_attribute.as<double>()));

    // auto propagate_to_child_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
    //     return P;
    // };
    // auto update_child_positon =
    //     std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
    //         child_position_handle,
    //         pt_attribute,
    //         propagate_to_child_position);
    // update_child_positon->run_on_all();

    // std::map<std::string, std::vector<int64_t>> names;

    // names["tetmesh"] = tetmesh->absolute_multi_mesh_id();
    // names["surface_mesh"] = facemesh->absolute_multi_mesh_id();

    // cache.write_mesh(*tetmesh, options.name, names);
}

} // namespace wmtk::components