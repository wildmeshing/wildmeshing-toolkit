#include "triangle_insertion.hpp"


#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/VolumeRemesherTriangleInsertion.hpp>

#include <wmtk/invariants/SimplexInversionInvariant.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/operations/Rounding.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>


// this should change! make a util?
#include <wmtk/components/multimesh_from_tag/internal/MultiMeshFromTag.hpp>


namespace wmtk::components {

std::vector<std::pair<std::shared_ptr<wmtk::Mesh>, std::string>> triangle_insertion(
    const std::shared_ptr<Mesh>& mesh_in,
    const std::string& in_position,
    const std::shared_ptr<Mesh>& bg_mesh,
    const std::string& bg_position,
    std::vector<attribute::MeshAttributeHandle>& pass_through,
    bool make_child_free)
{
    if (mesh_in->top_simplex_type() != PrimitiveType::Triangle)
        log_and_throw_error("triangle_insertion supports only triangle meshes");
    if (bg_mesh->top_simplex_type() != PrimitiveType::Tetrahedron)
        log_and_throw_error("triangle_insertion supports only bg tet meshes");


    wmtk::utils::EigenMatrixWriter writer;
    mesh_in->serialize(writer);


    wmtk::utils::EigenMatrixWriter writer_bg;
    bg_mesh->serialize(writer_bg);

    Eigen::MatrixXd V;
    writer.get_double_matrix(in_position, PrimitiveType::Vertex, V);

    Eigen::MatrixX<int64_t> F;
    writer.get_FV_matrix(F);


    Eigen::MatrixXd Vbg;
    writer_bg.get_double_matrix(bg_position, PrimitiveType::Vertex, Vbg);

    Eigen::MatrixX<int64_t> Fbg;
    writer_bg.get_TV_matrix(Fbg);


    /* new code for nonmanifold substructure and open boundaries*/

    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

    auto [tetmesh, tet_face_on_input_surface] =
        utils::generate_raw_tetmesh_from_input_surface(V, F, 0.1, Vbg, Fbg);

    std::vector<std::pair<std::shared_ptr<wmtk::Mesh>, std::string>> all_meshes;
    all_meshes.push_back(std::make_pair(tetmesh, "main"));


    /* -------------rounding------------ */


    auto rounding_pt_attribute =
        tetmesh->get_attribute_handle_typed<Rational>(in_position, PrimitiveType::Vertex);

    std::shared_ptr<Mesh> m_ptr = tetmesh;

    auto rounding = std::make_shared<wmtk::operations::Rounding>(*m_ptr, rounding_pt_attribute);
    rounding->add_invariant(
        std::make_shared<SimplexInversionInvariant<Rational>>(*m_ptr, rounding_pt_attribute));

    Scheduler scheduler;
    auto stats = scheduler.run_operation_on_all(*rounding);

    logger().info(
        "Executed rounding, {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, "
        "executing: {}",
        stats.number_of_performed_operations(),
        stats.number_of_successful_operations(),
        stats.number_of_failed_operations(),
        stats.collecting_time,
        stats.sorting_time,
        stats.executing_time);


    /* -----------input surface--------- */

    logger().error("Registering input surface from tag surface");
    auto surface_handle =
        tetmesh->register_attribute<int64_t>("surface", PrimitiveType::Triangle, 1);
    auto surface_accessor = tetmesh->create_accessor<int64_t>(surface_handle);

    // register surface tag

    const auto& tets = tetmesh->get_all(PrimitiveType::Tetrahedron);
    assert(tets.size() == tet_face_on_input_surface.size());

    logger().error("Assigning surface tags");
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
    std::shared_ptr<Mesh> surface_mesh;

    if (make_child_free) {
        logger().error("Making free child surface mesh");
        surface_mesh = wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag_handle(
            *tetmesh,
            surface_handle.as<int64_t>(),
            1,
            /*free = */ true);

    } else {
        logger().error("Making child surface mesh");
        internal::MultiMeshFromTag SurfaceMeshFromTag(*tetmesh, surface_handle, 1);
        SurfaceMeshFromTag.compute_substructure_mesh();

        surface_mesh = tetmesh->get_child_meshes().back();

        SurfaceMeshFromTag.remove_soup();
    }

    all_meshes.push_back(std::make_pair(surface_mesh, "surface"));
    pass_through.push_back(surface_handle);

    /* -----------open boundary and nonmanifold edges in input surface--------- */

    logger().error("Going through open/nonmanifold stuff");
    auto open_boundary_handle =
        tetmesh->register_attribute<int64_t>("open_boundary", PrimitiveType::Edge, 1);
    auto open_boundary_accessor = tetmesh->create_accessor<int64_t>(open_boundary_handle);

    auto nonmanifold_edge_handle =
        tetmesh->register_attribute<int64_t>("nonmanifold_edge", PrimitiveType::Edge, 1);
    auto nonmanifold_edge_accessor = tetmesh->create_accessor<int64_t>(nonmanifold_edge_handle);

    // register edge tags
    bool has_openboundary = false;
    bool has_nonmanifold_edge = false;

    logger().error("Looping edges for open/nonmanifold ones");
    for (const auto& e : surface_mesh->get_all(PrimitiveType::Edge)) {
        const auto surface_edge = simplex::Simplex::edge(*surface_mesh, e);
        if (!surface_mesh->is_boundary(surface_edge)) continue;

        const auto& parent_e = surface_mesh->map_to_parent(surface_edge);
        const auto& child_e = tetmesh->map_to_child_tuples(*surface_mesh, parent_e);

        assert(child_e.size() > 0);

        if (child_e.size() == 1) {
            // if edge is a boundary on the surface mesh and only appears once,
            // then it is an open boundary
            open_boundary_accessor.scalar_attribute(parent_e.tuple()) = 1;
            has_openboundary = true;
        } else {
            // else it is a nonmanifold edge in the input surface
            nonmanifold_edge_accessor.scalar_attribute(parent_e.tuple()) = 1;
            has_nonmanifold_edge = true;
        }
    }

    const bool process_nonmanifold_edges = !make_child_free && has_nonmanifold_edge;

    // get multimeshes from tag

    std::shared_ptr<Mesh> open_boundary_mesh, nonmanifold_edge_mesh;

    if (has_openboundary) {
        if (make_child_free) {
            logger().error("Creating free open boundary child mesh");
            open_boundary_mesh =
                wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag_handle(
                    *tetmesh,
                    open_boundary_handle.as<int64_t>(),
                    1,
                    /*free = */ true);

        } else {
            logger().error("Creating open boundary child mesh");
            internal::MultiMeshFromTag OpenBoundaryFromTag(*tetmesh, open_boundary_handle, 1);
            OpenBoundaryFromTag.compute_substructure_mesh();

            open_boundary_mesh = tetmesh->get_child_meshes().back();
            OpenBoundaryFromTag.remove_soup();
        }

        all_meshes.push_back(std::make_pair(open_boundary_mesh, "open_boundary"));
        pass_through.push_back(open_boundary_handle);
    }

    if (process_nonmanifold_edges) {
        logger().error("Creating nonmanifold edge mesh");
        internal::MultiMeshFromTag NonmanifoldEdgeFromTag(*tetmesh, nonmanifold_edge_handle, 1);
        NonmanifoldEdgeFromTag.compute_substructure_mesh();

        nonmanifold_edge_mesh = tetmesh->get_child_meshes().back();
        NonmanifoldEdgeFromTag.remove_soup();

        all_meshes.push_back(std::make_pair(nonmanifold_edge_mesh, "nonmanifold_edges"));
        pass_through.push_back(nonmanifold_edge_handle);
    }

    /* ---------------------bounding box-------------------------*/
    auto bbox_handle = tetmesh->register_attribute<int64_t>("bbox", PrimitiveType::Triangle, 1);
    auto bbox_accessor = tetmesh->create_accessor<int64_t>(bbox_handle);

    logger().error("Annotating bounding box boundary");
    for (const auto& f : tetmesh->get_all(PrimitiveType::Triangle)) {
        bbox_accessor.scalar_attribute(f) =
            tetmesh->is_boundary(PrimitiveType::Triangle, f) ? 1 : 0;
    }

    std::shared_ptr<Mesh> bbox_mesh;

    internal::MultiMeshFromTag BboxFromTag(*tetmesh, bbox_handle, 1);
    BboxFromTag.compute_substructure_mesh();

    bbox_mesh = tetmesh->get_child_meshes().back();
    BboxFromTag.remove_soup();

    all_meshes.push_back(std::make_pair(bbox_mesh, "bounding_box"));
    pass_through.push_back(bbox_handle);

    /* -----------nonmanifold vertices in input surface--------- */

    logger().error("Nonmanifold vertices");
    auto nonmanifold_vertex_handle =
        tetmesh->register_attribute<int64_t>("nonmanifold_vertex", PrimitiveType::Vertex, 1);
    auto nonmanifold_vertex_accessor = tetmesh->create_accessor<int64_t>(nonmanifold_vertex_handle);

    for (const auto& v : tetmesh->get_all(PrimitiveType::Vertex)) {
        int64_t on_open_boundary_cnt = 0;
        int64_t on_nonmanifold_edge_cnt = 0;

        if (has_openboundary) {
            on_open_boundary_cnt =
                tetmesh->map_to_child(*open_boundary_mesh, simplex::Simplex::vertex(*tetmesh, v))
                    .size();
        }
        if (process_nonmanifold_edges) {
            on_nonmanifold_edge_cnt =
                tetmesh->map_to_child(*nonmanifold_edge_mesh, simplex::Simplex::vertex(*tetmesh, v))
                    .size();
        }

        if (on_open_boundary_cnt + on_open_boundary_cnt > 1) {
            // if on the edgemeshes and more than 1 copy
            nonmanifold_vertex_accessor.scalar_attribute(v) = 1;
        } else if (
            // not on the edgemeshes and more than 1 copy on the surface mesh
            on_open_boundary_cnt + on_nonmanifold_edge_cnt == 0 &&
            tetmesh->map_to_child(*surface_mesh, simplex::Simplex::vertex(*tetmesh, v)).size() >
                1) {
            nonmanifold_vertex_accessor.scalar_attribute(v) = 1;
        }
    }

    pass_through.push_back(nonmanifold_vertex_handle);

    /* ------------------ post processing -------------------*/

    logger().error("Propagating position to child meshes");
    // propagate position to all child meshes
    auto pt_attribute = tetmesh->get_attribute_handle<Rational>(in_position, PrimitiveType::Vertex);

    for (auto child : tetmesh->get_child_meshes()) {
        auto child_position_handle = child->register_attribute<Rational>(
            in_position,
            PrimitiveType::Vertex,
            tetmesh->get_attribute_dimension(pt_attribute.as<Rational>()));

        auto propagate_to_child_position =
            [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorX<Rational> { return P; };
        auto update_child_positon =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<Rational, Rational>>(
                child_position_handle,
                pt_attribute,
                propagate_to_child_position);
        update_child_positon->run_on_all();
    }

    logger().info(
        "TetMesh created: {} tets, {} faces, {} edges, {} vertices",
        tetmesh->capacity(PrimitiveType::Tetrahedron),
        tetmesh->capacity(PrimitiveType::Triangle),
        tetmesh->capacity(PrimitiveType::Edge),
        tetmesh->capacity(PrimitiveType::Vertex));

    logger().info("Surface child TriMesh registered");

    if (has_openboundary) {
        logger().info("Open boundary child EdgeMesh registered");
    }
    if (process_nonmanifold_edges) {
        logger().info("Nonmanifold edge child EdgeMesh registered");
    }
    logger().info("Bbox child TriMesh registered");

    return all_meshes;

    /* -------------------- new code end ----------------------- */
    /* ----don't forget to comment out/ delete the old code----- */

    // auto [tetmesh, facemesh] =
    //     utils::generate_raw_tetmesh_with_surface_from_input(V, F, 0.1, Vbg, Fbg);

    // auto pt_attribute =
    //     tetmesh->get_attribute_handle<double>(in_position, PrimitiveType::Vertex);
    // auto child_position_handle = facemesh->register_attribute<double>(
    //     in_position,
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
}

} // namespace wmtk::components
