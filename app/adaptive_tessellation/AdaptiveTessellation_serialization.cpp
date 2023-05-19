#include <igl/read_triangle_mesh.h>
#include <igl/remove_unreferenced.h>
#include <igl/writeDMAT.h>
#include <igl/write_triangle_mesh.h>
#include <lagrange/io/load_mesh.h>
#include <lean_vtk.hpp>
#include "AdaptiveTessellation.h"

using namespace adaptive_tessellation;
using namespace wmtk;

void AdaptiveTessellation::create_paired_seam_mesh_with_offset(
    const std::filesystem::path input_mesh_path,
    Eigen::MatrixXd& UV,
    Eigen::MatrixXi& F)
{
    throw std::runtime_error("Archived function. Please use mesh_preprocessing instead.");
    // load uv coordinates and connectivities
    auto mesh = lagrange::io::load_mesh<lagrange::SurfaceMesh32d>(input_mesh_path);
    triangulate_polygonal_facets(mesh);
    auto& uv_attr = mesh.get_indexed_attribute<double>(lagrange::AttributeName::texcoord);
    UV = lagrange::matrix_view(uv_attr.values());
    F = lagrange::reshaped_view(uv_attr.indices(), 3).cast<int>();
    assert(mesh.is_triangle_mesh());
    assert(mesh.get_num_facets() == F.rows());

    // load 3d coordinates and connectivities for computing the offset and scaling
    Eigen::MatrixXd CN;
    Eigen::MatrixXd FN;
    // igl::read_triangle_mesh(input_mesh_path.string(), input_V_, input_F_);
    igl::readOBJ(input_mesh_path.string(), input_V_, input_VT_, CN, input_F_, input_FT_, FN);

    const Eigen::MatrixXd box_min = input_V_.colwise().minCoeff();
    const Eigen::MatrixXd box_max = input_V_.colwise().maxCoeff();
    double max_comp = (box_max - box_min).maxCoeff();
    Eigen::MatrixXd scene_offset = -box_min;
    Eigen::MatrixXd scene_extent = box_max - box_min;
    scene_offset.array() -= (scene_extent.array() - max_comp) * 0.5;
    mesh_parameters.m_scale = max_comp;
    mesh_parameters.m_offset = scene_offset;

    wmtk::logger().info("///// #v : {} {}", UV.rows(), UV.cols());
    wmtk::logger().info("///// #f : {} {}", F.rows(), F.cols());

    wmtk::TriMesh m_3d;
    std::vector<std::array<size_t, 3>> tris;
    for (auto f = 0; f < input_F_.rows(); f++) {
        std::array<size_t, 3> tri = {(size_t)input_F_(f, 0),
                                     (size_t)input_F_(f, 1),
                                     (size_t)input_F_(f, 2)};
        tris.emplace_back(tri);
    }
    m_3d.create_mesh(input_V_.rows(), tris);
    create_mesh(UV, F);
    // loop through faces
    // for each edge, get fid on both side.
    // if it's boundary edge, do nothing
    // if not, index to the matrix for F_3d, and check the 3 edges
    // populate E0 and E1 for boundary construction
    std::map<std::pair<size_t, size_t>, std::pair<size_t, size_t>> seam_edges;
    Eigen::MatrixXi E0(F.rows() * 3, 2), E1(F.rows() * 3, 2);
    int seam_edge_cnt = 0;
    // for each vertex that's seam vertex, assign same color for mirror vertices
    // build the mapping from uv_index to color
    // and mapping from color to uv_index
    color_to_uv_indices.reserve(UV.rows());
    assert(uv_index_to_color.empty());
    int current_color = 0;
    for (auto fi = 0; fi < m_3d.tri_capacity(); ++fi) {
        for (auto lvi1 = 0; lvi1 < 3; ++lvi1) {
            auto lvi2 = (lvi1 + 1) % 3;
            auto local_eid = 3 - lvi1 - lvi2;
            // construct the edge tuple of current vertex in 3d mesh
            Tuple edge1_3d = Tuple(input_F_(fi, lvi1), local_eid, fi, m_3d);
            assert(input_F_(fi, lvi1) == edge1_3d.vid(m_3d));
            if (!edge1_3d.switch_face(m_3d).has_value()) {
                // Boundary edge, skipping...
                continue;
            } else {
                auto edge2_3d = edge1_3d.switch_face(m_3d).value();
                auto fj = edge2_3d.fid(m_3d);
                size_t lvj1, lvj2;
                for (auto i = 0; i < 3; i++) {
                    if (input_F_(fj, i) == edge1_3d.vid(m_3d)) lvj1 = i;
                    if (input_F_(fj, i) == edge1_3d.switch_vertex(m_3d).vid(m_3d)) lvj2 = i;
                }

                assert(input_F_(fi, lvi1) == input_F_(fj, lvj1));
                assert(input_F_(fi, lvi2) == input_F_(fj, lvj2));
                // set up seam edge
                if ((F(fi, lvi1) != F(fj, lvj1)) || (F(fi, lvi2) != F(fj, lvj2))) {
                    // this is a seam. init the mirror_edge tuple
                    // the orientation of the mirror edges is inccw (half edge conventions)
                    // However, the edge tuple in operations have arbitraty orientation
                    // !!! need to check orientations of mirror edge in operations !!!!!
                    TriMesh::Tuple seam_edge_fj(F(fj, lvj2), (3 - lvj1 - lvj2), fj, *this);
                    if (!seam_edge_fj.is_ccw(*this))
                        seam_edge_fj = seam_edge_fj.switch_vertex(*this);
                    face_attrs[fi].mirror_edges[local_eid] =
                        std::make_optional<wmtk::TriMesh::Tuple>(seam_edge_fj);

                    TriMesh::Tuple seam_edge_fi(F(fi, lvi1), local_eid, fi, *this);
                    if (!seam_edge_fi.is_ccw(*this))
                        seam_edge_fi = seam_edge_fi.switch_vertex(*this);
                    face_attrs[fj].mirror_edges[(3 - lvj1 - lvj2)] =
                        std::make_optional<wmtk::TriMesh::Tuple>(seam_edge_fi);
                    // check if seam_edge_fj is already in the map
                    size_t ei0 = std::min(F(fi, lvi1), F(fi, lvi2));
                    size_t ei1 = std::max(F(fi, lvi1), F(fi, lvi2));
                    size_t ej0 = std::min(F(fj, lvj1), F(fj, lvj2));
                    size_t ej1 = std::max(F(fj, lvj1), F(fj, lvj2));
                    if (seam_edges.find(std::pair{ei0, ei1}) != seam_edges.end()) {
                        assert((seam_edges[{ei0, ei1}] == std::pair<size_t, size_t>(ej0, ej1)));
                    } else {
                        // if not add it to the map and add to the edge matrix
                        seam_edges[{ej0, ej1}] = std::pair<size_t, size_t>(ei0, ei1);
                        E0.row(seam_edge_cnt) << F(fi, lvi1), F(fi, lvi2);
                        E1.row(seam_edge_cnt) << F(fj, lvj1), F(fj, lvj2);
                        seam_edge_cnt++;
                    }
                }
            }
            // set up fixed t-junction vertices
            // set up t-junction vertex coloring
            // get the one ring edges of this tuple
            if (uv_index_to_color.find(F(fi, lvi1)) != uv_index_to_color.end()) {
                // already colored, skipping...
                continue;
            } else {
                auto edge2_3d = edge1_3d.switch_face(m_3d).value();
                auto fj_3d = edge2_3d.fid(m_3d);
                size_t lvj1_3d, lvj2_3d;
                for (auto i = 0; i < 3; i++) {
                    if (input_F_(fj_3d, i) == edge1_3d.vid(m_3d)) lvj1_3d = i;
                    if (input_F_(fj_3d, i) == edge1_3d.switch_vertex(m_3d).vid(m_3d)) lvj2_3d = i;
                }

                assert(input_F_(fi, lvi1) == input_F_(fj_3d, lvj1_3d));
                assert(input_F_(fi, lvi2) == input_F_(fj_3d, lvj2_3d));
                // edge1_3d is a seam edge
                if ((F(fi, lvi1) != F(fj_3d, lvj1_3d)) || (F(fi, lvi2) != F(fj_3d, lvj2_3d))) {
                    uv_index_to_color.insert({F(fi, lvi1), current_color});
                    if (current_color < color_to_uv_indices.size() &&
                        std::find(
                            color_to_uv_indices[current_color].begin(),
                            color_to_uv_indices[current_color].end(),
                            F(fi, lvi1)) == color_to_uv_indices[current_color].end())
                        color_to_uv_indices[current_color].emplace_back(F(fi, lvi1));
                    else
                        color_to_uv_indices.emplace_back(1, F(fi, lvi1));
                    for (auto& e_3d : m_3d.get_one_ring_edges_for_vertex(edge1_3d)) {
                        if (!e_3d.switch_face(m_3d).has_value()) {
                            // Boundary edge, skipping...
                            continue;
                        } else {
                            auto e2_3d = e_3d.switch_face(m_3d).value();
                            auto fj = e2_3d.fid(m_3d);
                            size_t lvj1, lvj2;
                            for (auto i = 0; i < 3; i++) {
                                if (input_F_(fj, i) == edge1_3d.vid(m_3d)) lvj1 = i;
                                if (input_F_(fj, i) == e2_3d.vid(m_3d)) lvj2 = i;
                            }
                            assert(input_F_(fi, lvi1) == input_F_(fj, lvj1));
                            // this is a mirror vertex at a seam edge
                            if (F(fi, lvi1) != F(fj, lvj1)) {
                                // the mirror vertex has not been colored yet
                                if (uv_index_to_color.find(F(fj, lvj1)) ==
                                    uv_index_to_color.end()) {
                                    // add the color or the primary vertex to the mirror
                                    // vertex
                                    uv_index_to_color.insert(
                                        {F(fj, lvj1), uv_index_to_color[F(fi, lvi1)]});
                                }
                                // if the mirror vertex is not in the color busket, add it
                                if (std::find(
                                        color_to_uv_indices[uv_index_to_color[F(fi, lvi1)]].begin(),
                                        color_to_uv_indices[uv_index_to_color[F(fi, lvi1)]].end(),
                                        F(fj, lvj1)) ==
                                    color_to_uv_indices[uv_index_to_color[F(fi, lvi1)]].end())
                                    color_to_uv_indices[uv_index_to_color[F(fi, lvi1)]]
                                        .emplace_back(F(fj, lvj1));
                                assert(
                                    uv_index_to_color[F(fj, lvj1)] ==
                                    uv_index_to_color[F(fi, lvi1)]);
                            }
                        }
                    }
                    current_color++;
                }
            }
        }
    }
    color_to_uv_indices.resize(current_color);
    E0.conservativeResize(seam_edge_cnt, 2);
    E1.conservativeResize(seam_edge_cnt, 2);
    assert(E0.rows() == E1.rows());
    mesh_construct_boundaries(UV, F, E0, E1);
}

void AdaptiveTessellation::create_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    std::vector<Eigen::Vector3d> V_env;
    V_env.resize(V.rows());
    std::vector<Eigen::Vector3i> F_env;
    F_env.resize(F.rows());
    // Register attributes
    p_vertex_attrs = &vertex_attrs;
    p_face_attrs = &face_attrs;
    p_edge_attrs = &edge_attrs;
    // Convert from eigen to internal representation (TODO: move to utils and remove it from all
    // app)
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < F.rows(); i++) {
        F_env[i] << (size_t)F(i, 0), (size_t)F(i, 1), (size_t)F(i, 2);
        tri[i][0] = (size_t)F(i, 0);
        tri[i][1] = (size_t)F(i, 1);
        tri[i][2] = (size_t)F(i, 2);
    }
    // Initialize the trimesh class which handles connectivity
    wmtk::TriMesh::create_mesh(V.rows(), tri);
    // Save the vertex position in the vertex attributes
    for (unsigned i = 0; i < V.rows(); ++i) {
        vertex_attrs[i].pos << V.row(i)[0], V.row(i)[1];
        V_env[i] << V.row(i)[0], V.row(i)[1], 0.0;
    }
    for (const auto& tri : this->get_faces()) {
        assert(!is_inverted(tri));
    }
}

// set fixed vertices due to boundary
// set the curve-id for each edge
void AdaptiveTessellation::mesh_construct_boundaries(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXi& E0,
    const Eigen::MatrixXi& E1)
{
    // construct the boundary map for boundary parametrization
    if (mesh_parameters.m_boundary_parameter)
        mesh_parameters.m_boundary.construct_boundaries(V, F, E0, E1);
    // mark boundary vertices as boundary_vertex
    // but this is not indiscriminatively rejected for all operations
    // other operations are conditioned on whether m_bnd_freeze is turned on
    // also obtain the boudnary parametrizatin t for each vertex
    // for now keep the per vertex curve-id. but this is now a edge property
    for (auto v : this->get_vertices()) {
        if (is_boundary_vertex(v)) {
            vertex_attrs[v.vid(*this)].boundary_vertex = is_boundary_vertex(v);
            set_feature(v);
            // one vertex can have more than one curve-id.
            // current curve-id ofr vertex is arbitrarily picked among them
            std::tie(vertex_attrs[v.vid(*this)].curve_id, vertex_attrs[v.vid(*this)].t) =
                mesh_parameters.m_boundary.uv_to_t(vertex_attrs[v.vid(*this)].pos);
        }
    }
    // after the boundary is constructed, set the start and end of each curve to be fixed
    set_fixed();
    // assign curve-id to each edge using the curve-it assigned for each vertex
    assign_edge_curveid();
    // TODO move this to the boundary unit test
    for (const auto& v : get_vertices()) {
        assert(vertex_attrs[v.vid(*this)].t >= 0);
        if (is_boundary_vertex(v))
            assert(
                (vertex_attrs[v.vid(*this)].pos - mesh_parameters.m_boundary.t_to_uv(
                                                      vertex_attrs[v.vid(*this)].curve_id,
                                                      vertex_attrs[v.vid(*this)].t))
                    .squaredNorm() < 1e-8);
    }
}

Eigen::Matrix<uint64_t, Eigen::Dynamic, 2, Eigen::RowMajor>
AdaptiveTessellation::get_bnd_edge_matrix()
{
    int num_bnd_edge = 0;
    for (const auto& e : get_edges()) {
        if (is_boundary_edge(e)) num_bnd_edge++;
    }
    Eigen::Matrix<uint64_t, Eigen::Dynamic, 2, Eigen::RowMajor> E(num_bnd_edge, 2);
    int i = 0;
    for (const auto& e : get_edges()) {
        if (is_boundary_edge(e)) {
            E.row(i) << (uint64_t)e.vid(*this), (uint64_t)e.switch_vertex(*this).vid(*this);
            i++;
        }
    }
    return E;
}

void AdaptiveTessellation::export_uv(Eigen::MatrixXd& V, Eigen::MatrixXi& F) const
{
    V = Eigen::MatrixXd::Zero(vert_capacity(), 2);
    for (auto& t : get_vertices()) {
        auto i = t.vid(*this);
        V.row(i) = vertex_attrs[i].pos;
    }

    const auto faces = get_faces();
    F = Eigen::MatrixXi::Constant(faces.size(), 3, -1);
    for (size_t i = 0; i < faces.size(); ++i) {
        const auto vs = oriented_tri_vertices(faces[i]);
        for (size_t j = 0; j < 3; ++j) {
            F(i, j) = vs[j].vid(*this);
        }
    }
}

void AdaptiveTessellation::export_displaced_uv(
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& faces,
    Eigen::MatrixXd& vertices_uv,
    Eigen::MatrixXi& faces_uv) const
{
    export_uv(vertices_uv, faces_uv);
    faces = faces_uv;
    const size_t rows = vertices_uv.rows();
    vertices = Eigen::MatrixXd::Zero(rows, 3);
    for (size_t i = 0; i < rows; ++i) {
        vertices.row(i) = mesh_parameters.m_displacement->get(vertices_uv(i, 0), vertices_uv(i, 1));
    }
}

void AdaptiveTessellation::export_mesh_with_displacement(
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& faces,
    Eigen::MatrixXd& vertices_uv,
    Eigen::MatrixXi& faces_uv) const
{
    export_uv(vertices_uv, faces_uv);

    vertices.resize(vertices_uv.rows(), 3);
    for (int i = 0; i < vertices_uv.rows(); i++) {
        const double& u = vertices_uv(i, 0);
        const double& v = vertices_uv(i, 1);
        vertices.row(i) = mesh_parameters.m_displacement->get(u, v);
    }

    faces = faces_uv;
    remove_seams(vertices, faces);

    // get rid of unreferenced vertices in both meshes
    Eigen::MatrixXd V_buf;
    Eigen::MatrixXi F_buf;
    Eigen::MatrixXi map_old_to_new_v_ids;
    igl::remove_unreferenced(vertices, faces, V_buf, F_buf, map_old_to_new_v_ids);
    vertices = V_buf;
    faces = F_buf;

    igl::remove_unreferenced(vertices_uv, faces_uv, V_buf, F_buf, map_old_to_new_v_ids);
    vertices_uv = V_buf;
    faces_uv = F_buf;
}

void AdaptiveTessellation::export_mesh_mapped_on_input(
    Eigen::MatrixXd& vertices,
    Eigen::MatrixXi& faces,
    Eigen::MatrixXd& vertices_uv,
    Eigen::MatrixXi& faces_uv) const
{
    auto tri_signed_area =
        [](const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c) -> double {
        return 0.5 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    };

    auto compute_barycentric_coordinates = [tri_signed_area](
                                               const Eigen::Vector2d& p,
                                               const Eigen::Vector2d& a,
                                               const Eigen::Vector2d& b,
                                               const Eigen::Vector2d& c) -> Eigen::Vector3d {
        const double area = tri_signed_area(a, b, c);
        const double area_a = tri_signed_area(p, b, c);
        const double area_b = tri_signed_area(a, p, c);
        const double area_c = tri_signed_area(a, b, p);

        return {area_a / area, area_b / area, area_c / area};
    };

    auto compute_barycentric_interpolation = [](const Eigen::Vector3d& a,
                                                const Eigen::Vector3d& b,
                                                const Eigen::Vector3d& c,
                                                const Eigen::Vector3d& coords) -> Eigen::Vector3d {
        return a * coords[0] + b * coords[1] + c * coords[2];
    };

    export_uv(vertices_uv, faces_uv);

    vertices.resize(vertices_uv.rows(), 3);
    for (int i = 0; i < vertices_uv.rows(); i++) {
        const double& u = vertices_uv(i, 0);
        const double& v = vertices_uv(i, 1);
        Eigen::Vector2d uv = {u, v};
        // find input triangle and map to it
        double barycentric_min = -std::numeric_limits<double>::max();
        Eigen::Vector3d barycentric_coords;
        size_t j_min = -1;
        for (size_t j = 0; j < input_FT_.rows(); ++j) {
            const Eigen::Vector3i tri = input_FT_.row(j);
            const std::array<Eigen::Vector2d, 3> pts = {input_VT_.row(tri[0]),
                                                        input_VT_.row(tri[1]),
                                                        input_VT_.row(tri[2])};
            const Eigen::Vector3d bars =
                compute_barycentric_coordinates(uv, pts[0], pts[1], pts[2]);
            const double bar_min = bars.minCoeff();
            if (bar_min > barycentric_min) {
                barycentric_min = bar_min;
                barycentric_coords = bars;
                j_min = j;
            }
        }

        const auto& input_triangle = input_F_.row(j_min);
        Eigen::Matrix3d pts;
        pts.row(0) = input_V_.row(input_triangle[0]);
        pts.row(1) = input_V_.row(input_triangle[1]);
        pts.row(2) = input_V_.row(input_triangle[2]);

        Eigen::Vector3d p = compute_barycentric_interpolation(
            input_V_.row(input_triangle[0]),
            input_V_.row(input_triangle[1]),
            input_V_.row(input_triangle[2]),
            barycentric_coords);

        vertices.row(i) = p;
    }

    faces = faces_uv;
    remove_seams(vertices, faces);

    // get rid of unreferenced vertices in both meshes
    Eigen::MatrixXd V_buf;
    Eigen::MatrixXi F_buf;
    Eigen::MatrixXi map_old_to_new_v_ids;
    igl::remove_unreferenced(vertices, faces, V_buf, F_buf, map_old_to_new_v_ids);
    vertices = V_buf;
    faces = F_buf;

    igl::remove_unreferenced(vertices_uv, faces_uv, V_buf, F_buf, map_old_to_new_v_ids);
    vertices_uv = V_buf;
    faces_uv = F_buf;
}

void AdaptiveTessellation::remove_seams(Eigen::MatrixXd& V, Eigen::MatrixXi& F) const
{
    std::map<size_t, size_t> paired_vertices; // mapping from removed to remaining vertex

    // find seam vertices
    for (const auto& t : get_faces()) {
        const auto f_id = t.fid(*this);
        for (size_t e = 0; e < 3; ++e) {
            if (face_attrs[f_id].mirror_edges[e].has_value()) {
                Tuple e_tuple1 = tuple_from_edge(f_id, e);
                if (!e_tuple1.is_ccw(*this)) {
                    e_tuple1 = e_tuple1.switch_vertex(*this);
                }
                Tuple e_tuple2 = face_attrs[f_id].mirror_edges[e].value();
                if (!e_tuple2.is_ccw(*this)) {
                    e_tuple2 = e_tuple2.switch_vertex(*this);
                }
                const size_t v0 = e_tuple1.vid(*this);
                const size_t v1 = e_tuple1.switch_vertex(*this).vid(*this);
                const size_t v2 = e_tuple2.vid(*this);
                const size_t v3 = e_tuple2.switch_vertex(*this).vid(*this);
                if (v0 < v3) {
                    paired_vertices[v3] = v0;
                } else if (v3 < v0) {
                    paired_vertices[v0] = v3;
                }

                if (v1 < v2) {
                    paired_vertices[v2] = v1;
                } else if (v2 < v1) {
                    paired_vertices[v1] = v2;
                }
            }
        }
    }

    // make sure that all vertices are paired with the partner that has the lowest index
    for (const auto& [v0, _] : paired_vertices) {
        while (paired_vertices.count(paired_vertices[v0]) != 0) {
            paired_vertices[v0] = paired_vertices[paired_vertices[v0]];
        }
    }

    // collect all positions that belong to the same seam vertex
    std::map<size_t, std::vector<Eigen::Vector3d>> map_id_to_pos_vec;
    for (const auto& [v0, v1] : paired_vertices) {
        if (map_id_to_pos_vec.count(v1) == 0) {
            map_id_to_pos_vec[v1] = {V.row(v1)};
        }
        map_id_to_pos_vec[v1].push_back(V.row(v0));
    }

    // compute average positions
    for (const auto& [v, pos_vec] : map_id_to_pos_vec) {
        Eigen::Vector3d p(0, 0, 0);
        for (const auto& pp : pos_vec) {
            p += pp;
        }
        p /= pos_vec.size();
        V.row(v) = p;
    }

    Eigen::MatrixXi NF;
    NF.resize(F.rows(), F.cols());
    for (size_t i = 0; i < NF.rows(); ++i) {
        for (size_t j = 0; j < NF.cols(); ++j) {
            size_t new_v_id = F(i, j);
            if (paired_vertices.count(new_v_id) != 0) {
                new_v_id = paired_vertices[new_v_id];
            }
            assert(paired_vertices.count(new_v_id) == 0);
            NF(i, j) = new_v_id;
        }
    }

    // overwrite F
    F = NF;
}

void AdaptiveTessellation::write_obj_only_texture_coords(const std::filesystem::path& path)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    export_uv(V, F);

    Eigen::MatrixXd V3 = Eigen::MatrixXd::Zero(V.rows(), 3);
    V3.leftCols(2) = V;

    igl::writeOBJ(path.string(), V3, F);
    wmtk::logger().info("============>> current edge length {}", avg_edge_len());
}

void AdaptiveTessellation::write_ply(const std::filesystem::path& path)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    export_uv(V, F);

    Eigen::MatrixXd V3 = Eigen::MatrixXd::Zero(V.rows(), 3);
    V3.leftCols(2) = V;

    igl::writePLY(path.string(), V3, F);
}

void AdaptiveTessellation::write_vtk(const std::filesystem::path& path)
{
    std::vector<double> points;
    std::vector<int> elements;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    export_uv(V, F);
    wmtk::logger().info("=== # vertices {}", V.rows());
    for (int i = 0; i < V.rows(); i++) {
        auto p = mesh_parameters.m_displacement->get(V(i, 0), V(i, 1));
        points.emplace_back(p(0));
        points.emplace_back(p(1));
        points.emplace_back(p(2));
    }
    assert(points.size() == 3 * V.rows());

    for (const auto& e : get_edges()) {
        elements.emplace_back(e.vid(*this));
        elements.emplace_back(e.switch_vertex(*this).vid(*this));
    }
    assert(elements.size() == 2 * get_edges().size());
    // vector<double> scalar_field = {0., 1., 2.};
    // vector<double> vector_field = points;
    const int dim = 3;
    const int cell_size = 2;
    leanvtk::VTUWriter writer;

    std::vector<double> scalar_field;
    for (const auto& e : get_edges()) {
        if (!e.is_valid(*this)) continue;
        double cost = 0.;
        if (mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
            cost = get_quadrics_area_accuracy_error_for_split(e) * get_length2d(e);
        }
        if (mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
            cost = get_cached_area_accuracy_error_for_split(e) * get_length2d(e);
        }

        // Eigen::Matrix<double, 2, 1> pos1 = vertex_attrs[e.vid(*this)].pos;
        // Eigen::Matrix<double, 2, 1> pos2 =
        // vertex_attrs[e.switch_vertex(*this).vid(*this)].pos; Eigen::Matrix<double, 2, 1>
        // posnew = (pos1 + pos2) * 0.5; cost -=
        //     (mesh_parameters.m_displacement->get_error_per_edge(pos1, posnew) +
        //      mesh_parameters.m_displacement->get_error_per_edge(posnew, pos2));
        scalar_field.emplace_back(cost);
    }
    writer.add_cell_scalar_field("scalar_field", scalar_field);
    // writer.add_vector_field("vector_field", vector_field, dim);
    writer.write_surface_mesh(path.string(), dim, cell_size, points, elements);
}
/// @brief write vtu with elements represent per face attributes
/// @param path
void AdaptiveTessellation::write_perface_vtk(const std::filesystem::path& path)
{
    std::vector<double> points;
    std::vector<int> elements;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    export_uv(V, F);
    for (int i = 0; i < V.rows(); i++) {
        auto p = mesh_parameters.m_displacement->get(V(i, 0), V(i, 1));
        points.emplace_back(p(0));
        points.emplace_back(p(1));
        points.emplace_back(p(2));
    }
    assert(points.size() == 3 * V.rows());

    for (const auto& f : get_faces()) {
        auto its = oriented_tri_vids(f);
        elements.emplace_back(its[0]);
        elements.emplace_back(its[1]);
        elements.emplace_back(its[2]);
    }
    assert(elements.size() == 3 * get_faces().size());
    // vector<double> scalar_field = {0., 1., 2.};
    // vector<double> vector_field = points;
    const int dim = 3;
    const int cell_size = 3;
    leanvtk::VTUWriter writer;

    std::vector<double> scalar_field2;
    for (const auto& f : get_faces()) {
        auto error = face_attrs[f.fid(*this)].accuracy_measure.cached_distance_integral;
        scalar_field2.emplace_back(error);
    }
    writer.add_cell_scalar_field("scalar_field", scalar_field2);
    // writer.add_vector_field("vector_field", vector_field, dim);

    writer.write_surface_mesh(path.string(), dim, cell_size, points, elements);
}

void AdaptiveTessellation::write_obj(const std::filesystem::path& path)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd CN;
    Eigen::MatrixXd FN;
    Eigen::MatrixXd VT;
    Eigen::MatrixXi FT;
    export_mesh_with_displacement(V, F, VT, FT);

    std::map<size_t, size_t> world_to_uv_ids;
    for (Eigen::Index i = 0; i < F.rows(); ++i) {
        for (Eigen::Index j = 0; j < F.cols(); ++j) {
            world_to_uv_ids[F(i, j)] = FT(i, j);
        }
    }

    for (int i = 0; i < V.rows(); i++) {
        V.row(i) = vertex_attrs[world_to_uv_ids[i]].pos_world;
    }

    igl::writeOBJ(path.string(), V, F, CN, FN, VT, FT);
    wmtk::logger().info("============>> current edge length {}", avg_edge_len());
}

void AdaptiveTessellation::write_obj_displaced(const std::filesystem::path& path)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd CN;
    Eigen::MatrixXd FN;
    Eigen::MatrixXd VT;
    Eigen::MatrixXi FT;
    export_mesh_with_displacement(V, F, VT, FT);
    igl::writeOBJ(path.string(), V, F, CN, FN, VT, FT);
    wmtk::logger().info("============>> current edge length {}", avg_edge_len());
}

void AdaptiveTessellation::write_obj_mapped_on_input(const std::filesystem::path& path)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd CN;
    Eigen::MatrixXd FN;
    Eigen::MatrixXd VT;
    Eigen::MatrixXi FT;
    export_mesh_mapped_on_input(V, F, VT, FT);
    igl::writeOBJ(path.string(), V, F, CN, FN, VT, FT);
    wmtk::logger().info("============>> current edge length {}", avg_edge_len());
}
