#include "AdaptiveTessellation.h"
#include <fastenvelope/FastEnvelope.h>
#include <igl/Timer.h>
#include <igl/predicates/predicates.h>
#include <igl/readOBJ.h>
#include <igl/remove_unreferenced.h>
#include <lagrange/utils/timing.h>
#include <tbb/concurrent_vector.h>
#include <wmtk/quadrature/TriangleQuadrature.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <Eigen/Core>
#include <map>
#include <tracy/Tracy.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include "GlobalIntersection.h"
using namespace wmtk;
namespace adaptive_tessellation {
// TODO change this to accomodate new error
double AdaptiveTessellation::avg_edge_len() const
{
    double avg_len = 0.0;
    auto edges = get_edges();
    for (auto& e : edges) avg_len += std::sqrt(get_length3d(e));
    return avg_len / edges.size();
}

//// preprocess the mesh for remeshing
//// replace function create_paired_seam_mesh_with_offset, and mesh_construct_boundaries
// 0. create mesh
// 1. vertex_attrs: initiate uv pos and pos_world
// 2. face_attrs:   set seam local edge mirror data
// 3. build seam vertex index to color mapping
// 4. construct boundary parametrization
// 5. vertex_attrs: set boundary vertex with boudary tag
//                  set boundary vertex curve_id, and boundary paramter t
//                  set feature vertex as fixed, set strat/end/t-junction of curve fixed
// 6. edge_attrs:   set curve-id for each edge
// 7. face_attrs:   set initial accuracy error for each triangle
// 8. initiate the texture integraler
// 9. initiate the quadric integraler
void AdaptiveTessellation::mesh_preprocessing(
    const std::filesystem::path& input_mesh_path,
    const std::filesystem::path& position_image_path,
    const std::filesystem::path& normal_image_path,
    const std::filesystem::path& height_image_path)
{
    mesh_parameters.m_position_normal_paths = {position_image_path, normal_image_path};
    Eigen::MatrixXd CN, FN;
    // igl::read_triangle_mesh(input_mesh_path.string(), input_V_, input_F_);
    // igl::readOBJ(input_mesh_path.string(), V, VT, CN, F, FT, FN);
    igl::readOBJ(input_mesh_path.string(), input_V_, input_VT_, CN, input_F_, input_FT_, FN);

    {
        Eigen::MatrixXd V_buf;
        Eigen::MatrixXi F_buf;
        Eigen::MatrixXi map_old_to_new_v_ids;
        igl::remove_unreferenced(input_VT_, input_FT_, V_buf, F_buf, map_old_to_new_v_ids);
        input_VT_ = V_buf;
        input_FT_ = F_buf;
        assert(input_FT_.rows() == input_F_.rows());
    }

    wmtk::logger().info("///// #v : {} {}", input_VT_.rows(), input_VT_.cols());
    wmtk::logger().info("///// #f : {} {}", input_FT_.rows(), input_FT_.cols());
    wmtk::TriMesh m_3d;
    std::vector<std::array<size_t, 3>> tris;
    for (auto f = 0; f < input_F_.rows(); f++) {
        std::array<size_t, 3> tri = {
            (size_t)input_F_(f, 0),
            (size_t)input_F_(f, 1),
            (size_t)input_F_(f, 2)};
        tris.emplace_back(tri);
    }
    m_3d.create_mesh(input_V_.rows(), tris);
    create_mesh(input_VT_, input_FT_);
    // set up seam edges and seam vertex coloring
    Eigen::MatrixXi E0, E1;
    std::tie(E0, E1) = seam_edges_set_up(input_V_, input_F_, m_3d, input_VT_, input_FT_);
    set_seam_vertex_coloring(input_V_, input_F_, m_3d, input_VT_, input_FT_);
    assert(E0.rows() == E1.rows());
    // construct the boundary map for boundary parametrization
    mesh_parameters.m_boundary.construct_boundaries(input_VT_, input_FT_, E0, E1);
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

    const Eigen::MatrixXd box_min = input_V_.colwise().minCoeff();
    const Eigen::MatrixXd box_max = input_V_.colwise().maxCoeff();
    double max_comp = (box_max - box_min).maxCoeff();
    Eigen::MatrixXd scene_offset = -box_min;
    Eigen::MatrixXd scene_extent = box_max - box_min;
    scene_offset.array() -= (scene_extent.array() - max_comp) * 0.5;
    mesh_parameters.m_scale = max_comp;
    mesh_parameters.m_offset = scene_offset;

    // cache the initial accuracy error per triangle
    std::array<wmtk::Image, 3> displaced = wmtk::combine_position_normal_texture(
        mesh_parameters.m_scale,
        mesh_parameters.m_offset,
        position_image_path,
        normal_image_path,
        height_image_path);

    m_quadric_integral =
        wmtk::QuadricIntegral(displaced, wmtk::QuadricIntegral::QuadricType::Point);
    m_texture_integral = wmtk::TextureIntegral(std::move(displaced));
}

void AdaptiveTessellation::prepare_distance_quadrature_cached_energy()
{
    std::vector<std::array<float, 6>> uv_triangles(tri_capacity());
    std::vector<TriMesh::Tuple> tris_tuples = get_faces();
    for (int i = 0; i < tris_tuples.size(); i++) {
        auto oriented_vids = oriented_tri_vids(tris_tuples[i]);
        for (int j = 0; j < 3; j++) {
            uv_triangles[tris_tuples[i].fid(*this)][2 * j + 0] =
                vertex_attrs[oriented_vids[j]].pos[0];
            uv_triangles[tris_tuples[i].fid(*this)][2 * j + 1] =
                vertex_attrs[oriented_vids[j]].pos[1];
        }
    }
    std::vector<float> computed_errors(tri_capacity());
    m_texture_integral.get_error_per_triangle(uv_triangles, computed_errors);
    set_faces_cached_distance_integral(tris_tuples, computed_errors);
}

void AdaptiveTessellation::prepare_quadrics()
{
    wmtk::logger().info("preparing quadrics");
    auto facets = get_faces();
    std::vector<wmtk::Quadric<double>> compressed_quadrics(facets.size());
    m_quadric_integral.get_quadric_per_triangle(
        facets.size(),
        [&](int f) -> std::array<float, 6> {
            // Get triangle uv positions
            std::array<Tuple, 3> local_tuples = oriented_tri_vertices(facets[f]);
            const Eigen::Vector2f& p0 = vertex_attrs[local_tuples[0].vid(*this)].pos.cast<float>();
            const Eigen::Vector2f& p1 = vertex_attrs[local_tuples[1].vid(*this)].pos.cast<float>();
            const Eigen::Vector2f& p2 = vertex_attrs[local_tuples[2].vid(*this)].pos.cast<float>();
            return {p0.x(), p0.y(), p1.x(), p1.y(), p2.x(), p2.y()};
        },
        compressed_quadrics);
    set_faces_quadrics(facets, compressed_quadrics);
}

// return E0, E1 of corresponding seam edges in uv mesh
// set up the seam vertex coloring
std::pair<Eigen::MatrixXi, Eigen::MatrixXi> AdaptiveTessellation::seam_edges_set_up(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const wmtk::TriMesh& m_3d,
    const Eigen::MatrixXd& VT,
    const Eigen::MatrixXi& FT)
{
    // loop through faces
    // for each edge, get fid on both side.
    // if it's boundary edge, do nothing
    // if not, index to the matrix for F_3d, and check the 3 edges
    // populate E0 and E1 for boundary construction
    std::map<std::pair<size_t, size_t>, std::pair<size_t, size_t>> seam_edges;
    Eigen::MatrixXi E0(FT.rows() * 3, 2), E1(FT.rows() * 3, 2);
    int seam_edge_cnt = 0;
    for (auto fi = 0; fi < m_3d.tri_capacity(); ++fi) {
        for (auto lvi1 = 0; lvi1 < 3; ++lvi1) {
            auto lvi2 = (lvi1 + 1) % 3;
            auto local_eid = 3 - lvi1 - lvi2;
            // construct the edge tuple of current vertex in 3d mesh
            Tuple edge1_3d = Tuple(F(fi, lvi1), local_eid, fi, m_3d);
            assert(F(fi, lvi1) == edge1_3d.vid(m_3d));
            if (!edge1_3d.switch_face(m_3d).has_value()) {
                // Boundary edge, skipping...
                continue;
            } else {
                auto edge2_3d = edge1_3d.switch_face(m_3d).value();
                auto fj = edge2_3d.fid(m_3d);
                size_t lvj1, lvj2;
                for (auto i = 0; i < 3; i++) {
                    if (F(fj, i) == edge1_3d.vid(m_3d)) lvj1 = i;
                    if (F(fj, i) == edge1_3d.switch_vertex(m_3d).vid(m_3d)) lvj2 = i;
                }

                assert(F(fi, lvi1) == F(fj, lvj1));
                assert(F(fi, lvi2) == F(fj, lvj2));
                // set up seam edge
                if ((FT(fi, lvi1) != FT(fj, lvj1)) || (FT(fi, lvi2) != FT(fj, lvj2))) {
                    // this is a seam. init the mirror_edge tuple
                    // the orientation of the mirror edges is inccw (half edge conventions)
                    // However, the edge tuple in operations have arbitraty orientation
                    // !!! need to check orientations of mirror edge in operations !!!!!
                    TriMesh::Tuple seam_edge_fj(FT(fj, lvj2), (3 - lvj1 - lvj2), fj, *this);
                    if (!seam_edge_fj.is_ccw(*this))
                        seam_edge_fj = seam_edge_fj.switch_vertex(*this);
                    face_attrs[fi].mirror_edges[local_eid] =
                        std::make_optional<wmtk::TriMesh::Tuple>(seam_edge_fj);

                    TriMesh::Tuple seam_edge_fi(FT(fi, lvi1), local_eid, fi, *this);
                    if (!seam_edge_fi.is_ccw(*this))
                        seam_edge_fi = seam_edge_fi.switch_vertex(*this);
                    face_attrs[fj].mirror_edges[(3 - lvj1 - lvj2)] =
                        std::make_optional<wmtk::TriMesh::Tuple>(seam_edge_fi);
                    // check if seam_edge_fj is already in the map
                    size_t ei0 = std::min(FT(fi, lvi1), FT(fi, lvi2));
                    size_t ei1 = std::max(FT(fi, lvi1), FT(fi, lvi2));
                    size_t ej0 = std::min(FT(fj, lvj1), FT(fj, lvj2));
                    size_t ej1 = std::max(FT(fj, lvj1), FT(fj, lvj2));
                    if (seam_edges.find(std::pair{ei0, ei1}) != seam_edges.end()) {
                        assert((seam_edges[{ei0, ei1}] == std::pair<size_t, size_t>(ej0, ej1)));
                    } else {
                        // if not add it to the map and add to the edge matrix
                        seam_edges[{ej0, ej1}] = std::pair<size_t, size_t>(ei0, ei1);
                        E0.row(seam_edge_cnt) << FT(fi, lvi1), FT(fi, lvi2);
                        E1.row(seam_edge_cnt) << FT(fj, lvj1), FT(fj, lvj2);
                        seam_edge_cnt++;
                    }
                }
            }
        }
    }
    E0.conservativeResize(seam_edge_cnt, 2);
    E1.conservativeResize(seam_edge_cnt, 2);
    assert(E0.rows() == E1.rows());
    return {E0, E1};
} // namespace adaptive_tessellation

// TODO wait why I'm doing this? The coloring isn't used anymore.
// i wanted to use it for get_all_mirror_vertex
// for each vertex that's seam vertex, assign same color for mirror vertices
// build the mapping from uv_index to color
// and mapping from color to uv_index
void AdaptiveTessellation::set_seam_vertex_coloring(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const wmtk::TriMesh& m_3d,
    const Eigen::MatrixXd& VT,
    const Eigen::MatrixXi& FT)
{
    color_to_uv_indices.reserve(VT.rows());
    assert(uv_index_to_color.empty());
    int current_color = 0;
    for (auto fi = 0; fi < m_3d.tri_capacity(); ++fi) {
        for (auto lvi1 = 0; lvi1 < 3; ++lvi1) {
            auto lvi2 = (lvi1 + 1) % 3;
            auto local_eid = 3 - lvi1 - lvi2;
            // construct the edge tuple of current vertex in 3d mesh
            Tuple edge1_3d = Tuple(F(fi, lvi1), local_eid, fi, m_3d);
            assert(F(fi, lvi1) == edge1_3d.vid(m_3d));
            // current vertex in 2d mesh
            int current_v = FT(fi, lvi1);
            if (!edge1_3d.switch_face(m_3d).has_value()) {
                // Boundary edge
                continue;
            } else if (uv_index_to_color.find(current_v) != uv_index_to_color.end()) {
                // already colored, skipping...
                continue;
            } else {
                auto edge2_3d = edge1_3d.switch_face(m_3d).value();
                auto fj_3d = edge2_3d.fid(m_3d);
                size_t lvj1_3d, lvj2_3d;
                for (auto i = 0; i < 3; i++) {
                    if (F(fj_3d, i) == edge1_3d.vid(m_3d)) lvj1_3d = i;
                    if (F(fj_3d, i) == edge1_3d.switch_vertex(m_3d).vid(m_3d)) lvj2_3d = i;
                }

                assert(F(fi, lvi1) == F(fj_3d, lvj1_3d));
                assert(F(fi, lvi2) == F(fj_3d, lvj2_3d));
                // edge1_3d is a seam edge
                if ((current_v != FT(fj_3d, lvj1_3d)) || (FT(fi, lvi2) != FT(fj_3d, lvj2_3d))) {
                    uv_index_to_color.insert({current_v, current_color});
                    if (current_color < color_to_uv_indices.size() &&
                        std::find(
                            color_to_uv_indices[current_color].begin(),
                            color_to_uv_indices[current_color].end(),
                            current_v) == color_to_uv_indices[current_color].end()) {
                        // color already exists
                        color_to_uv_indices[current_color].emplace_back(current_v);
                    } else {
                        // add new color
                        color_to_uv_indices.emplace_back(1, current_v);
                    }
                    for (const auto& e_3d : m_3d.get_one_ring_edges_for_vertex(edge1_3d)) {
                        if (!e_3d.switch_face(m_3d).has_value()) {
                            // Boundary edge, skipping...
                            continue;
                        } else {
                            for (const auto e2_3d : {e_3d, e_3d.switch_face(m_3d).value()}) {
                                auto fj = e2_3d.fid(m_3d);
                                size_t lvj1;
                                for (auto i = 0; i < 3; i++) {
                                    if (F(fj, i) == edge1_3d.vid(m_3d)) lvj1 = i;
                                }
                                assert(F(fi, lvi1) == F(fj, lvj1));
                                // this is a mirror vertex at a seam edge
                                if (current_v != FT(fj, lvj1)) {
                                    int current_v_color = uv_index_to_color.at(current_v);
                                    assert(current_v_color == current_color);
                                    // the mirror vertex has not been colored yet
                                    if (uv_index_to_color.find(FT(fj, lvj1)) ==
                                        uv_index_to_color.end()) {
                                        // add the color or the primary vertex to the mirror vertex
                                        uv_index_to_color.insert({FT(fj, lvj1), current_v_color});
                                    }
                                    // if the mirror vertex is not in the color busket, add it
                                    if (std::find(
                                            color_to_uv_indices[current_v_color].begin(),
                                            color_to_uv_indices[current_v_color].end(),
                                            FT(fj, lvj1)) ==
                                        color_to_uv_indices[current_v_color].end()) {
                                        color_to_uv_indices[current_v_color].emplace_back(
                                            FT(fj, lvj1));
                                    }
                                    assert(uv_index_to_color[FT(fj, lvj1)] == current_v_color);
                                }
                            }
                        }
                    }
                    current_color++;
                }
            }
        }
    }
    color_to_uv_indices.resize(current_color);
}

void AdaptiveTessellation::set_parameters(
    const double target_accuracy,
    const double target_edge_length,
    const wmtk::Image& image,
    const WrappingMode wrapping_mode,
    const SAMPLING_MODE sampling_mode,
    const DISPLACEMENT_MODE displacement_mode,
    const ENERGY_TYPE energy_type,
    const EDGE_LEN_TYPE edge_len_type,
    const bool boundary_parameter)
{
    if (mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::EDGE_ACCURACY ||
        mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY ||
        mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS)
        mesh_parameters.m_accuracy_threshold = target_accuracy;
    mesh_parameters.m_quality_threshold = target_edge_length;

    // setting needs to be in the order of image-> displacement-> energy-> edge_length
    // set the image first since it is used for displacement and energy setting
    set_image_function(image, wrapping_mode);
    mesh_parameters.m_sampling_mode = sampling_mode;
    set_displacement(displacement_mode);
    set_energy(energy_type);
    set_edge_length_measurement(edge_len_type);
    mesh_parameters.m_boundary_parameter = boundary_parameter;
}

void AdaptiveTessellation::set_parameters(
    const double target_edge_length,
    const std::function<DScalar(const DScalar&, const DScalar&)>& displacement_function,
    const EDGE_LEN_TYPE edge_len_type,
    const ENERGY_TYPE energy_type,
    const bool boundary_parameter)
{
    mesh_parameters.m_target_l = target_edge_length;
    mesh_parameters.m_get_z = displacement_function;
    set_energy(energy_type); // set the displacement_function first since it is used for
                             // energy setting
    set_edge_length_measurement(edge_len_type); // set the default to get_legnth_3d
    mesh_parameters.m_boundary_parameter = boundary_parameter;
}
// using boundary parametrization,
// find the vertex that are the start and end of each cruve and set them as fixed
// use the vertex curve-id and assign edge curve-id
void AdaptiveTessellation::set_fixed()
{
    for (int curve_id = 0; curve_id < mesh_parameters.m_boundary.num_curves(); curve_id++) {
        if (mesh_parameters.m_boundary.is_periodic(curve_id)) {
            // periodic curve has no fixed vertex
            continue;
        }
        // set the first and last vertex as fixed
        auto uv_first = mesh_parameters.m_boundary.t_to_uv(curve_id, 0.);
        auto uv_last = mesh_parameters.m_boundary.t_to_uv(
            curve_id,
            mesh_parameters.m_boundary.upper_bound(curve_id));
        // find the closest points to the uv_first and uv_last
        double dist_first = std::numeric_limits<double>::infinity();
        double dist_last = std::numeric_limits<double>::infinity();
        size_t first_vid = -1;
        size_t last_vid = -1;
        for (auto& v : get_vertices()) {
            if ((vertex_attrs[v.vid(*this)].pos - uv_first).squaredNorm() < dist_first) {
                dist_first = (vertex_attrs[v.vid(*this)].pos - uv_first).squaredNorm();
                first_vid = v.vid(*this);
            }
            if ((vertex_attrs[v.vid(*this)].pos - uv_last).squaredNorm() < dist_last) {
                dist_last = (vertex_attrs[v.vid(*this)].pos - uv_last).squaredNorm();
                last_vid = v.vid(*this);
            }
        }
        assert(dist_first < 1e-8);
        assert(dist_last < 1e-8);
        vertex_attrs[first_vid].fixed = true;
        vertex_attrs[last_vid].fixed = true;
    }
}

/// @brief set the v as feature vertex, that is, it is a boundary vertex and the  incident boundary vertices are not colinear
/// @param v
void AdaptiveTessellation::set_feature(Tuple& v)
{
    // default fixed is false
    vertex_attrs[v.vid(*this)].fixed = false;
    // only set the feature if it is a boundary vertex
    assert(is_boundary_vertex(v));
    // get 3 vertices

    std::vector<size_t> incident_boundary_verts;
    auto one_ring = get_one_ring_edges_for_vertex(v);
    for (const auto& e : one_ring) {
        if (is_boundary_edge(e)) incident_boundary_verts.emplace_back(e.vid(*this));
    }
    // every boundary vertex has to have 2 neighbors that are boundary vertices
    assert(incident_boundary_verts.size() == 2);
    const auto& p1 = vertex_attrs[v.vid(*this)].pos;
    const auto& p2 = vertex_attrs[incident_boundary_verts[0]].pos;
    const auto& p3 = vertex_attrs[incident_boundary_verts[1]].pos;
    // check if they are co linear. set fixed = true if not
    double costheta = ((p2 - p1).stableNormalized()).dot((p3 - p1).stableNormalized()); // cos theta
    double theta = std::acos(std::clamp<double>(costheta, -1, 1));
    if (theta <= M_PI / 2) vertex_attrs[v.vid(*this)].fixed = true;
}

// assuming the m_triwild_displacement in mesh_parameter has been set
void AdaptiveTessellation::set_energy(const ENERGY_TYPE energy_type)
{
    std::unique_ptr<Energy> energy_ptr;
    switch (energy_type) {
    case ENERGY_TYPE::AMIPS: energy_ptr = std::make_unique<wmtk::AMIPS>(); break;
    case ENERGY_TYPE::SYMDI: energy_ptr = std::make_unique<wmtk::SymDi>(); break;
    case ENERGY_TYPE::EDGE_LENGTH:
        energy_ptr = std::make_unique<wmtk::EdgeLengthEnergy>(mesh_parameters.m_get_z);
        break;
    case ENERGY_TYPE::EDGE_QUADRATURE:
        energy_ptr = std::make_unique<wmtk::AccuracyEnergy>(mesh_parameters.m_displacement);
        break;
    case ENERGY_TYPE::AREA_QUADRATURE:
        energy_ptr = std::make_unique<wmtk::AreaAccuracyEnergy>(
            mesh_parameters.m_displacement,
            std::cref(m_texture_integral));
        break;
    case ENERGY_TYPE::QUADRICS:
        energy_ptr = std::make_unique<wmtk::QuadricEnergy>(mesh_parameters.m_displacement);
        break;
    }

    mesh_parameters.m_energy = std::move(energy_ptr);
}

void AdaptiveTessellation::set_faces_cached_distance_integral(
    const std::vector<TriMesh::Tuple>& tris,
    const std::vector<float>& computed_errors)
{
    // update the face_attrs with modified tris error
    for (int i = 0; i < tris.size(); i++) {
        face_attrs[tris[i].fid(*this)].accuracy_measure.cached_distance_integral =
            computed_errors[i];
    }
}

void AdaptiveTessellation::set_faces_quadrics(
    const std::vector<TriMesh::Tuple>& tris,
    const std::vector<wmtk::Quadric<double>>& compressed_quadrics)
{
    throw std::runtime_error("do not use");
    // update the face_attrs with modified tris error
    for (int i = 0; i < tris.size(); i++) {
        // face_attrs[tris[i].fid(*this)].accuracy_measure.quadric = compressed_quadrics[i];
    }
}


void AdaptiveTessellation::set_edge_length_measurement(const EDGE_LEN_TYPE edge_len_type)
{
    mesh_parameters.m_edge_length_type = edge_len_type;
    switch (edge_len_type) {
    case EDGE_LEN_TYPE::LINEAR2D:
        mesh_parameters.m_get_length = [&](const Tuple& edge_tuple) -> double {
            return this->get_length2d(edge_tuple);
        };
        break;
    case EDGE_LEN_TYPE::LINEAR3D:
        mesh_parameters.m_get_length = [&](const Tuple& edge_tuple) -> double {
            return this->get_length3d(edge_tuple);
        };
        break;
    case EDGE_LEN_TYPE::N_IMPLICIT_POINTS:
        mesh_parameters.m_get_length = [&](const Tuple& edge_tuple) -> double {
            return this->get_length_n_implicit_points(edge_tuple);
        };
        break;
    case EDGE_LEN_TYPE::PT_PER_PIXEL:
        mesh_parameters.m_get_length = [&](const Tuple& edge_tuple) -> double {
            return this->get_length_1ptperpixel(edge_tuple);
        };
        break;
    case EDGE_LEN_TYPE::MIPMAP:
        mesh_parameters.m_get_length = [&](const Tuple& edge_tuple) -> double {
            return this->get_length_mipmap(edge_tuple);
        };
        break;
    case EDGE_LEN_TYPE::EDGE_ACCURACY:
        mesh_parameters.m_get_length = [&](const Tuple& edge_tuple) -> double {
            return this->get_edge_accuracy_error(edge_tuple);
        };
        break;
        // for AREA_ACCURACY and TRI_QUADRICS errors are calculated using dedicated functions
        // but we want to do the preprocessing for each facefor them

    // we cache each face error for AREA_ACCURACY
    case EDGE_LEN_TYPE::AREA_ACCURACY: prepare_distance_quadrature_cached_energy(); break;
    // we cache each face quadrics for TRI_QUADRICS
    case EDGE_LEN_TYPE::TRI_QUADRICS: prepare_quadrics(); break;
    default: break;
    }
}

void AdaptiveTessellation::set_image_function(
    const wmtk::Image& image,
    const WrappingMode wrapping_mode)
{
    mesh_parameters.m_wrapping_mode = wrapping_mode;
    mesh_parameters.m_image = image;
    mesh_parameters.m_get_z = [this](const DScalar& u, const DScalar& v) -> DScalar {
        throw std::runtime_error("do not use");
        return this->mesh_parameters.m_image.get(u, v);
    };
    mesh_parameters.m_image_get_coordinate =
        [this](const double& x, const double& y) -> std::pair<int, int> {
        throw std::runtime_error("do not use");
        auto [xx, yy] = this->mesh_parameters.m_image.get_pixel_index(x, y);
        return {
            this->mesh_parameters.m_image.get_coordinate(xx, this->mesh_parameters.m_wrapping_mode),
            this->mesh_parameters.m_image.get_coordinate(
                yy,
                this->mesh_parameters.m_wrapping_mode)};
    };
    // skipped (not used currently, and image can be empty when using vector displacement)
    // mesh_parameters.m_mipmap = wmtk::MipMap(image);
    // mesh_parameters.m_mipmap.set_wrapping_mode(wrapping_mode);
}

void AdaptiveTessellation::set_displacement(const DISPLACEMENT_MODE displacement_mode)
{
    // needs to be called after m_image is initiated
    // can be also set depending on a user parameter that initialize different Displacement
    // type
    std::shared_ptr<Displacement> displacement_ptr;
    switch (displacement_mode) {
    case DISPLACEMENT_MODE::MESH_3D: {
        std::array<wmtk::Image, 6> position_normal_images;
        for (size_t i = 0; i < 2; i++) {
            std::filesystem::path path = mesh_parameters.m_position_normal_paths[i];
            wmtk::logger().debug("======= path {} {}", i, path);
            std::array<wmtk::Image, 3> rgb_image = wmtk::load_rgb_image(path);
            position_normal_images[i * 3 + 0] = rgb_image[0];
            position_normal_images[i * 3 + 1] = rgb_image[1];
            position_normal_images[i * 3 + 2] = rgb_image[2];
        }
        displacement_ptr = std::make_shared<DisplacementMesh>(
            mesh_parameters.m_image,
            position_normal_images,
            mesh_parameters.m_sampling_mode,
            mesh_parameters.m_scale,
            mesh_parameters.m_offset);
        break;
    }
    case DISPLACEMENT_MODE::PLANE:
        displacement_ptr = std::make_shared<DisplacementPlane>(
            mesh_parameters.m_image,
            mesh_parameters.m_sampling_mode);
        break;
    case DISPLACEMENT_MODE::VECTOR: {
        // Directly use baked positions as our displaced 3d coordinate
        auto displaced_positions = wmtk::load_rgb_image(mesh_parameters.m_position_normal_paths[0]);
        displacement_ptr = std::make_shared<DisplacementVector>(
            displaced_positions,
            mesh_parameters.m_sampling_mode,
            mesh_parameters.m_scale,
            mesh_parameters.m_offset);
        break;
    }
    default: break;
    }
    mesh_parameters.m_displacement = displacement_ptr;
}

void AdaptiveTessellation::set_projection()
{
    struct Data
    {
        RowMatrix2<Index> E;
        RowMatrix2<Scalar> V;
        lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2> aabb;
    };
    auto data = std::make_shared<Data>();
    data->E = get_bnd_edge_matrix();
    RowMatrix2<Scalar> V_aabb = Eigen::MatrixXd::Zero(vert_capacity(), 2);
    for (int i = 0; i < vert_capacity(); ++i) {
        V_aabb.row(i) << vertex_attrs[i].pos[0], vertex_attrs[i].pos[1];
    }
    data->V = V_aabb;
    data->aabb =
        lagrange::bvh::EdgeAABBTree<RowMatrix2<Scalar>, RowMatrix2<Index>, 2>(data->V, data->E);
    std::function<Eigen::RowVector2d(const Eigen::RowVector2d& p)> projection =
        [data = std::move(data)](const Eigen::RowVector2d& p) -> Eigen::RowVector2d {
        uint64_t ind = 0;
        double distance = 0.0;
        Eigen::RowVector2d p_ret;
        assert(data != nullptr);
        (data->aabb).get_closest_point(p, ind, p_ret, distance);
        return p_ret;
    };

    mesh_parameters.m_get_closest_point = std::move(projection);
}

void AdaptiveTessellation::set_vertex_world_positions()
{
    for (const auto& v : get_vertices()) {
        const size_t v_id = v.vid(*this);
        vertex_attrs[v_id].pos_world = mesh_parameters.m_displacement->get(
            vertex_attrs[v_id].pos[0],
            vertex_attrs[v_id].pos[1]);
    }

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
    std::map<size_t, std::vector<size_t>> map_id_to_ids;
    for (const auto& [v0, v1] : paired_vertices) {
        if (map_id_to_pos_vec.count(v1) == 0) {
            map_id_to_pos_vec[v1] = {vertex_attrs[v1].pos_world};
            map_id_to_ids[v1].push_back(v1);
        }
        map_id_to_pos_vec[v1].push_back(vertex_attrs[v0].pos_world);
        map_id_to_ids[v1].push_back(v0);
    }

    // compute averate positions
    for (const auto& [v, pos_vec] : map_id_to_pos_vec) {
        Eigen::Vector3d p(0, 0, 0);
        for (const auto& pp : pos_vec) {
            p += pp;
        }
        p /= pos_vec.size();
        for (const auto& vv : map_id_to_ids[v]) {
            vertex_attrs[vv].pos_world = p;
        }
    }
}


bool AdaptiveTessellation::invariants(const std::vector<Tuple>& new_tris)
{
    TriMesh::invariants(new_tris);
    if (mesh_parameters.m_has_envelope) {
        for (auto& t : new_tris) {
            std::array<Eigen::Vector3d, 3> tris;
            auto vs = oriented_tri_vertices(t);
            for (auto j = 0; j < 3; j++) {
                tris[j] << vertex_attrs[vs[j].vid(*this)].pos(0),
                    vertex_attrs[vs[j].vid(*this)].pos(1), 0.0;
            }
            if (mesh_parameters.m_envelope.is_outside(tris)) {
                wmtk::logger().debug("envelop invariant fail");
                return false;
            }
        }
    }

    for (auto& t : new_tris) {
        Eigen::Vector2d a, b, c;
        auto verts = oriented_tri_vertices(t);
        assert(verts.size() == 3);
        a = vertex_attrs[verts[0].vid(*this)].pos;
        b = vertex_attrs[verts[1].vid(*this)].pos;
        c = vertex_attrs[verts[2].vid(*this)].pos;

        // check both inverted and exact colinear
        if (wmtk::orient2d_t(a, b, c) != 1) {
            wmtk::logger().debug(
                "----{} false in orientation and collinear {}",
                t.fid(*this),
                wmtk::orient2d_t(a, b, c));
            wmtk::logger().debug("{} {} {}", a, b, c);
            // write_ply("rejected_split_" + std::to_string(t.fid(*this)) + ".ply");
            return false;
        }
    }
    return true;
}
std::vector<AdaptiveTessellation::Tuple> AdaptiveTessellation::new_edges_after(
    const std::vector<AdaptiveTessellation::Tuple>& tris) const
{
    std::vector<AdaptiveTessellation::Tuple> new_edges;

    for (const auto& t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(*this), j));
        }
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}

double AdaptiveTessellation::get_length2d(const Tuple& edge_tuple) const
{
    auto v1 = edge_tuple.vid(*this);
    auto v2 = edge_tuple.switch_vertex(*this).vid(*this);
    return (vertex_attrs[v1].pos - vertex_attrs[v2].pos).stableNorm();
}

double AdaptiveTessellation::get_length3d(const Tuple& edge_tuple) const
{
    auto v1 = edge_tuple.vid(*this);
    auto v2 = edge_tuple.switch_vertex(*this).vid(*this);

    auto v1_3d =
        mesh_parameters.m_displacement->get(vertex_attrs[v1].pos(0), vertex_attrs[v1].pos(1));
    auto v2_3d =
        mesh_parameters.m_displacement->get(vertex_attrs[v2].pos(0), vertex_attrs[v2].pos(1));

    return (v1_3d - v2_3d).stableNorm();
}

double AdaptiveTessellation::get_length_n_implicit_points(const Tuple& edge_tuple) const
{
    const auto& v1 = edge_tuple.vid(*this);
    const auto& v2 = edge_tuple.switch_vertex(*this).vid(*this);
    const auto& v12d = vertex_attrs[v1].pos;
    const auto& v22d = vertex_attrs[v2].pos;

    double length = 0.0;

    // add 3d displacement. add n implicit points to approximate quadrature of the curve
    std::vector<Eigen::Vector3d> quadrature;
    for (size_t i = 0; i < 6; i++) {
        auto tmp_v2d = v12d * (5 - i) / 5. + v22d * i / 5.;
        quadrature.emplace_back(mesh_parameters.m_project_to_3d(tmp_v2d(0), tmp_v2d(1)));
    }
    for (size_t i = 0; i < 5; i++) {
        length += (quadrature[i] - quadrature[i + 1]).stableNorm();
    }
    return length;
}

double AdaptiveTessellation::get_length_1ptperpixel(const Tuple& edge_tuple) const
{
    throw std::runtime_error("do no use");
    const auto& vid1 = edge_tuple.vid(*this);
    const auto& vid2 = edge_tuple.switch_vertex(*this).vid(*this);
    const auto& v12d = vertex_attrs[vid1].pos;
    const auto& v22d = vertex_attrs[vid2].pos;

    double length = 0.0;
    // get the pixel index of p1 and p2
    const auto [xx1, yy1] = mesh_parameters.m_image_get_coordinate(v12d(0), v12d(1));
    const auto [xx2, yy2] = mesh_parameters.m_image_get_coordinate(v22d(0), v22d(1));
    // get all the pixels in between p1 and p2
    const size_t pixel_num = std::max(abs(xx2 - xx1), abs(yy2 - yy1));
    // sum up the length
    // add 3d displacement. add n implicit points to approximate quadrature of the curve
    if (pixel_num <= 0) return -1.;
    assert(pixel_num > 0);
    std ::vector<Eigen::Vector3d> quadrature;
    for (size_t i = 0; i < (pixel_num + 1); i++) {
        const double u = static_cast<double>(i) / pixel_num;
        auto tmp_v2d = v12d * (1. - u) + v22d * u;
        quadrature.emplace_back(mesh_parameters.m_project_to_3d(tmp_v2d(0), tmp_v2d(1)));
    }
    for (size_t i = 0; i < pixel_num; i++) {
        length += (quadrature[i] - quadrature[i + 1]).stableNorm();
    }
    return length;
}

double AdaptiveTessellation::get_length_mipmap(const Tuple& edge_tuple) const
{
    const auto& vid1 = edge_tuple.vid(*this);
    const auto& vid2 = edge_tuple.switch_vertex(*this).vid(*this);
    const auto& v12d = vertex_attrs[vid1].pos;
    const auto& v22d = vertex_attrs[vid2].pos;
    int idx = 0;
    int pixel_num = -1;

    std::tie(idx, pixel_num) = mesh_parameters.m_mipmap.get_mipmap_level_pixelnum(v12d, v22d);
    const auto& image = mesh_parameters.m_mipmap.get_image(idx);
    double length = 0.0;

    if (pixel_num <= 0) return -1.;
    // sum up the length
    // add 3d displacement. add n implicit points to approximate quadrature of the curve
    std::vector<Eigen::Vector3d> quadrature;
    for (int i = 0; i < (pixel_num + 1); i++) {
        auto tmp_v2d = v12d * (pixel_num - i) / pixel_num + v22d * i / pixel_num;
        double z = image.get(tmp_v2d(0), tmp_v2d(1));
        quadrature.emplace_back(tmp_v2d(0), tmp_v2d(1), z);
    }
    for (size_t i = 0; i < pixel_num; i++) {
        length += (quadrature[i] - quadrature[i + 1]).stableNorm();
    }
    return length;
}

double AdaptiveTessellation::get_quality(const Tuple& loc, int idx) const
{
    // Global ids of the vertices of the triangle
    auto its = oriented_tri_vids(loc);

    // Temporary variable to store the stacked coordinates of the triangle
    std::array<double, 6> T;
    auto energy = -1.;
    for (size_t k = 0; k < 3; k++)
        for (size_t j = 0; j < 2; j++) T[k * 2 + j] = vertex_attrs[its[k]].pos[j];

    // Energy evaluation
    // energy = wmtk::AMIPS2D_energy(T);
    wmtk::State state = {};
    state.input_triangle = T;
    state.scaling = mesh_parameters.m_target_l;
    state.idx = idx;
    mesh_parameters.m_energy->eval(state);
    energy = state.value;

    // Filter for numerical issues
    if (std::isinf(energy) || std::isnan(energy)) return mesh_parameters.MAX_ENERGY;

    return energy;
}

double AdaptiveTessellation::get_edge_accuracy_error(const Tuple& edge_tuple) const
{
    const auto& vid1 = edge_tuple.vid(*this);
    const auto& vid2 = edge_tuple.switch_vertex(*this).vid(*this);
    const auto& v12d = vertex_attrs[vid1].pos;
    const auto& v22d = vertex_attrs[vid2].pos;

    return mesh_parameters.m_displacement->get_error_per_edge(v12d, v22d);
}

double AdaptiveTessellation::get_area_accuracy_error_per_face(const Tuple& edge_tuple) const
{
    auto vids = oriented_tri_vertices(edge_tuple);
    Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle;
    for (int i = 0; i < 3; i++) {
        triangle.row(i) = vertex_attrs[vids[i].vid(*this)].pos;
    }
    auto triangle_area = wmtk::polygon_signed_area(triangle);
    if (triangle_area < 0.) {
        Eigen::Matrix<double, 1, 2, Eigen::RowMajor> tmp;
        tmp = triangle.row(0);
        triangle.row(0) = triangle.row(1);
        triangle.row(1) = tmp;
    }
    return mesh_parameters.m_displacement->get_error_per_triangle(triangle);
}

double AdaptiveTessellation::get_area_accuracy_error_per_face_triangle_matrix(
    Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle) const
{
    auto triangle_area = wmtk::polygon_signed_area(triangle);
    if (triangle_area < 0.) {
        Eigen::Matrix<double, 1, 2, Eigen::RowMajor> tmp;
        tmp = triangle.row(0);
        triangle.row(0) = triangle.row(1);
        triangle.row(1) = tmp;
    }
    return mesh_parameters.m_displacement->get_error_per_triangle(triangle);
}

double AdaptiveTessellation::get_cached_area_accuracy_error_for_split(const Tuple& edge_tuple) const
{
    double error1 = 0.;
    double error2 = 0.;

    error1 = face_attrs[edge_tuple.fid(*this)].accuracy_measure.cached_distance_integral;
    if (edge_tuple.switch_face(*this).has_value()) {
        error2 = face_attrs[edge_tuple.switch_face(*this).value().fid(*this)]
                     .accuracy_measure.cached_distance_integral;
    } else {
        if (is_seam_edge(edge_tuple))
            error2 = face_attrs[get_oriented_mirror_edge(edge_tuple).fid(*this)]
                         .accuracy_measure.cached_distance_integral;
        else
            error2 = error1;
    }

    return (error1 + error2);
}

std::tuple<double, double, double> AdaptiveTessellation::get_projected_relative_error_for_split(
    const Tuple& edge_tuple) const
{
    throw std::runtime_error("ATshould not be used");

    ///////// THIS IS NOT USED
    double error, error1, error2;
    error1 = face_attrs[edge_tuple.fid(*this)].accuracy_measure.cached_distance_integral;
    if (edge_tuple.switch_face(*this).has_value()) {
        error2 = face_attrs[edge_tuple.switch_face(*this).value().fid(*this)]
                     .accuracy_measure.cached_distance_integral;
    } else
        error2 = error1;
    double e_before = error1 + error2;

    double e_after, error_after_1, error_after_2, error_after_3, error_after_4;
    std::vector<std::array<float, 6>> new_triangles(2);
    std::vector<float> new_computed_errors(2);

    const Eigen::Vector2f mid_point_uv =
        (0.5 * (vertex_attrs[edge_tuple.vid(*this)].pos +
                vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].pos))
            .cast<float>();
    const Eigen::Vector2f uv1 = vertex_attrs[edge_tuple.vid(*this)].pos.cast<float>();
    const Eigen::Vector2f uv2 =
        vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].pos.cast<float>();
    const Eigen::Vector2f uv3 =
        vertex_attrs[edge_tuple.switch_edge(*this).switch_vertex(*this).vid(*this)]
            .pos.cast<float>();

    new_triangles[0] = {uv1(0), uv1(1), mid_point_uv(0), mid_point_uv(1), uv3(0), uv3(1)};
    new_triangles[1] = {uv2(0), uv2(1), mid_point_uv(0), mid_point_uv(1), uv3(0), uv3(1)};
    m_texture_integral.get_error_per_triangle(new_triangles, new_computed_errors);
    error_after_1 = new_computed_errors[0];
    error_after_2 = new_computed_errors[1];
    if (edge_tuple.switch_face(*this).has_value()) {
        const Eigen::Vector2f uv4 =
            vertex_attrs
                [edge_tuple.switch_face(*this).value().switch_edge(*this).switch_vertex(*this).vid(
                     *this)]
                    .pos.cast<float>();

        new_triangles[0] = {uv1(0), uv1(1), mid_point_uv(0), mid_point_uv(1), uv4(0), uv4(1)};
        new_triangles[1] = {uv2(0), uv2(1), mid_point_uv(0), mid_point_uv(1), uv4(0), uv4(1)};
        m_texture_integral.get_error_per_triangle(new_triangles, new_computed_errors);

        error_after_3 = new_computed_errors[0];
        error_after_4 = new_computed_errors[1];
    } else {
        // TODO set error after 3 and 4 to the the mirror face error when it's seam
        error_after_3 = error_after_1;
        error_after_4 = error_after_2;
    }
    e_after = error_after_1 + error_after_2 + error_after_3 + error_after_4;
    error = e_before - e_after;
    return {error, error1, error2};
}

double AdaptiveTessellation::get_one_ring_quadrics_error_for_vertex(const Tuple& v) const
{
    throw std::runtime_error("do not use");
    double ret = 0.0;
    wmtk::Quadric<double> q;
    for (const Tuple& tri : get_one_ring_tris_for_vertex(v)) {
        // q += get_face_attrs(tri).accuracy_measure.quadric;
    }
    auto v_pos = vertex_attrs[v.vid(*this)].pos;
    Eigen::Matrix<double, 3, 1> v_world_pos =
        mesh_parameters.m_displacement->get(v_pos(0), v_pos(1));
    ret = q(v_world_pos);
    return ret;
}

double AdaptiveTessellation::get_quadric_error_for_face(const Tuple& f) const
{
    throw std::runtime_error("do not use");
    wmtk::Quadric<double> q;
    // q += get_face_attrs(f).accuracy_measure.quadric;

    Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle_uv;
    triangle_uv.row(0) = vertex_attrs[f.vid(*this)].pos.transpose();
    triangle_uv.row(1) = vertex_attrs[f.switch_vertex(*this).vid(*this)].pos.transpose();
    triangle_uv.row(2) =
        vertex_attrs[f.switch_edge(*this).switch_vertex(*this).vid(*this)].pos.transpose();

    auto get = [&](auto uv) -> Eigen::Matrix<double, 1, 3> {
        return mesh_parameters.m_displacement->get(uv(0), uv(1)).transpose();
    };

    Eigen::Matrix3d triangle_3d;
    triangle_3d.row(0) = get(triangle_uv.row(0));
    triangle_3d.row(1) = get(triangle_uv.row(1));
    triangle_3d.row(2) = get(triangle_uv.row(2));

    if (0) {
        // Quadric integrated over the face
        auto triangle_area_2d = [](auto a, auto b, auto c) {
            return ((a[0] - b[0]) * (a[1] - c[1]) - (a[0] - c[0]) * (a[1] - b[1])) / 2;
        };

        double uv_area =
            triangle_area_2d(triangle_uv.row(0), triangle_uv.row(1), triangle_uv.row(2));
        if (uv_area < std::numeric_limits<double>::denorm_min()) {
            return 0;
        }
        q /= uv_area;

        const double u1 = triangle_uv(0, 0);
        const double v1 = triangle_uv(0, 1);
        const double u2 = triangle_uv(1, 0);
        const double v2 = triangle_uv(1, 1);
        const double u3 = triangle_uv(2, 0);
        const double v3 = triangle_uv(2, 1);
        const double denom = ((v2 - v3) * (u1 - u3) + (u3 - u2) * (v1 - v3));
        if (denom < std::numeric_limits<double>::denorm_min()) {
            // Degenerate triangle
            return 0.;
        }

        auto get_p_interpolated = [&](double u, double v) -> Eigen::Matrix<double, 3, 1> {
            auto lambda1 = ((v2 - v3) * (u - u3) + (u3 - u2) * (v - v3)) / denom;
            auto lambda2 = ((v3 - v1) * (u - u3) + (u1 - u3) * (v - v3)) / denom;
            auto lambda3 = 1 - lambda1 - lambda2;
            return (lambda1 * triangle_3d.row(0) + lambda2 * triangle_3d.row(1) +
                    lambda3 * triangle_3d.row(2))
                .transpose();
        };

        const int order = 2;
        Quadrature quadr;
        TriangleQuadrature::transformed_triangle_quadrature(order, triangle_uv, quadr);

        double ret = 0;
        for (size_t i = 0; i < quadr.size(); ++i) {
            double u = quadr.points()(i, 0);
            double v = quadr.points()(i, 1);
            Eigen::Matrix<double, 3, 1> p = get_p_interpolated(u, v);
            ret += q(p);
        }
    } else {
        // Quadric evaluated at the vertices
        double ret = 0;
        ret += q(triangle_3d.row(0).transpose());
        ret += q(triangle_3d.row(1).transpose());
        ret += q(triangle_3d.row(2).transpose());
        return ret;
    }
}

double AdaptiveTessellation::get_two_faces_quadrics_error_for_edge(const Tuple& e0) const
{
    double ret = 0.0;

    ret += get_quadric_error_for_face(e0);
    // interior
    if (e0.switch_face(*this).has_value()) {
        ret += get_quadric_error_for_face(e0.switch_face(*this).value());
    }
    // boundary
    // TODO

    if (is_seam_edge(e0)) {
        ret += get_quadric_error_for_face(get_oriented_mirror_edge(e0));
    }

    throw std::runtime_error("Not fully implemented, should not be used");

    return ret;
}

void AdaptiveTessellation::get_nminfo_for_vertex(const Tuple& v, wmtk::NewtonMethodInfo& nminfo)
    const
{
    auto is_inverted_coordinates = [](auto& A, auto& B, auto& primary_pos) {
        auto res = igl::predicates::orient2d(A, B, primary_pos);
        if (res != igl::predicates::Orientation::POSITIVE)
            return true;
        else
            return false;
    };
    std::vector<Tuple> one_ring_tris = get_one_ring_tris_for_vertex(v);
    nminfo.curve_id = vertex_attrs[v.vid(*this)].curve_id;
    nminfo.target_length = mesh_parameters.m_target_l;
    nminfo.neighbors.resize(one_ring_tris.size(), 4);
    nminfo.facet_ids.resize(one_ring_tris.size());
    for (auto i = 0; i < one_ring_tris.size(); i++) {
        const Tuple& tri = one_ring_tris[i];
        assert(!is_inverted(tri));
        std::array<Tuple, 3> local_tuples = oriented_tri_vertices(tri);
        nminfo.facet_ids[i] = tri.fid(*this);
        for (size_t j = 0; j < 3; j++) {
            if (local_tuples[j].vid(*this) == v.vid(*this)) {
                const Eigen::Vector2d& v2 = vertex_attrs[local_tuples[(j + 1) % 3].vid(*this)].pos;
                const Eigen::Vector2d& v3 = vertex_attrs[local_tuples[(j + 2) % 3].vid(*this)].pos;
                nminfo.neighbors.row(i) << v2(0), v2(1), v3(0), v3(1);
                assert(!is_inverted_coordinates(v2, v3, vertex_attrs[v.vid(*this)].pos));
                // sanity check. Should not be inverted
            }
        }
        assert(one_ring_tris.size() == nminfo.neighbors.rows());
    }
}
// do not include one-ring energy of the mirror vertex
// the curveid of the vertex is the curve id of any seam edge incident to this vertex
std::pair<double, Eigen::Vector2d> AdaptiveTessellation::get_one_ring_energy(const Tuple& v)
{
    std::vector<wmtk::TriMesh::Tuple> one_ring_tris = get_one_ring_tris_for_vertex(v);
    assert(one_ring_tris.size() > 0);
    wmtk::DofVector dofx;
    if (is_boundary_vertex(v) && mesh_parameters.m_boundary_parameter) {
        dofx.resize(1);
        dofx[0] = vertex_attrs[v.vid(*this)].t; // t
    } else {
        dofx.resize(2); // uv;
        dofx = vertex_attrs[v.vid(*this)].pos;
    }

    // assign curve id for vertex using the boundary edge of the one ring edges if it has one
    // (if it has more than one, it's ok to be the first one)
    for (auto& e : get_one_ring_edges_for_vertex(v)) {
        if (is_boundary_edge(e)) {
            vertex_attrs[v.vid(*this)].curve_id = edge_attrs[e.eid(*this)].curve_id.value();
            break;
        }
    }
    // infomation needed for newton's method
    // multiple nminfo for seam vertices
    std::vector<wmtk::NewtonMethodInfo> nminfos;
    // push in current vertex's nminfo
    wmtk::NewtonMethodInfo primary_nminfo;
    get_nminfo_for_vertex(v, primary_nminfo);
    nminfos.emplace_back(primary_nminfo);

    State state = {};
    state.dofx = dofx;
    wmtk::optimization_state_update(
        *mesh_parameters.m_energy,
        nminfos,
        mesh_parameters.m_boundary,
        state);

    return {state.value, state.gradient};
}

Eigen::VectorXd AdaptiveTessellation::get_quality_all_triangles()
{
    // Use a concurrent vector as for_each_face is parallel
    tbb::concurrent_vector<double> quality;
    quality.reserve(vertex_attrs.size());

    // Evaluate quality in parallel
    for_each_face([&](auto& f) { quality.push_back(get_quality(f)); });

    // Copy back in a VectorXd
    Eigen::VectorXd ret(quality.size());
    for (unsigned i = 0; i < quality.size(); ++i) ret[i] = quality[i];
    return ret;
}

bool AdaptiveTessellation::is_inverted(const Tuple& loc) const
{
    // Get the vertices ids
    auto vs = oriented_tri_vertices(loc);

    igl::predicates::exactinit();

    // Use igl for checking orientation
    auto res = igl::predicates::orient2d(
        vertex_attrs[vs[0].vid(*this)].pos,
        vertex_attrs[vs[1].vid(*this)].pos,
        vertex_attrs[vs[2].vid(*this)].pos);
    // The element is inverted if it not positive (i.e. it is negative or it is degenerate)
    return (res != igl::predicates::Orientation::POSITIVE);
}

void AdaptiveTessellation::mesh_improvement(int max_its)
{
    auto start_time = lagrange::get_timestamp();

    [[maybe_unused]] igl::Timer timer;
    [[maybe_unused]] double avg_len = 0.0;
    [[maybe_unused]] double pre_avg_len = 0.0;
    [[maybe_unused]] double pre_max_energy = -1.0;
    [[maybe_unused]] double old_average = 1e-4;
    wmtk::logger().info("target len {}", mesh_parameters.m_target_l);
    wmtk::logger().info("current length {}", avg_edge_len());
    // mesh_parameters.js_log["edge_length_avg_start"] = avg_edge_len();
    for (int it = 0; it < max_its; it++) {
        wmtk::logger().info("\n========it {}========", it);

        ///energy check
        wmtk::logger().info(
            "current max energy {} stop energy {}",
            mesh_parameters.m_max_energy,
            mesh_parameters.m_stop_energy);
        wmtk::logger().info("current length {}", avg_edge_len());

        split_all_edges();
        assert(invariants(get_faces()));
        auto split_finish_time = lagrange::get_timestamp();

        mesh_parameters.log(
            {{"iteration_" + std::to_string(it),
              {"split time", lagrange::timestamp_diff_in_seconds(start_time, split_finish_time)}}});

        // consolidate_mesh();

        if (!mesh_parameters.m_do_not_output) {
            write_obj_displaced(
                mesh_parameters.m_output_folder + "/after_split_" + std::to_string(it) + ".obj");
            displace_self_intersection_free(*this);
            write_obj(
                mesh_parameters.m_output_folder + "/after_split_" + std::to_string(it) +
                "3d_intersection_free.obj");
        }

        swap_all_edges();
        assert(invariants(get_faces()));
        auto swap_finish_time = lagrange::get_timestamp();
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["swap time"] =
            lagrange::timestamp_diff_in_seconds(split_finish_time, swap_finish_time);
        // consolidate_mesh();
        if (!mesh_parameters.m_do_not_output) {
            write_obj_displaced(
                mesh_parameters.m_output_folder + "/after_swap_" + std::to_string(it) + ".obj");
            write_obj_only_texture_coords(
                mesh_parameters.m_output_folder + "/after_swap_" + std::to_string(it) + "2d.obj");
        }
        collapse_all_edges();
        assert(invariants(get_faces()));
        // consolidate_mesh();
        auto collapse_finish_time = lagrange::get_timestamp();
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["collapse time"] =
            lagrange::timestamp_diff_in_seconds(swap_finish_time, collapse_finish_time);
        if (!mesh_parameters.m_do_not_output) {
            write_obj_displaced(
                mesh_parameters.m_output_folder + "/after_collapse_" + std::to_string(it) + ".obj");
            write_obj_only_texture_coords(
                mesh_parameters.m_output_folder + "/after_collapse_" + std::to_string(it) +
                "2d.obj");
        }

        smooth_all_vertices();
        assert(invariants(get_faces()));
        auto smooth_finish_time = lagrange::get_timestamp();

        mesh_parameters.log(
            {{"iteration_" + std::to_string(it),
              {"smooth time",
               lagrange::timestamp_diff_in_seconds(swap_finish_time, smooth_finish_time)}}});
        if (!mesh_parameters.m_do_not_output) {
            write_obj_displaced(
                mesh_parameters.m_output_folder + "/after_smooth_" + std::to_string(it) + ".obj");
            write_obj_only_texture_coords(
                mesh_parameters.m_output_folder + "/after_smooth_" + std::to_string(it) + "2d.obj");
        }

        auto avg_grad = (mesh_parameters.m_gradient / vert_capacity()).stableNorm();

        wmtk::logger().info(
            "++++++++v {} t {} avg gradient {}++++++++",
            vert_capacity(),
            tri_capacity(),
            avg_grad);

        mesh_parameters.js_log["iteration_" + std::to_string(it)]["avg_grad"] = avg_grad;
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["num_v"] = vert_capacity();
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["num_f"] = tri_capacity();
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["energy_max"] =
            mesh_parameters.m_max_energy;
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["edge_len_avg"] = avg_edge_len();
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["edge_len_target"] =
            mesh_parameters.m_target_l;
        if (avg_grad < 1e-4) {
            wmtk::logger().info(
                "!!!avg grad is less than 1e-4 !!! energy doesn't improve anymore. early "
                "stop"
                "itr {}, avg length {} ",
                it,
                avg_len);
            break;
        }
        mesh_parameters.m_gradient = Eigen::Vector2d(0., 0.);
        avg_len = avg_edge_len();
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["edge_len_avg_final"] = avg_len;
        if (abs(avg_grad - old_average) < 1e-4) break;
        old_average = avg_grad;
        if (mesh_parameters.m_target_l <= 0 &&
            mesh_parameters.m_max_energy < mesh_parameters.m_stop_energy) {
            break;
        }
        pre_avg_len = avg_len;
        pre_max_energy = mesh_parameters.m_max_energy;
        // consolidate_mesh();
        FrameMark;
    }

    wmtk::logger().info(
        "/////final: max energy {} , avg len {} ",
        mesh_parameters.m_max_energy,
        avg_len);
    // consolidate_mesh();
}
void AdaptiveTessellation::flatten_dofs(Eigen::VectorXd& v_flat)
{
    auto verts = get_vertices();
    v_flat.resize(verts.size() * 2);
    for (const auto& v : verts) {
        if (is_boundary_vertex(v) && mesh_parameters.m_boundary_parameter) {
            v_flat(v.vid(*this) * 2) = vertex_attrs[v.vid(*this)].t;
            v_flat(v.vid(*this) * 2 + 1) = std::numeric_limits<double>::infinity();
        } else {
            v_flat(v.vid(*this) * 2) = vertex_attrs[v.vid(*this)].pos(0);
            v_flat(v.vid(*this) * 2 + 1) = vertex_attrs[v.vid(*this)].pos(1);
        }
    }
}


bool AdaptiveTessellation::is_seam_edge(const TriMesh::Tuple& t) const
{
    return get_mirror_edge_opt(t).has_value();
}
bool AdaptiveTessellation::is_seam_vertex(const TriMesh::Tuple& t) const
{
    // it is seam vertex if it's incident to a seam edge
    for (auto& e : get_one_ring_edges_for_vertex(t)) {
        if (is_seam_edge(e)) return true;
    }
    return false;
}
bool AdaptiveTessellation::is_stitched_boundary_edge(const TriMesh::Tuple& t) const
{
    return is_boundary_edge(t) && !is_seam_edge(t);
}
bool AdaptiveTessellation::is_stitched_boundary_vertex(const TriMesh::Tuple& t) const
{
    // it is seam vertex if it's incident to a seam edge
    for (auto& e : get_one_ring_edges_for_vertex(t)) {
        if (is_stitched_boundary_edge(e)) return true;
    }
    return false;
}

void AdaptiveTessellation::set_mirror_edge_data(
    const TriMesh::Tuple& primary_t,
    const TriMesh::Tuple& mirror_edge)
{
    face_attrs[primary_t.fid(*this)].mirror_edges[primary_t.local_eid(*this)] =
        mirror_edge.is_ccw(*this) ? mirror_edge : mirror_edge.switch_vertex(*this);
}
std::optional<TriMesh::Tuple> AdaptiveTessellation::get_sibling_edge_opt(
    const TriMesh::Tuple& t) const
{
    if (is_boundary_edge(t)) {
        if (is_seam_edge(t)) {
            return get_oriented_mirror_edge(t);
        } else {
            return std::nullopt;
        }
    } else {
        assert(t.switch_face(*this).has_value());
        return t.switch_face(*this).value().switch_vertex(*this);
        // return the sibling edge that's of opposite diretion
    }
}
std::optional<TriMesh::Tuple> AdaptiveTessellation::get_mirror_edge_opt(
    const TriMesh::Tuple& t) const
{
    return face_attrs[t.fid(*this)].mirror_edges[t.local_eid(*this)];
}

// given a seam edge retrieve its mirror edge in opposite direction (half egde conventions )
TriMesh::Tuple AdaptiveTessellation::get_oriented_mirror_edge(const TriMesh::Tuple& t) const
{
    assert(t.is_valid(*this));
    assert(is_seam_edge(t));
    TriMesh::Tuple mirror_edge = get_mirror_edge_opt(t).value();
    assert(is_seam_edge(mirror_edge));
    TriMesh::Tuple primary_edge =
        face_attrs[mirror_edge.fid(*this)].mirror_edges[mirror_edge.local_eid(*this)].value();
    if (primary_edge.is_ccw(*this) == t.is_ccw(*this)) {
        return mirror_edge;
    } else {
        return mirror_edge.switch_vertex(*this);
    }
}

// given a seam edge with vid v retrieve the correpsonding vertex on the mirror edge
TriMesh::Tuple AdaptiveTessellation::get_mirror_vertex(const TriMesh::Tuple& vertex) const
{
    const TriMesh::Tuple& edge = vertex; // This code implicitly treats the edge as a vertex
    assert(is_seam_edge(edge));
    TriMesh::Tuple mirror_edge = get_oriented_mirror_edge(edge);
    return mirror_edge.switch_vertex(*this);
}

// return a vector of mirror vertices. store v itself at index 0 of the returned vector
// assume no operation has made fixed vertices outdated
// TODO maybe delete?
std::vector<TriMesh::Tuple> AdaptiveTessellation::get_all_mirror_vertices(
    const TriMesh::Tuple& v) const
{
    assert(is_seam_vertex(v));
    std::vector<TriMesh::Tuple> ret_vertices;
    // always put v itself at index 0
    ret_vertices.emplace_back(v);
    if (vertex_attrs[v.vid(*this)].fixed) {
        // use the same color vids to initiate Tuples
        auto same_color_uv_indices = color_to_uv_indices.at(uv_index_to_color.at(v.vid(*this)));
        for (auto mirror_vid : same_color_uv_indices) {
            ret_vertices.emplace_back(tuple_from_vertex(mirror_vid));
        }
    } else if (is_seam_edge(v)) {
        ret_vertices.emplace_back(get_mirror_vertex(v));
    } else {
        for (Tuple edge : get_one_ring_edges_for_vertex(v)) {
            edge = edge.switch_vertex(*this);
            if (is_seam_edge(edge)) {
                ret_vertices.emplace_back(get_mirror_vertex(edge));
            }
        }
    }
    auto lt = [&](const Tuple& a, const Tuple& b) { return a.vid(*this) < b.vid(*this); };
    auto eq = [&](const Tuple& a, const Tuple& b) { return a.vid(*this) == b.vid(*this); };
    std::sort(ret_vertices.begin(), ret_vertices.end(), lt);
    ret_vertices.erase(
        std::unique(ret_vertices.begin(), ret_vertices.end(), eq),
        ret_vertices.end());
    return ret_vertices;
}

// get all mirror_vids using navigation
std::vector<size_t> AdaptiveTessellation::get_all_mirror_vids(const TriMesh::Tuple& v) const
{
    std::vector<size_t> ret_vertices_vid;
    std::queue<TriMesh::Tuple> queue;

    ret_vertices_vid.emplace_back(v.vid(*this));

    for (auto& e : get_one_ring_edges_for_vertex(v)) queue.push(e);
    while (!queue.empty()) {
        auto e = queue.front();
        queue.pop();
        if (is_seam_edge(e)) {
            auto mirror_v = get_mirror_vertex(e.switch_vertex(*this));
            if (std::find(ret_vertices_vid.begin(), ret_vertices_vid.end(), mirror_v.vid(*this)) ==
                ret_vertices_vid.end()) {
                ret_vertices_vid.emplace_back(mirror_v.vid(*this));
                for (auto& new_e : get_one_ring_edges_for_vertex(mirror_v)) queue.push(new_e);
            }
        }
    }
    return ret_vertices_vid;
}

void AdaptiveTessellation::assign_edge_curveid()
{
    for (const auto& e : get_edges()) {
        assert(e.is_valid(*this));
        if (!is_boundary_edge(e)) {
            // is an interior edge, skip ....
            continue;
        }
        // find the mid-point uv of the edge
        auto midpoint_uv =
            (vertex_attrs[e.vid(*this)].pos + vertex_attrs[e.switch_vertex(*this).vid(*this)].pos) /
            2.;
        // use the mid-point uv to find edge curve id
        int curve_id = -1;
        double t = 0.;
        std::tie(curve_id, t) = mesh_parameters.m_boundary.uv_to_t(midpoint_uv);
        // assign the curve id to the edge
        edge_attrs[e.eid(*this)].curve_id = std::make_optional<int>(curve_id);
    }
}

auto AdaptiveTessellation::get_one_ring_tris_accross_seams_for_vertex(const Tuple& vertex) const
    -> std::vector<Tuple>
{
    if (!is_seam_vertex(vertex)) {
        return get_one_ring_tris_for_vertex(vertex);
    } else {
        std::vector<Tuple> tris;
        for (const auto& vert : get_all_mirror_vertices(vertex)) {
            const auto ring = get_one_ring_tris_for_vertex(vert);
            tris.insert(tris.end(), ring.begin(), ring.end());
        }
        auto lt = [&](const Tuple& a, const Tuple& b) { return a.fid(*this) < b.fid(*this); };
        auto eq = [&](const Tuple& a, const Tuple& b) { return a.fid(*this) == b.fid(*this); };
        std::sort(tris.begin(), tris.end(), lt);
        tris.erase(std::unique(tris.begin(), tris.end(), eq), tris.end());
        return tris;
    }
}


void AdaptiveTessellation::update_energy_cache(const std::vector<Tuple>& tris)
{
    // update the face_attrs (accuracy error)
    if (!mesh_parameters.m_ignore_embedding) {
        // get a vector of new traingles uvs
        if (mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
            std::vector<std::array<float, 6>> modified_tris_uv(tris.size());
            for (int i = 0; i < tris.size(); i++) {
                auto tri = tris[i];
                auto verts = oriented_tri_vids(tri);
                std::array<float, 6> tri_uv;
                for (int i = 0; i < 3; i++) {
                    tri_uv[i * 2] = vertex_attrs[verts[i]].pos(0);
                    tri_uv[i * 2 + 1] = vertex_attrs[verts[i]].pos(1);
                }
                modified_tris_uv[i] = tri_uv;
            }
            std::vector<float> renewed_errors(tris.size());
            m_texture_integral.get_error_per_triangle(modified_tris_uv, renewed_errors);
            set_faces_cached_distance_integral(tris, renewed_errors);
        } else if (mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
            std::vector<wmtk::Quadric<double>> compressed_quadrics(tris.size());
            m_quadric_integral.get_quadric_per_triangle(
                tris.size(),
                [&](int f) -> std::array<float, 6> {
                    // Get triangle uv positions
                    const std::array<Tuple, 3> local_tuples = oriented_tri_vertices(tris[f]);
                    const Eigen::Vector2f p0 =
                        vertex_attrs[local_tuples[0].vid(*this)].pos.cast<float>();
                    const Eigen::Vector2f p1 =
                        vertex_attrs[local_tuples[1].vid(*this)].pos.cast<float>();
                    const Eigen::Vector2f p2 =
                        vertex_attrs[local_tuples[2].vid(*this)].pos.cast<float>();
                    return {p0.x(), p0.y(), p1.x(), p1.y(), p2.x(), p2.y()};
                },
                compressed_quadrics);
            set_faces_quadrics(tris, compressed_quadrics);
        }
    }
}
} // namespace adaptive_tessellation
