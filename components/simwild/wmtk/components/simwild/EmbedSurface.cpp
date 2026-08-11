#include "EmbedSurface.hpp"
#include <wmtk/utils/DelaunayBoxMesh.hpp>
#include <wmtk/utils/EmbedTriangles.hpp>

// clang-format off
#include <igl/remove_unreferenced.h>
#include <igl/tet_tet_adjacency.h>
#include <igl/read_triangle_mesh.h>
#include <igl/winding_number.h>
// igl must be included BEFORE VolumeRemesher
#include <VolumeRemesher/embed.h>
#include <VolumeRemesher/numerics.h>
// clang-format on

#include <wmtk/utils/VectorUtils.h>
#include <bitset>
#include <filesystem>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/InsertTriangleUtils.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/WindingNumber.hpp>
#include <wmtk/utils/io.hpp>

#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>

namespace wmtk::components::simwild {

void delaunay_box_mesh(
    const wmtk::SampleEnvelope& envelope,
    const MatrixXd& vertices,
    std::vector<wmtk::delaunay::Point3D>& points,
    std::vector<wmtk::delaunay::Tetrahedron>& tets,
    Vector3d& box_min,
    Vector3d& box_max)
{
    assert(vertices.cols() == 3);

    const Vector3d vertices_max = vertices.colwise().maxCoeff();
    const Vector3d vertices_min = vertices.colwise().minCoeff();

    ///points for delaunay
    points.resize(vertices.rows());
    for (int i = 0; i < vertices.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            points[i][j] = vertices(i, j);
        }
    }

    // Unlike tetwild, the box is grown from the bounding box of the very vertices handed in.
    wmtk::utils::delaunay_box_mesh(
        envelope,
        vertices_min,
        vertices_max,
        (vertices_max - vertices_min).norm(),
        points,
        tets,
        box_min,
        box_max);
}

void embed_surface(
    const MatrixXd& V_surface,
    const MatrixXi& F_surface,
    const MatrixXd& V_vol,
    const MatrixXi& T_vol,
    MatrixXr& V_emb,
    MatrixXi& T_emb,
    MatrixXi& F_on_surface,
    const bool perform_sanity_checks)
{
    // old output
    std::vector<std::array<size_t, 3>> facets_after;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets_after;
    std::vector<bool> tet_face_on_input_surface;
    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> polygon_faces;

    ////
    std::vector<double> tet_vrt_coord(V_vol.size());
    std::vector<uint32_t> tet_indices(T_vol.size());

    for (int i = 0; i < V_vol.rows(); ++i) {
        tet_vrt_coord[3 * i + 0] = V_vol(i, 0);
        tet_vrt_coord[3 * i + 1] = V_vol(i, 1);
        tet_vrt_coord[3 * i + 2] = V_vol(i, 2);
    }

    for (int i = 0; i < T_vol.rows(); ++i) {
        tet_indices[4 * i + 0] = T_vol(i, 0);
        tet_indices[4 * i + 1] = T_vol(i, 1);
        tet_indices[4 * i + 2] = T_vol(i, 2);
        tet_indices[4 * i + 3] = T_vol(i, 3);
    }

    // Step 2 (cont.): flatten the input surface the same way.
    // prepare input surfaces info
    std::vector<double> tri_vrt_coord(V_surface.size());
    std::vector<uint32_t> triangle_indices(F_surface.size());

    for (int i = 0; i < V_surface.rows(); ++i) {
        tri_vrt_coord[3 * i + 0] = V_surface(i, 0);
        tri_vrt_coord[3 * i + 1] = V_surface(i, 1);
        tri_vrt_coord[3 * i + 2] = V_surface(i, 2);
    }

    for (int i = 0; i < F_surface.rows(); ++i) {
        triangle_indices[3 * i + 0] = (uint32_t)F_surface(i, 0);
        triangle_indices[3 * i + 1] = (uint32_t)F_surface(i, 1);
        triangle_indices[3 * i + 2] = (uint32_t)F_surface(i, 2);
    }

    // Steps 3-5: the exact arrangement and everything that has to happen to its output --
    // bigrational conversion, facet decoding, surface tracking, vertex compaction and the
    // per-tet face-flag reorder. Shared with tetwild, which does the same from its own
    // vector-of-Vector3d inputs.
    std::vector<bool> polygon_faces_on_input;
    wmtk::utils::EmbedTrianglesOptions opts;
    opts.check_orientation = perform_sanity_checks;
    wmtk::utils::embed_triangles_in_tets(
        tri_vrt_coord,
        triangle_indices,
        tet_vrt_coord,
        tet_indices,
        v_rational,
        polygon_faces,
        polygon_faces_on_input,
        is_v_on_input,
        tets_after,
        tet_face_on_input_surface,
        opts);

    T_emb.resize(tets_after.size(), 4);
    for (int i = 0; i < tets_after.size(); ++i) {
        const auto& t = tets_after[i];
        T_emb.row(i) = Vector4i(t[0], t[1], t[2], t[3]);
    }

    V_emb.resize(v_rational.size(), 3);
    for (int i = 0; i < v_rational.size(); ++i) {
        V_emb.row(i) = v_rational[i];
    }

    size_t n_face_on_input = 0;
    for (const bool b : polygon_faces_on_input) {
        if (b) {
            ++n_face_on_input;
        }
    }
    F_on_surface.resize(n_face_on_input, 3);
    n_face_on_input = 0;
    for (int i = 0; i < polygon_faces.size(); ++i) {
        if (!polygon_faces_on_input[i]) {
            continue;
        }
        const auto& f = polygon_faces[i];
        F_on_surface.row(n_face_on_input++) = Vector3i(f[0], f[1], f[2]);
    }
}


EmbedSurface::EmbedSurface(
    const std::vector<std::string>& img_filenames,
    const std::vector<Matrix4d>& img_transform,
    const double tol_rel,
    const double tol_abs)
    : m_img_filenames(img_filenames)
{
    assert(img_filenames.size() == img_transform.size());

    Vs.resize(m_img_filenames.size());
    Fs.resize(m_img_filenames.size());

    for (size_t i = 0; i < m_img_filenames.size(); ++i) {
        if (!std::filesystem::exists(m_img_filenames[i])) {
            log_and_throw_error("Input file {} does not exist", m_img_filenames[i]);
        }
        MatrixXi F_single;
        io::read_triangle_mesh(m_img_filenames[i], Vs[i], F_single, tol_rel, tol_abs);

        assert(Vs[i].cols() == 3);
        assert(F_single.cols() == 3);

        // apply transform to Vs[i]
        for (size_t j = 0; j < Vs[i].rows(); ++j) {
            Vector3d v = Vs[i].row(j);
            Vector4d x = to_homogenuous(v);
            x = img_transform[i] * x;
            Vs[i].row(j) = from_homogenuous(x);
        }

        Fs[i] = F_single;

        const size_t nV_old = m_V_surface.rows();
        const size_t nF_old = m_F_surface.rows();

        m_V_surface.conservativeResize(m_V_surface.rows() + Vs[i].rows(), 3);
        m_V_surface.block(nV_old, 0, Vs[i].rows(), 3) = Vs[i];

        F_single.array() += nV_old;
        m_F_surface.conservativeResize(m_F_surface.rows() + F_single.rows(), 3);
        m_F_surface.block(nF_old, 0, Fs[i].rows(), 3) = F_single;
    }


    // process triangle soup
    MatrixXd V;
    MatrixXi F;
    VectorXi _I;

    // remove unreferenced vertices
    igl::remove_unreferenced(m_V_surface, m_F_surface, V, F, _I);

    if (V.rows() == 0 || F.rows() == 0) {
        log_and_throw_error("Empty Input");
    }

    wmtk::logger().info("All inputs #V = {}, #F = {}", V.rows(), F.rows());

    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    VF_to_vectors(V, F, verts, tris);

    V_surf_from_vector(verts);
    F_surf_from_vector(tris);
}

void EmbedSurface::simplify_surface(const double eps, const int num_threads)
{
    // convert to STL vectors
    std::vector<Eigen::Vector3d> verts = V_surf_to_vector();
    std::vector<std::array<size_t, 3>> tris = F_surf_to_vector();

    shortest_edge_collapse::ShortestEdgeCollapse surf_mesh(verts, num_threads, false);

    // must be a small envelope to ensure correct tet tags later on
    surf_mesh.create_mesh(verts.size(), tris, modified_nonmanifold_v, eps);
    assert(surf_mesh.check_mesh_connectivity_validity());

    surf_mesh.collapse_shortest(0);
    surf_mesh.consolidate_mesh();
    // surf_mesh.write_triangle_mesh("triangle_soup_coarse.off");

    //// get the simplified input
    std::vector<Eigen::Vector3d> v_simplified;
    std::vector<std::array<size_t, 3>> f_simplified;
    v_simplified.resize(surf_mesh.vert_capacity());
    f_simplified.resize(surf_mesh.tri_capacity());
    for (const auto& t : surf_mesh.get_vertices()) {
        const size_t i = t.vid(surf_mesh);
        v_simplified[i] = surf_mesh.vertex_attrs[i].pos;
    }

    for (const auto& t : surf_mesh.get_faces()) {
        const auto i = t.fid(surf_mesh);
        const auto vs = surf_mesh.oriented_tri_vids(t);
        for (int j = 0; j < 3; j++) {
            f_simplified[i][j] = vs[j];
        }
    }

    V_surf_from_vector(v_simplified);
    F_surf_from_vector(f_simplified);
}

void EmbedSurface::remove_duplicates(const double eps)
{
    auto v = V_surf_to_vector();
    auto f = F_surf_to_vector();

    wmtk::remove_duplicates(v, f, eps);

    V_surf_from_vector(v);
    F_surf_from_vector(f);
}

bool EmbedSurface::embed_surface(const bool flood_fill)
{
    logger().info("Embed with VolumeInsertion");

    double eps = 0.5;
    if (!Fs.empty()) {
        const std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax =
            std::pair(m_V_surface.colwise().minCoeff(), m_V_surface.colwise().maxCoeff());
        double diag = (box_minmax.first - box_minmax.second).norm();
        eps = 1e-2 * diag;
    }

    std::shared_ptr<SampleEnvelope> ptr_env;
    {
        const auto v_simplified = V_surf_to_vector();

        std::vector<Eigen::Vector3i> tempF(m_F_surface.rows());
        for (size_t i = 0; i < tempF.size(); ++i) {
            tempF[i] = m_F_surface.row(i);
        }
        ptr_env = std::make_shared<SampleEnvelope>();
        ptr_env->use_exact = true;
        ptr_env->init(v_simplified, tempF, eps);
    }

    std::vector<wmtk::delaunay::Point3D> points;
    std::vector<wmtk::delaunay::Tetrahedron> tets;
    Vector3d box_min;
    Vector3d box_max;
    delaunay_box_mesh(*ptr_env, m_V_surface, points, tets, box_min, box_max);

    MatrixXd V_vol;
    V_vol.resize(points.size(), 3);
    for (int i = 0; i < points.size(); ++i) {
        const auto& v = points[i];
        V_vol.row(i) = Vector3d(v[0], v[1], v[2]);
    }

    MatrixXi T_vol;
    T_vol.resize(tets.size(), 4);
    for (int i = 0; i < tets.size(); ++i) {
        const auto& t = tets[i];
        T_vol.row(i) = Vector4i(t[0], t[1], t[2], t[3]);
    }

    simwild::embed_surface(
        m_V_surface,
        m_F_surface,
        V_vol,
        T_vol,
        m_V_emb_r,
        m_T_emb,
        m_F_on_surface,
        m_perform_sanity_checks);

    if (m_perform_sanity_checks) {
        // check that all tets contain valid vertex indices and no duplicates
        logger().info("Check for duplicate vertices...");
        for (int i = 0; i < m_T_emb.rows(); ++i) {
            const Vector4i& t = m_T_emb.row(i);
            std::set<int> s{t[0], t[1], t[2], t[3]};
            if (s.size() != 4) {
                log_and_throw_error("Tet {} has duplicate vertices: {}", i, t);
            }
            for (int j = 0; j < 4; ++j) {
                if (t[j] < 0 || t[j] >= m_V_emb_r.rows()) {
                    log_and_throw_error(
                        "Tet {} has invalid vertex index {} (out of range [0, {})",
                        i,
                        t[j],
                        m_V_emb_r.rows());
                }
            }
        }

        // check that all surface faces contain valid vertex indices and no duplicates
        for (int i = 0; i < m_F_on_surface.rows(); ++i) {
            const Vector3i& f = m_F_on_surface.row(i);
            std::set<int> s{f[0], f[1], f[2]};
            if (s.size() != 3) {
                log_and_throw_error("Surface face {} has duplicate vertices: {}", i, f);
            }
            for (int j = 0; j < 3; ++j) {
                if (f[j] < 0 || f[j] >= m_V_emb_r.rows()) {
                    log_and_throw_error(
                        "Surface face {} has invalid vertex index {} (out of range [0, {})",
                        i,
                        f[j],
                        m_V_emb_r.rows());
                }
            }
        }
        logger().info("done");

        // // check that all vertices are used in at least one tet
        // logger().info("Check that all vertices are used in at least one tet...");
        // std::vector<bool> vertex_used(m_V_emb_r.rows(), false);
        // for (int i = 0; i < m_T_emb.rows(); ++i) {
        //     const Vector4i& t = m_T_emb.row(i);
        //     for (int j = 0; j < 4; ++j) {
        //         vertex_used[t[j]] = true;
        //     }
        // }

        // for (int i = 0; i < vertex_used.size(); ++i) {
        //     if (!vertex_used[i]) {
        //         log_and_throw_error("Vertex {} is not used in any tet", i);
        //     }
        // }
        // logger().info("done");

        // check for degenerate tets (zero or negative volume)
        logger().info("Check for degenerate tets...");
        for (int i = 0; i < m_T_emb.rows(); ++i) {
            const Vector4i& t = m_T_emb.row(i);
            const Vector3r& v0 = m_V_emb_r.row(t[0]);
            const Vector3r& v1 = m_V_emb_r.row(t[1]);
            const Vector3r& v2 = m_V_emb_r.row(t[2]);
            const Vector3r& v3 = m_V_emb_r.row(t[3]);
            Vector3r v0v1 = v1 - v0;
            Vector3r v0v2 = v2 - v0;
            Vector3r v0v3 = v3 - v0;
            if ((v0v1.cross(v0v2)).dot(v0v3) <= 0) {
                log_and_throw_error(
                    "Tet {} is degenerate (zero or negative volume): vids = {}",
                    i,
                    t);
            }
        }
        logger().info("done");

        // check that each tet appears only once in the tet list
        logger().info("Check for duplicate tets...");
        std::set<std::set<int>> tet_set;
        for (int i = 0; i < m_T_emb.rows(); ++i) {
            const Vector4i& t = m_T_emb.row(i);
            std::set<int> s{t[0], t[1], t[2], t[3]};
            if (tet_set.count(s) > 0) {
                log_and_throw_error("Tet {} is a duplicate: {}", i, t);
            }
            tet_set.insert(s);
        }
        logger().info("done");

        // check that faces appear at most twice in the tet list (once for each adjacent tet)
        logger().info("Check for duplicate faces...");
        std::map<std::set<int>, int> face_count;
        for (int i = 0; i < m_T_emb.rows(); ++i) {
            const Vector4i& t = m_T_emb.row(i);
            std::array<std::set<int>, 4> faces = {{
                std::set<int>{t[0], t[1], t[2]},
                std::set<int>{t[0], t[1], t[3]},
                std::set<int>{t[1], t[2], t[3]},
                std::set<int>{t[2], t[0], t[3]},
            }};
            for (const auto& f : faces) {
                if (face_count.count(f) == 0) {
                    face_count[f] = 0;
                }
                face_count[f]++;
                if (face_count[f] > 2) {
                    log_and_throw_error("Face {} appears more than twice in the tet list", f);
                }
            }
        }
        logger().info("done");
    }

    // check for consistent orientation of tets
    logger().info("Check for consistent orientation between tets...");
    std::set<std::array<int, 3>> face_set; // each face must be unique
    for (int i = 0; i < m_T_emb.rows(); ++i) {
        const Vector4i& t = m_T_emb.row(i);
        std::array<std::array<int, 3>, 4> faces = {{
            {{t[1], t[3], t[2]}}, // opposite t[0]
            {{t[0], t[2], t[3]}}, // opposite t[1]
            {{t[0], t[3], t[1]}}, // opposite t[2]
            {{t[0], t[1], t[2]}}, // opposite t[3]
        }};
        // Rotate vertex IDs in faces to start with the lowest vertex ID. This ensures that the
        // same face is represented by the same set of vertex IDs, regardless of the order in
        // which they appear in the tet.
        for (auto& f : faces) {
            std::rotate(f.begin(), std::min_element(f.begin(), f.end()), f.end());
        }

        for (const auto& f : faces) {
            if (face_set.count(f) > 0) {
                logger().warn("Face {} appears more than once in the tet list", f);
            }
            face_set.insert(f);
        }
    }
    logger().info("done");

    // const bool all_rounded = VF_rational_to_double(m_V_emb_r, m_T_emb, m_V_emb);
    // if (!all_rounded) {
    //     // log_and_throw_error("Tets are inverted after converting to double precision.");
    //     logger().info("Not all vertices can be rounded to double precision.");
    // }

    m_V_emb.resizeLike(m_V_emb_r);
    for (int i = 0; i < m_V_emb_r.size(); ++i) {
        m_V_emb(i) = m_V_emb_r(i).to_double();
    }
    /**
     * Only do a trivial rounding here. The more complex rounding is performed later on.
     */
    bool all_rounded = true;
    for (int i = 0; i < m_V_emb.rows(); ++i) {
        const Vector3r& r = m_V_emb_r.row(i);
        const Vector3r r_from_d = to_rational(Vector3d(m_V_emb.row(i)));
        if (r != r_from_d) {
            all_rounded = false;
            break;
        }
    }

    // add tags
    if (Fs.empty()) {
        log_and_throw_error("No input surface to embed");
    }
    tag_from_winding_number();

    /**
     * Cluster tags by flood-filling regions that are bounded by the surface. All tags within one
     * region are unified by taking the tag with most occurances.
     *
     * This is for several reasons not the best way to do this.
     * 1. The tags should be volume-weighted so that the tag that represents the most volume should
     * be picked.
     * 2. We should not rely on geometric look-up at all, but use the information stored in
     * m_F_tags_surface. This is more work so I took a shortcut here.
     */
    if (flood_fill) {
        logger().info("Use flood fill to unify tags.");
        std::set<simplex::Face> surface;
        for (size_t i = 0; i < m_F_on_surface.rows(); ++i) {
            const simplex::Face f(m_F_on_surface(i, 0), m_F_on_surface(i, 1), m_F_on_surface(i, 2));
            surface.insert(f);
        }

        // flood fill regions
        std::vector<bool> visited(m_T_emb.rows(), false);
        MatrixXi TT;
        igl::tet_tet_adjacency(m_T_emb, TT);

        for (size_t i = 0; i < m_T_emb.rows(); ++i) {
            if (visited[i]) {
                continue;
            }

            std::queue<size_t> q;
            q.push(i);
            visited[i] = true;

            std::vector<size_t> region; // all tets in that region

            while (!q.empty()) {
                size_t tid = q.front();
                q.pop();

                region.push_back(tid);
                const auto& tet = m_T_emb.row(tid);

                // face-vertex IDs according to igl::tet_tet_adjacency
                std::array<simplex::Face, 4> f{{
                    simplex::Face(tet[0], tet[1], tet[2]),
                    simplex::Face(tet[0], tet[1], tet[3]),
                    simplex::Face(tet[1], tet[2], tet[3]),
                    simplex::Face(tet[2], tet[0], tet[3]),
                }};
                for (size_t j = 0; j < 4; ++j) {
                    if (TT(tid, j) < 0) {
                        continue; // boundary
                    }
                    if (surface.count(f[j]) == 0 && !visited[TT(tid, j)]) {
                        visited[TT(tid, j)] = true;
                        q.push(TT(tid, j));
                    }
                }
            }

            // get all tags for one image
            for (size_t img_id = 0; img_id < Fs.size(); ++img_id) {
                std::map<int, int> m;
                for (size_t j = 0; j < region.size(); ++j) {
                    int tag = m_T_tags.coeff(region[j], img_id);
                    if (m.count(tag) == 0) {
                        m[tag] = 1;
                    } else {
                        m[tag] += 1;
                    }
                }
                int max_count = -1;
                int max_tag = -1;
                for (const auto& [tag, count] : m) {
                    if (count > max_count) {
                        max_count = count;
                        max_tag = tag;
                    }
                }
                assert(max_count > 0);
                for (size_t j = 0; j < region.size(); ++j) {
                    if (m_T_tags.coeff(region[j], img_id) != max_tag) {
                        m_T_tags.coeffRef(region[j], img_id) = max_tag;
                    }
                }
            }
        }
    }

    return all_rounded;
}

void EmbedSurface::consolidate()
{
    std::map<size_t, size_t> old2new;
    std::map<size_t, size_t> new2old;
    size_t new_vid_counter = 0;
    for (size_t i = 0; i < m_T_emb.rows(); ++i) {
        for (size_t j = 0; j < 4; ++j) {
            const auto vid = m_T_emb(i, j);
            if (old2new.count(vid) == 0) {
                old2new[vid] = new_vid_counter;
                new2old[new_vid_counter] = vid;
                ++new_vid_counter;
            }
        }
    }

    MatrixXd V;
    MatrixXr Vr;
    V.resize(new_vid_counter, 3);
    Vr.resize(new_vid_counter, 3);
    for (size_t i = 0; i < new_vid_counter; ++i) {
        V.row(i) = m_V_emb.row(new2old[i]);
        Vr.row(i) = m_V_emb_r.row(new2old[i]);
    }

    MatrixXi T;
    T.resizeLike(m_T_emb);
    for (size_t i = 0; i < m_T_emb.rows(); ++i) {
        for (size_t j = 0; j < 4; ++j) {
            T(i, j) = old2new[m_T_emb(i, j)];
        }
    }

    MatrixXi F_surf;
    F_surf.resizeLike(m_F_on_surface);
    for (size_t i = 0; i < m_F_on_surface.rows(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            F_surf(i, j) = old2new[m_F_on_surface(i, j)];
        }
    }

    m_V_emb = V;
    m_V_emb_r = Vr;
    m_T_emb = T;
    m_F_on_surface = F_surf;
}

void EmbedSurface::write_surf_off(const std::string& filename) const
{
    igl::writeOFF(filename, m_V_surface, m_F_surface);
}

void EmbedSurface::write_emb_surf_off(const std::string& filename) const
{
    igl::writeOFF(filename, m_V_emb, m_F_on_surface);
}

void EmbedSurface::write_emb_msh(const std::string& filename) const
{
    wmtk::MshData msh;
    msh.add_tet_vertices(m_V_emb.rows(), [this](size_t k) -> Vector3d { return m_V_emb.row(k); });

    const size_t n_tet_vertices = m_V_emb.rows();

    msh.add_tets(m_T_emb.rows(), [this](size_t k) { return m_T_emb.row(k); });

    for (size_t i = 0; i < m_T_tags.cols(); ++i) {
        msh.add_tet_attribute<1>(fmt::format("tag_{}", i), [this, i](size_t j) {
            return m_T_tags.coeff(j, i);
        });
    }

    msh.add_physical_group("ImageVolume");

    msh.add_face_vertices();
    msh.add_faces(m_F_on_surface.rows(), [this](size_t k) { return m_F_on_surface.row(k); });
    msh.add_physical_group("EmbeddedSurface");

    msh.add_face_vertices(m_V_surface.rows(), [this](size_t k) { return m_V_surface.row(k); });
    msh.add_faces(m_F_surface.rows(), [this](size_t k) { return m_F_surface.row(k); });
    msh.add_physical_group("EnvelopeSurface");

    msh.save(filename, true);
}

void EmbedSurface::write_emb_vtu(const std::string& filename) const
{
    paraviewo::VTUWriter writer;
    for (size_t i = 0; i < m_T_tags.cols(); ++i) {
        writer.add_cell_field(fmt::format("tag_{}", i), m_T_tags.col(i).cast<double>());
    }
    logger().info("Write {}.vtu and _surf.vtu", filename);
    writer.write_mesh(filename + ".vtu", m_V_emb, m_T_emb, paraviewo::CellType::Tetrahedron);
    writer
        .write_mesh(filename + "_surf.vtu", m_V_emb, m_F_on_surface, paraviewo::CellType::Triangle);
}

std::pair<Vector3d, Vector3d> EmbedSurface::bbox_minmax() const
{
    const Vector3d bbox_max = m_V_emb.colwise().maxCoeff();
    const Vector3d bbox_min = m_V_emb.colwise().minCoeff();

    return std::make_pair(bbox_min, bbox_max);
}

std::pair<Vector3d, Vector3d> EmbedSurface::bbox_surf_minmax() const
{
    const Vector3d bbox_max = m_V_surface.colwise().maxCoeff();
    const Vector3d bbox_min = m_V_surface.colwise().minCoeff();

    return std::make_pair(bbox_min, bbox_max);
}

std::vector<Eigen::Vector3d> EmbedSurface::V_surf_to_vector() const
{
    std::vector<Eigen::Vector3d> verts;

    verts.resize(m_V_surface.rows());
    for (size_t i = 0; i < m_V_surface.rows(); ++i) {
        verts[i] = m_V_surface.row(i);
    }

    return verts;
}

std::vector<std::array<size_t, 3>> EmbedSurface::F_surf_to_vector() const
{
    std::vector<std::array<size_t, 3>> tris;

    tris.resize(m_F_surface.rows());
    for (size_t i = 0; i < m_F_surface.rows(); ++i) {
        tris[i][0] = m_F_surface(i, 0);
        tris[i][1] = m_F_surface(i, 1);
        tris[i][2] = m_F_surface(i, 2);
    }

    return tris;
}

void EmbedSurface::V_surf_from_vector(const std::vector<Eigen::Vector3d>& verts)
{
    const V_MAP V_surface(verts[0].data(), verts.size(), 3);
    m_V_surface = V_surface;
    assert(m_V_surface.rows() == verts.size());
    assert(m_V_surface.cols() == 3);
}

void EmbedSurface::F_surf_from_vector(const std::vector<std::array<size_t, 3>>& tris)
{
    m_F_surface.resize(tris.size(), 3);
    for (size_t i = 0; i < tris.size(); ++i) {
        m_F_surface(i, 0) = tris[i][0];
        m_F_surface(i, 1) = tris[i][1];
        m_F_surface(i, 2) = tris[i][2];
    }
}

void EmbedSurface::tag_from_winding_number()
{
    m_T_tags.resize(m_T_emb.rows(), Fs.size());
    m_T_tags.setZero();
    m_tags.resize(m_T_emb.rows());

    MatrixXd P;
    P.resize(m_T_emb.rows(), 3);
    for (size_t i = 0; i < m_T_emb.rows(); ++i) {
        const Vector3d v0 = m_V_emb.row(m_T_emb(i, 0));
        const Vector3d v1 = m_V_emb.row(m_T_emb(i, 1));
        const Vector3d v2 = m_V_emb.row(m_T_emb(i, 2));
        const Vector3d v3 = m_V_emb.row(m_T_emb(i, 3));
        P.row(i) = (v0 + v1 + v2 + v3) * 0.25;
    }

    VectorXd W;
    W.resize(m_T_emb.rows());
    for (size_t i = 0; i < Fs.size(); ++i) {
        // igl::winding_number(Vs[i], Fs[i], P, W);
        utils::winding_number(Vs[i], Fs[i], P, W, m_num_threads);
        assert(W.size() == m_T_tags.rows());
        for (size_t j = 0; j < W.size(); ++j) {
            if (W[j] < 0.5) {
                continue;
            }
            m_T_tags.coeffRef(j, i) = 1;
            m_tags[j].insert((int64_t)i);
        }
    }
    m_T_tags.makeCompressed();
}

} // namespace wmtk::components::simwild