#include "InterfaceSelection.hpp"

#include "Hdf5Writers.hpp"
#include "PythonFormat.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <unordered_map>

namespace wmtk::components::polyfem_ops {

namespace {

/// The orientation tests below decide the winding of every OBJ face from the sign of a dot
/// product, so they have to round exactly the way numpy does or the two engines can disagree on a
/// face. numpy evaluates a cross product as three independent multiply-subtracts and a length-3
/// dot as a sequential sum, with a rounding after every operation; the named temporaries here
/// keep the compiler from fusing any of them into an FMA, which would round differently.
struct Vec3
{
    double x = 0, y = 0, z = 0;
};

Vec3 sub3(const Vec3& a, const Vec3& b)
{
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 cross3(const Vec3& a, const Vec3& b)
{
    const double p0 = a.y * b.z;
    const double p1 = a.z * b.y;
    const double p2 = a.z * b.x;
    const double p3 = a.x * b.z;
    const double p4 = a.x * b.y;
    const double p5 = a.y * b.x;
    return Vec3{p0 - p1, p2 - p3, p4 - p5};
}

double dot3(const Vec3& a, const Vec3& b)
{
    const double p0 = a.x * b.x;
    const double p1 = a.y * b.y;
    const double p2 = a.z * b.z;
    const double s = p0 + p1;
    return s + p2;
}

double dot2(double ax, double ay, double bx, double by)
{
    const double p0 = ax * bx;
    const double p1 = ay * by;
    return p0 + p1;
}

Vec3 row3(const MatrixXd& m, int64_t i)
{
    return Vec3{m(i, 0), m(i, 1), m(i, 2)};
}

Vec3 to_vec3(const VectorXd& v)
{
    return Vec3{v(0), v(1), v.size() > 2 ? v(2) : 0.0};
}

std::array<int64_t, 2> undirected(int64_t a, int64_t b)
{
    return {std::min(a, b), std::max(a, b)};
}

/**
 * @brief Append `face` to `faces3d` (3D) or `edges2d` (2D), wound so that its normal points along
 * `outward`.
 *
 * Both interface passes below decide the winding this way and differ only in where `outward`
 * comes from: the two incident cell centroids for an explicit selection, the incident physical
 * tags for the legacy auto-detect. In 3D the face normal itself is tested against `outward`; in 2D
 * it is the edge's LEFT normal against `-outward`, which is the legacy GCP convention (the left
 * normal points toward the region).
 *
 * `-outward` is a negation rather than the opposite subtraction, which is what the selection pass
 * used to write: IEEE negation is exact and round-to-nearest is symmetric about zero, so
 * `-(outside - inside)` and `inside - outside` are the same double except for the sign of a zero,
 * and a signed zero cannot change a `< 0` test or the sign of the sum it enters.
 */
void push_oriented(
    const TaggedMesh& mesh,
    const std::vector<int64_t>& face,
    const Vec3& outward,
    std::vector<std::array<int64_t, 3>>& faces3d,
    std::vector<std::array<int64_t, 2>>& edges2d)
{
    const int64_t a = face[0];
    const int64_t b = face[1];
    if (mesh.mesh_dim == 3) {
        const int64_t c = face[2];
        const Vec3 pa = row3(mesh.coords, a);
        const Vec3 n = cross3(sub3(row3(mesh.coords, b), pa), sub3(row3(mesh.coords, c), pa));
        if (dot3(n, outward) < 0) {
            faces3d.push_back({a, c, b});
        } else {
            faces3d.push_back({a, b, c});
        }
    } else {
        const double tx = mesh.coords(b, 0) - mesh.coords(a, 0);
        const double ty = mesh.coords(b, 1) - mesh.coords(a, 1);
        // left normal of the edge tangent
        if (dot2(-ty, tx, -outward.x, -outward.y) < 0) {
            edges2d.push_back({b, a});
        } else {
            edges2d.push_back({a, b});
        }
    }
}

/// (oriented_faces_3d, oriented_edges_2d, tags_rows) for explicit selections. Mirrors
/// `constraints._selected_interfaces`. Orientation: 3D normals point out of the region; 2D
/// left-normals point toward the region (legacy GCP convention).
void selected_interfaces(
    const TaggedMesh& mesh,
    const std::vector<Selection>& selections,
    std::vector<std::array<int64_t, 3>>& faces3d,
    std::vector<std::array<int64_t, 2>>& edges2d,
    std::vector<std::vector<int64_t>>& tags)
{
    std::vector<Selection> unique;
    std::vector<int64_t> ids_per_input;
    nlohmann::json as_json = nlohmann::json::array();
    for (const auto& s : selections) {
        nlohmann::json j = {{"region", s.region}};
        if (s.filter) j["filter"] = *s.filter;
        if (s.id) j["id"] = *s.id;
        as_json.push_back(j);
    }
    assign_selection_ids(as_json, unique, ids_per_input);
    const auto records = select_boundary_faces(mesh, unique);

    for (const auto& r : records) {
        const Vec3 inside = to_vec3(mesh.centroid(r.a_prim));
        const Vec3 outside = to_vec3(mesh.centroid(r.b_prim));
        push_oriented(mesh, r.face, sub3(outside, inside), faces3d, edges2d);
        tags.push_back(r.ids);
    }
}

/// Legacy auto-detect: every face whose incident physical-tag multiset is not a same-tag interior
/// pair -- material interfaces plus one-sided boundary skins. Orientation: the high physical tag
/// is the inside. Mirrors `constraints._auto_interfaces`.
void auto_interfaces(
    const TaggedMesh& mesh,
    std::vector<std::array<int64_t, 3>>& faces3d,
    std::vector<std::array<int64_t, 2>>& edges2d,
    std::vector<std::vector<int64_t>>& tags)
{
    for (size_t fk = 0; fk < mesh.num_faces(); ++fk) {
        std::vector<std::pair<int64_t, Vec3>> incidents;
        for (const int64_t p : mesh.face_to_prims[fk]) {
            const Vec3 c = to_vec3(mesh.centroid(p));
            for (const auto& n : mesh.prim_tags[p]) { // std::set iterates sorted, like sorted()
                incidents.emplace_back(mesh.names.at(n), c);
            }
        }
        std::set<int64_t> unique;
        for (const auto& [t, c] : incidents) unique.insert(t);
        if (incidents.size() == 2 && unique.size() == 1) continue;
        if (incidents.size() == 1 && incidents[0].first == 0) continue;

        const auto& fr = mesh.face_repr[fk];
        Vec3 outward;
        if (incidents.size() == 1) {
            const Vec3 inside = incidents[0].second;
            Vec3 mid;
            for (const int64_t v : fr) {
                mid.x += mesh.coords(v, 0);
                mid.y += mesh.coords(v, 1);
                if (mesh.mesh_dim == 3) mid.z += mesh.coords(v, 2);
            }
            const double n = static_cast<double>(fr.size());
            mid = Vec3{mid.x / n, mid.y / n, mid.z / n};
            outward = sub3(mid, inside);
        } else {
            const int64_t hi = *unique.rbegin();
            const int64_t lo = *unique.begin();
            const auto mean_of = [&incidents](int64_t tag) {
                Vec3 sum;
                double n = 0;
                for (const auto& [t, c] : incidents) {
                    if (t != tag) continue;
                    sum.x += c.x;
                    sum.y += c.y;
                    sum.z += c.z;
                    n += 1.0;
                }
                return Vec3{sum.x / n, sum.y / n, sum.z / n};
            };
            outward = sub3(mean_of(lo), mean_of(hi));
        }

        push_oriented(mesh, fr, outward, faces3d, edges2d);
        tags.emplace_back(unique.begin(), unique.end());
    }
}

/// The collision proxy's own vertex list and its edge list over those local indices. Mirrors the
/// `collision_node_ids` / `collision_edges_local` pair `constraints.load_mesh` builds the same way
/// in both of its branches: the proxy vertices are the sorted vertex set of the primitives the OBJ
/// writes (the faces in 3D, the edges in 2D), and every interface edge is then re-indexed against
/// them.
void build_collision_proxy(const std::set<int64_t>& proxy_verts, LoadedMesh& out)
{
    out.collision_node_ids.assign(proxy_verts.begin(), proxy_verts.end()); // std::set == sorted()
    std::unordered_map<int64_t, int64_t> g2l;
    for (size_t i = 0; i < out.collision_node_ids.size(); ++i) {
        g2l[out.collision_node_ids[i]] = static_cast<int64_t>(i);
    }
    for (const auto& e : out.interface_edges) {
        out.collision_edges_local.push_back({g2l[e[0]], g2l[e[1]]});
    }
}

} // namespace

// ---------------------------------------------------------------------------
// 2D loop orientation
// ---------------------------------------------------------------------------

std::vector<std::array<int64_t, 2>> orient_edge_loops_2d(
    const std::vector<std::array<int64_t, 2>>& oriented_edges)
{
    if (oriented_edges.empty()) return oriented_edges;

    // Undirected adjacency, keys in first-appearance order (Python: an insertion-ordered
    // defaultdict, and the component emission order follows its key order).
    std::vector<int64_t> adj_order;
    std::unordered_map<int64_t, std::set<int64_t>> adjacency;
    std::unordered_map<int64_t, std::vector<size_t>> comp_edge_indices;
    const auto touch = [&](int64_t v) {
        if (adjacency.find(v) == adjacency.end()) {
            adjacency.emplace(v, std::set<int64_t>());
            adj_order.push_back(v);
        }
    };
    for (size_t i = 0; i < oriented_edges.size(); ++i) {
        const int64_t a = oriented_edges[i][0];
        const int64_t b = oriented_edges[i][1];
        touch(a);
        adjacency[a].insert(b);
        touch(b);
        adjacency[b].insert(a);
        comp_edge_indices[a].push_back(i);
        comp_edge_indices[b].push_back(i);
    }

    std::set<int64_t> visited;
    std::vector<std::array<int64_t, 2>> result;

    for (const int64_t seed : adj_order) {
        if (visited.count(seed) != 0) continue;

        std::vector<int64_t> stack{seed};
        std::vector<int64_t> comp_vertices;
        while (!stack.empty()) {
            const int64_t v = stack.back();
            stack.pop_back();
            if (visited.count(v) != 0) continue;
            visited.insert(v);
            comp_vertices.push_back(v);
            for (const int64_t nb : adjacency[v]) {
                if (visited.count(nb) == 0) stack.push_back(nb);
            }
        }

        const std::set<int64_t> comp_vertex_set(comp_vertices.begin(), comp_vertices.end());
        std::set<size_t> comp_edges_idx;
        for (const int64_t v : comp_vertices) {
            const auto& idxs = comp_edge_indices[v];
            comp_edges_idx.insert(idxs.begin(), idxs.end());
        }

        bool all_degree_two = true;
        for (const int64_t v : comp_vertices) {
            size_t deg = 0;
            for (const int64_t nb : adjacency[v]) {
                if (comp_vertex_set.count(nb) != 0) ++deg;
            }
            if (deg != 2) {
                all_degree_two = false;
                break;
            }
        }
        const bool is_simple_loop = comp_edges_idx.size() >= 3 && all_degree_two;

        if (!is_simple_loop) {
            // Preserve original orientation/order for non-loop components.
            for (size_t i = 0; i < oriented_edges.size(); ++i) {
                if (comp_edges_idx.count(i) != 0) result.push_back(oriented_edges[i]);
            }
            continue;
        }

        // Reconstruct loop order (deterministic seed/neighbor choice).
        const int64_t start = *std::min_element(comp_vertices.begin(), comp_vertices.end());
        const auto& start_nbs = adjacency[start];
        int64_t curr = *start_nbs.begin();
        int64_t prev = start;
        std::vector<int64_t> loop{start, curr};
        while (curr != start) {
            const auto& neigh = adjacency[curr];
            auto it = neigh.begin();
            const int64_t n0 = *it;
            ++it;
            const int64_t nxt = n0 != prev ? n0 : *it;
            prev = curr;
            curr = nxt;
            if (curr != start) loop.push_back(curr);
        }

        std::set<std::array<int64_t, 2>> directed;
        for (const size_t i : comp_edges_idx) directed.insert(oriented_edges[i]);
        size_t agree = 0;
        size_t disagree = 0;
        for (size_t i = 0; i < loop.size(); ++i) {
            const int64_t a = loop[i];
            const int64_t b = loop[(i + 1) % loop.size()];
            if (directed.count({a, b}) != 0) ++agree;
            if (directed.count({b, a}) != 0) ++disagree;
        }
        if (agree != 0 && disagree != 0) {
            log_and_throw_error(
                "interface loop through vertex {} has contradictory orientations ({} edges one "
                "way, {} the other): the same interface was selected from both sides, so no "
                "single orientation is correct -- drop one side of the selection",
                start,
                agree,
                disagree);
        }
        if (disagree != 0) std::reverse(loop.begin(), loop.end());

        for (size_t i = 0; i < loop.size(); ++i) {
            result.push_back({loop[i], loop[(i + 1) % loop.size()]});
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// load_mesh
// ---------------------------------------------------------------------------

LoadedMesh load_mesh(const std::string& msh_path, const std::vector<Selection>& selections)
{
    const TaggedMesh mesh(msh_path);

    std::vector<std::array<int64_t, 3>> faces3d;
    std::vector<std::array<int64_t, 2>> edges2d;
    std::vector<std::vector<int64_t>> face_tags;
    if (!selections.empty()) {
        selected_interfaces(mesh, selections, faces3d, edges2d, face_tags);
    } else {
        auto_interfaces(mesh, faces3d, edges2d, face_tags);
    }

    LoadedMesh out;
    out.node_tag_to_idx = mesh.node_tag_to_idx;
    out.coords = mesh.coords;
    out.total_n_nodes = mesh.total_n_nodes;
    out.mesh_dim = mesh.mesh_dim;

    if (mesh.mesh_dim == 3) {
        out.interface_faces = faces3d;
        std::set<std::array<int64_t, 2>> edge_set;
        for (const auto& f : out.interface_faces) {
            edge_set.insert(undirected(f[0], f[1]));
            edge_set.insert(undirected(f[1], f[2]));
            edge_set.insert(undirected(f[2], f[0]));
        }
        out.interface_edges.assign(edge_set.begin(), edge_set.end()); // std::set == sorted()
        out.face_tags = face_tags;
        if (!out.interface_faces.empty()) {
            std::set<int64_t> verts;
            for (const auto& f : out.interface_faces) verts.insert(f.begin(), f.end());
            build_collision_proxy(verts, out);
        }
    } else {
        // The loop pass preserves the explicit per-edge orientation and throws on contradictions;
        // union id rows per undirected edge (multi-interface edges keep every id).
        std::map<std::array<int64_t, 2>, std::vector<int64_t>> edge_to_tags;
        for (size_t i = 0; i < edges2d.size(); ++i) {
            const auto key = undirected(edges2d[i][0], edges2d[i][1]);
            auto it = edge_to_tags.find(key);
            if (it == edge_to_tags.end()) {
                edge_to_tags.emplace(key, face_tags[i]);
            } else {
                std::set<int64_t> merged(it->second.begin(), it->second.end());
                merged.insert(face_tags[i].begin(), face_tags[i].end());
                it->second.assign(merged.begin(), merged.end());
            }
        }
        out.interface_edges = orient_edge_loops_2d(edges2d);
        out.face_tags.clear();
        for (const auto& e : out.interface_edges) {
            out.face_tags.push_back(edge_to_tags.at(undirected(e[0], e[1])));
        }
        if (!out.interface_edges.empty()) {
            std::set<int64_t> verts;
            for (const auto& e : out.interface_edges) verts.insert(e.begin(), e.end());
            build_collision_proxy(verts, out);
        }
    }

    return out;
}

// ---------------------------------------------------------------------------
// Writers
// ---------------------------------------------------------------------------

CollisionObj collision_mesh_obj(
    const MatrixXd& coords,
    const std::vector<int64_t>& node_ids,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    const std::vector<std::array<int64_t, 3>>& interface_faces,
    const std::vector<int64_t>& collision_node_ids,
    const std::vector<std::array<int64_t, 2>>& collision_edges_local)
{
    const std::vector<int64_t>& verts = collision_node_ids.empty() ? node_ids : collision_node_ids;
    std::unordered_map<int64_t, int64_t> idx_map;
    for (size_t i = 0; i < verts.size(); ++i) {
        idx_map[verts[i]] = static_cast<int64_t>(i);
    }

    CollisionObj obj;
    for (const int64_t nid : verts) {
        const double x = coords(nid, 0);
        const double y = coords(nid, 1);
        const double z = coords.cols() == 2 ? 0.0 : coords(nid, 2);
        obj.vertices.push_back({x, y, z});
    }

    if (!interface_faces.empty()) {
        for (const auto& t : interface_faces) {
            if (idx_map.count(t[0]) != 0 && idx_map.count(t[1]) != 0 &&
                idx_map.count(t[2]) != 0) {
                obj.faces.push_back({idx_map[t[0]], idx_map[t[1]], idx_map[t[2]]});
            }
        }
    } else if (!collision_edges_local.empty()) {
        obj.edges = collision_edges_local;
    } else {
        for (const auto& e : interface_edges) {
            if (idx_map.count(e[0]) != 0 && idx_map.count(e[1]) != 0) {
                obj.edges.push_back({idx_map[e[0]], idx_map[e[1]]});
            }
        }
    }
    return obj;
}

void write_collision_mesh_obj(const std::string& path, const CollisionObj& obj)
{
    std::ofstream f(path, std::ios::binary); // binary: no CRLF translation, the bytes must match
    if (!f) {
        log_and_throw_error("Cannot open {} for writing", path);
    }
    f << "# Interface collision mesh\n";
    for (const auto& [x, y, z] : obj.vertices) {
        f << "v " << python_repr(x) << " " << python_repr(y) << " " << python_repr(z) << "\n";
    }
    // OBJ is 1-based
    for (const auto& t : obj.faces) {
        f << "f " << t[0] + 1 << " " << t[1] + 1 << " " << t[2] + 1 << "\n";
    }
    for (const auto& e : obj.edges) {
        f << "l " << e[0] + 1 << " " << e[1] + 1 << "\n";
    }
    if (!obj.faces.empty()) {
        logger().info(
            "  collision  : {}  ({} verts, {} faces)",
            path,
            obj.vertices.size(),
            obj.faces.size());
    } else {
        logger().info(
            "  collision  : {}  ({} verts, {} edges)",
            path,
            obj.vertices.size(),
            obj.edges.size());
    }
}

void write_collision_body_ids_txt(
    const std::string& path,
    const std::vector<std::vector<int64_t>>& face_tags)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        log_and_throw_error("Cannot open {} for writing", path);
    }
    for (const auto& tags : face_tags) {
        for (size_t i = 0; i < tags.size(); ++i) {
            if (i != 0) f << " ";
            f << tags[i];
        }
        f << "\n";
    }
    logger().info("  body IDs   : {}  ({} faces)", path, face_tags.size());
}

// ---------------------------------------------------------------------------
// Collision pairs and the entry point
// ---------------------------------------------------------------------------

void normalize_collision_pairs(
    const nlohmann::json& raw_pairs,
    std::vector<Selection>& unique,
    std::vector<std::array<int64_t, 2>>& polyfem_pairs)
{
    for (const auto& pair : raw_pairs) {
        if (!pair.is_array() || pair.size() != 2) {
            log_and_throw_error(
                "collision_pairs entries must be [side_A, side_B], got {}",
                pair.dump());
        }
    }
    nlohmann::json flat = nlohmann::json::array();
    for (const auto& pair : raw_pairs) {
        for (const auto& side : pair) flat.push_back(side);
    }
    std::vector<int64_t> ids;
    assign_selection_ids(flat, unique, ids);

    polyfem_pairs.clear();
    std::set<std::array<int64_t, 2>> seen;
    for (size_t i = 0; i < raw_pairs.size(); ++i) {
        const std::array<int64_t, 2> pair{ids[2 * i], ids[2 * i + 1]};
        const std::array<int64_t, 2> key{
            std::min(pair[0], pair[1]),
            std::max(pair[0], pair[1])};
        if (seen.insert(key).second) polyfem_pairs.push_back(pair);
    }
}

InterfaceConstraint make_interface_constraint(
    const std::string& mesh_path,
    const std::vector<Selection>& selections,
    bool use_graph,
    bool normalize,
    double scale,
    bool smooth_positions)
{
    logger().info("Reading {} ...", mesh_path);
    const LoadedMesh m = load_mesh(mesh_path, selections);

    if (m.interface_edges.empty()) {
        log_and_throw_error(
            "no interface edges extracted -- check selections and mesh physical groups");
    }

    std::set<int64_t> node_id_set;
    for (const auto& e : m.interface_edges) node_id_set.insert(e.begin(), e.end());
    const std::vector<int64_t> node_ids(node_id_set.begin(), node_id_set.end());
    logger().info(
        "Found {} interface edges, {} interface nodes",
        m.interface_edges.size(),
        node_ids.size());

    InterfaceConstraint out;
    // `dim = mesh_dim`: the Python's `dim` override is never passed by an engine.
    out.fitting = fitting_constraint(
        node_ids,
        m.mesh_dim,
        m.coords,
        m.interface_edges,
        use_graph,
        normalize,
        m.interface_faces);
    out.laplacian = laplacian_constraint(
        node_ids,
        m.coords,
        m.interface_edges,
        use_graph,
        scale,
        normalize,
        m.interface_faces,
        smooth_positions);
    out.collision_mesh = collision_mesh_obj(
        m.coords,
        node_ids,
        m.interface_edges,
        m.interface_faces,
        m.collision_node_ids,
        m.collision_edges_local);
    // The proxy's own vertex list when there is one, falling back to the interface nodes -- the
    // two differ when a selected face has a vertex no selected EDGE reaches.
    out.linear_map =
        linear_map(m.collision_node_ids.empty() ? node_ids : m.collision_node_ids, m.total_n_nodes);
    out.collision_body_ids = m.face_tags;
    return out;
}

} // namespace wmtk::components::polyfem_ops
