#include "background_pyramids.hpp"
#include "junction_protection.hpp"
#include "prism_jacobian.hpp"
#include "prismatic_mesh.hpp"
#include "rod_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/predicates.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
using Tri = std::array<size_t, 3>;
using Tet = std::array<size_t, 4>;
using Quad = std::array<size_t, 4>;
using Decomposition = std::vector<Tet>;

template <typename T>
T sorted(T x)
{
    std::sort(x.begin(), x.end());
    return x;
}

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error("prism dominant: " + message);
}

Tet source_tet(const PrismaticMeshInput& input, size_t tid)
{
    require(tid < static_cast<size_t>(input.tetrahedra.rows()), "invalid source tetrahedron ID");
    Tet t;
    for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(tid, j);
    return t;
}

std::array<Tri, 4> tet_faces(const Tet& t)
{
    return {{{t[1], t[2], t[3]}, {t[0], t[3], t[2]}, {t[0], t[1], t[3]}, {t[0], t[2], t[1]}}};
}

std::vector<size_t> canonical_cycle(std::vector<size_t> cycle)
{
    std::rotate(cycle.begin(), std::min_element(cycle.begin(), cycle.end()), cycle.end());
    return cycle;
}

bool same_orientation(const Tri& a, const Tri& b)
{
    return canonical_cycle({a.begin(), a.end()}) == canonical_cycle({b.begin(), b.end()});
}

// The six permutations are the six corner-only, three-tet triangulations of a prism.
// Orient each template on the reference prism, never repair signs using deformed geometry.
std::vector<Decomposition> decompositions(const HybridCell& cell)
{
    const auto& v = cell.vertices;
    std::vector<Decomposition> result;
    if (cell.type == HybridCellType::Tetrahedron) {
        require(v.size() == 4, "tetrahedron needs four corners");
        return {{{v[0], v[1], v[2], v[3]}}};
    }
    if (cell.type == HybridCellType::Pyramid) {
        require(v.size() == 5, "pyramid needs five corners");
        return {
            {{v[0], v[1], v[2], v[4]}, {v[0], v[2], v[3], v[4]}},
            {{v[0], v[1], v[3], v[4]}, {v[1], v[2], v[3], v[4]}}};
    }
    require(cell.type == HybridCellType::Prism && v.size() == 6, "prism needs six corners");
    std::array<size_t, 3> p = {0, 1, 2};
    do {
        int inversions = 0;
        for (int i = 0; i < 3; ++i)
            for (int j = i + 1; j < 3; ++j) inversions += p[i] > p[j];
        const auto a = p[0], b = p[1], c = p[2];
        Decomposition d = {
            {v[a], v[b], v[c], v[c + 3]},
            {v[a], v[b], v[c + 3], v[b + 3]},
            {v[a], v[a + 3], v[b + 3], v[c + 3]}};
        if (inversions % 2)
            for (auto& t : d) std::swap(t[0], t[1]);
        result.push_back(std::move(d));
    } while (std::next_permutation(p.begin(), p.end()));
    return result;
}

bool matches_reference(const PrismaticMeshInput& input, const HybridCell& cell)
{
    std::set<Tet> actual;
    for (size_t tid : cell.source_tets) actual.insert(sorted(source_tet(input, tid)));
    if (actual.size() != cell.source_tets.size()) return false;
    for (const auto& d : decompositions(cell)) {
        std::set<Tet> expected;
        for (const auto& t : d) expected.insert(sorted(t));
        if (actual == expected) return true;
    }
    return false;
}

bool positive_decompositions(const PrismaticMeshInput& input, const HybridCell& cell, double floor)
{
    for (const auto& d : decompositions(cell))
        for (const auto& t : d)
            if (!tet_volume_above_threshold(input.vertices, t, floor)) return false;
    return true;
}

// Six-node wedge: x(r,s,t) = (1-t)((1-r-s)p0 + r*p1 + s*p2)
//                          + t*((1-r-s)p3 + r*p4 + s*p5).
// Sample corners, edge midpoints and centroid on the bottom, middle and top sections.
// This is a sampled Jacobian check, not a certificate over the whole reference wedge.
constexpr std::array<std::array<double, 2>, 7> jacobian_triangle_points = {
    {{0, 0}, {1, 0}, {0, 1}, {0.5, 0}, {0.5, 0.5}, {0, 0.5}, {1.0 / 3, 1.0 / 3}}};
constexpr std::array<double, 3> jacobian_height_points = {0, 0.5, 1};

struct SampledJacobian
{
    bool positive = true;
    long double minimum = std::numeric_limits<long double>::infinity();
};

SampledJacobian prism_jacobians(const PrismaticMeshInput& input, const HybridCell& cell)
{
    require(cell.type == HybridCellType::Prism && cell.vertices.size() == 6, "invalid prism");
    // Subtract after promotion, and use differences rather than absolute positions.
    std::array<std::array<long double, 3>, 6> p;
    for (size_t i = 0; i < p.size(); ++i)
        for (int j = 0; j < 3; ++j) p[i][j] = input.vertices(cell.vertices[i], j);
    SampledJacobian result;
    for (long double t : jacobian_height_points) {
        std::array<long double, 3> dr, ds;
        for (int j = 0; j < 3; ++j) {
            dr[j] = (1 - t) * (p[1][j] - p[0][j]) + t * (p[4][j] - p[3][j]);
            ds[j] = (1 - t) * (p[2][j] - p[0][j]) + t * (p[5][j] - p[3][j]);
        }
        for (const auto& rs : jacobian_triangle_points) {
            const long double r = rs[0], s = rs[1];
            std::array<long double, 3> dt;
            for (int j = 0; j < 3; ++j)
                dt[j] = (1 - r - s) * (p[3][j] - p[0][j]) + r * (p[4][j] - p[1][j]) +
                        s * (p[5][j] - p[2][j]);
            const long double determinant = dr[0] * (ds[1] * dt[2] - ds[2] * dt[1]) -
                                            dr[1] * (ds[0] * dt[2] - ds[2] * dt[0]) +
                                            dr[2] * (ds[0] * dt[1] - ds[1] * dt[0]);
            result.positive &= std::isfinite(determinant) && determinant > 0;
            result.minimum = std::min(result.minimum, determinant);
        }
    }
    return result;
}

std::vector<std::vector<size_t>> cell_faces(const HybridCell& cell)
{
    const auto& v = cell.vertices;
    if (cell.type == HybridCellType::Tetrahedron) {
        std::vector<std::vector<size_t>> result;
        for (const auto& f : tet_faces({v[0], v[1], v[2], v[3]}))
            result.emplace_back(f.begin(), f.end());
        return result;
    }
    if (cell.type == HybridCellType::Pyramid) {
        return {
            {v[0], v[3], v[2], v[1]},
            {v[0], v[1], v[4]},
            {v[1], v[2], v[4]},
            {v[2], v[3], v[4]},
            {v[3], v[0], v[4]}};
    }
    return {
        {v[0], v[2], v[1]},
        {v[3], v[4], v[5]},
        {v[0], v[1], v[4], v[3]},
        {v[1], v[2], v[5], v[4]},
        {v[2], v[0], v[3], v[5]}};
}

struct SourceFace
{
    std::vector<size_t> tets;
    std::vector<Tri> oriented;
    size_t band_count = 0;
};
using SourceFaces = std::map<Tri, SourceFace>;

SourceFaces source_faces(const PrismaticMeshInput& input)
{
    SourceFaces result;
    for (size_t tid = 0; tid < static_cast<size_t>(input.tetrahedra.rows()); ++tid) {
        const auto tet = source_tet(input, tid);
        require(tet_volume_above_threshold(input.vertices, tet, 0), "source tet is not positive");
        for (const auto& f : tet_faces(tet)) {
            auto& face = result[sorted(f)];
            face.tets.push_back(tid);
            face.oriented.push_back(f);
            face.band_count += input.offset_tet_tags.at(tid) == 1;
            require(face.tets.size() <= 2, "source mesh has a nonmanifold face");
            if (face.tets.size() == 2)
                require(
                    !same_orientation(f, face.oriented[0]),
                    "source face orientations disagree");
        }
    }
    return result;
}

struct QuadPatch
{
    Quad vertices; // sorted key; triangulation is inherited from the source mesh
    std::array<Tri, 2> triangles;
};

// Certify cell boundaries against a reference split. Shared nonplanar quadrilaterals
// must select the same triangulation on both sides.
std::vector<QuadPatch> patches(const HybridCell& cell, const Decomposition& split)
{
    std::map<Tri, std::vector<Tri>> all;
    for (const auto& t : split)
        for (const auto& f : tet_faces(t)) all[sorted(f)].push_back(f);
    std::map<Tri, Tri> boundary;
    for (const auto& [key, fs] : all) {
        require(fs.size() <= 2, "overlapping cell partition");
        if (fs.size() == 1)
            boundary[key] = fs[0];
        else
            require(!same_orientation(fs[0], fs[1]), "inconsistent internal cell face");
    }
    std::set<Tri> used;
    std::vector<QuadPatch> result;
    for (const auto& f : cell_faces(cell)) {
        if (f.size() == 3) {
            const Tri t = {f[0], f[1], f[2]};
            const auto it = boundary.find(sorted(t));
            require(
                it != boundary.end() && same_orientation(t, it->second),
                "invalid triangular cell boundary");
            require(used.insert(it->first).second, "duplicate cell boundary triangle");
            continue;
        }
        const std::array<std::array<Tri, 2>, 2> diagonals = {
            {{{{f[0], f[1], f[2]}, {f[0], f[2], f[3]}}},
             {{{f[0], f[1], f[3]}, {f[1], f[2], f[3]}}}}};
        bool found = false;
        for (const auto& pair : diagonals) {
            const auto a = boundary.find(sorted(pair[0])), b = boundary.find(sorted(pair[1]));
            if (a == boundary.end() || b == boundary.end()) continue;
            require(
                same_orientation(pair[0], a->second) && same_orientation(pair[1], b->second),
                "invalid quadrilateral orientation");
            require(
                used.insert(a->first).second && used.insert(b->first).second,
                "duplicate quadrilateral coverage");
            result.push_back(
                {sorted(Quad{f[0], f[1], f[2], f[3]}),
                 sorted(std::array<Tri, 2>{a->first, b->first})});
            found = true;
            break;
        }
        require(found, "cell quadrilateral does not match source triangulation");
    }
    require(used.size() == boundary.size(), "cell boundary leaves source triangles uncovered");
    return result;
}

std::vector<QuadPatch> patches(const PrismaticMeshInput& input, const HybridCell& cell)
{
    Decomposition split;
    for (size_t tid : cell.source_tets) split.push_back(source_tet(input, tid));
    return patches(cell, split);
}

std::set<Tet> tet_keys(const Decomposition& split)
{
    std::set<Tet> keys;
    for (const auto& t : split) keys.insert(sorted(t));
    return keys;
}

HybridCell retained_tet(const PrismaticMeshInput& input, size_t tid, int64_t candidate = -1)
{
    const auto t = source_tet(input, tid);
    return {HybridCellType::Tetrahedron, {t.begin(), t.end()}, {tid}, candidate};
}

struct PyramidOption
{
    HybridCell cell;
    int side;
    unsigned splits = 0;
    bool positive = false;
    size_t worst_base_corner = 0;
    bool jacobian_positive = false;
};

size_t worst_pyramid_corner(const PrismaticMeshInput& input, const HybridCell& pyramid)
{
    const std::array<Tet, 4> corners = {{{0, 1, 3, 4}, {0, 1, 2, 4}, {1, 2, 3, 4}, {0, 2, 3, 4}}};
    long double minimum = std::numeric_limits<long double>::infinity();
    size_t worst = 0;
    for (size_t i = 0; i < corners.size(); ++i) {
        std::array<std::array<long double, 3>, 3> d;
        for (size_t j = 0; j < 3; ++j)
            for (size_t k = 0; k < 3; ++k)
                d[j][k] = static_cast<long double>(
                              input.vertices(pyramid.vertices[corners[i][j + 1]], k)) -
                          input.vertices(pyramid.vertices[corners[i][0]], k);
        const long double determinant = d[0][0] * (d[1][1] * d[2][2] - d[1][2] * d[2][1]) -
                                        d[0][1] * (d[1][0] * d[2][2] - d[1][2] * d[2][0]) +
                                        d[0][2] * (d[1][0] * d[2][1] - d[1][1] * d[2][0]);
        if (determinant < minimum) {
            minimum = determinant;
            worst = i;
        }
    }
    return worst;
}

// A diagonal change on a strictly convex, exactly planar exterior quad preserves
// the geometric boundary. Nonplanar patches and interfaces to fixed cells stay fixed.
bool planar_convex_quad(const PrismaticMeshInput& input, const std::vector<size_t>& cycle)
{
    using namespace wmtk::utils::predicates;
    std::array<Vector3d, 4> p;
    for (size_t i = 0; i < 4; ++i) p[i] = input.vertices.row(cycle[i]);
    if (orient3d(p[0], p[1], p[2], p[3]) != Orientation::COPLANAR) return false;
    for (int dim = 0; dim < 3; ++dim) {
        Orientation sign = Orientation::COLLINEAR;
        bool convex = true;
        for (size_t i = 0; i < 4; ++i) {
            const auto project = [&](size_t j) {
                return Eigen::Vector2d(p[j][dim], p[j][(dim + 1) % 3]);
            };
            const auto next = orient2d(project(i), project((i + 1) % 4), project((i + 2) % 4));
            if (next == Orientation::COLLINEAR || (i && next != sign)) convex = false;
            sign = next;
        }
        if (convex) return true;
    }
    return false;
}

using Boundary = std::map<Tri, Tri>;

void check_boundary_reconstruction(
    Boundary before,
    Boundary after,
    const std::set<Quad>& flexible_exterior_quads)
{
    for (const auto& quad : flexible_exterior_quads) {
        const auto remove_patch = [&](Boundary& boundary) {
            std::map<std::array<size_t, 2>, int> edges;
            size_t triangles = 0;
            for (auto it = boundary.begin(); it != boundary.end();) {
                if (!std::all_of(it->first.begin(), it->first.end(), [&](size_t v) {
                        return std::binary_search(quad.begin(), quad.end(), v);
                    })) {
                    ++it;
                    continue;
                }
                ++triangles;
                const auto t = it->second;
                for (size_t j = 0; j < 3; ++j) {
                    const size_t a = t[j], b = t[(j + 1) % 3];
                    edges[sorted(std::array<size_t, 2>{a, b})] += a < b ? 1 : -1;
                }
                it = boundary.erase(it);
            }
            require(triangles == 2, "exterior quad lost its two-triangle coverage");
            for (auto it = edges.begin(); it != edges.end();)
                if (!it->second)
                    it = edges.erase(it);
                else {
                    require(std::abs(it->second) == 1, "invalid exterior patch orientation");
                    ++it;
                }
            require(edges.size() == 4, "invalid exterior quad boundary");
            return edges;
        };
        require(
            remove_patch(before) == remove_patch(after),
            "exterior retriangulation changes the oriented patch boundary");
    }
    require(before.size() == after.size(), "retriangulation changes the volume boundary");
    for (const auto& [key, triangle] : before) {
        const auto it = after.find(key);
        require(
            it != after.end() && same_orientation(triangle, it->second),
            "retriangulation changes a fixed oriented boundary triangle");
    }
}

struct Candidate
{
    HybridCell prism;
    std::vector<PyramidOption> pyramids;
    std::array<QuadPatch, 3> sides;
    std::vector<Decomposition> splits;
    std::vector<std::vector<QuadPatch>> split_sides;
    size_t original_split = 0;
    unsigned boundary_splits = 63;
    unsigned admissible_splits = 0;
    unsigned original_tet_sides = 0;
    bool partition = false;
    bool conflict = false;
    bool positive = false;
    double minimum_sampled_jacobian = 0;
    double minimum_global_jacobian = 0;
    unsigned forbidden = 0;
    int selected = -2; // -2: all tets, -1: prism, otherwise pyramid option
    size_t special_vertices = 0;
    std::string reason;
};

struct Interface
{
    size_t a, b;
    int side_a, side_b;
};

std::vector<Interface> split_interfaces(
    std::vector<Candidate>& candidates,
    const SourceFaces& faces,
    const PrismaticMeshInput& input,
    double min_tet_volume,
    const detail::BackgroundPyramidPlans& background,
    std::set<Quad>& flexible_exterior_quads)
{
    std::map<Quad, std::vector<std::pair<size_t, int>>> users;
    for (size_t ci = 0; ci < candidates.size(); ++ci) {
        auto& c = candidates[ci];
        if (!c.partition || c.conflict) continue;
        c.splits = decompositions(c.prism);
        for (size_t k = 0; k < c.splits.size(); ++k) {
            c.split_sides.push_back(patches(c.prism, c.splits[k]));
            bool original = true;
            for (int s = 0; s < 3; ++s)
                original &= c.split_sides[k][s].triangles == c.sides[s].triangles;
            if (original) c.original_split = k;
        }
        // A Jacobian-valid prism may have inverted alternative tet splits. Only
        // expose safe choices to the interface solver. Original positive tets may
        // stay below the operation floor; each newly introduced tet must exceed it.
        const auto original_tets = tet_keys(c.splits[c.original_split]);
        c.admissible_splits = 1u << c.original_split;
        for (size_t k = 0; k < c.splits.size(); ++k) {
            if (k == c.original_split) continue;
            const bool admissible =
                std::all_of(c.splits[k].begin(), c.splits[k].end(), [&](const Tet& tet) {
                    const double floor = original_tets.count(sorted(tet)) ? 0 : min_tet_volume;
                    return tet_volume_above_threshold(input.vertices, tet, floor);
                });
            if (admissible) c.admissible_splits |= 1u << k;
        }
        for (int s = 0; s < 3; ++s) users[c.sides[s].vertices].push_back({ci, s});
    }
    std::vector<Interface> result;
    for (const auto& [quad, list] : users) {
        const auto& q = candidates[list[0].first].sides[list[0].second];
        const bool paired =
            list.size() == 2 &&
            q.triangles == candidates[list[1].first].sides[list[1].second].triangles &&
            std::all_of(q.triangles.begin(), q.triangles.end(), [&](const Tri& t) {
                return faces.at(t).tets.size() == 2;
            });
        if (paired) {
            result.push_back({list[0].first, list[1].first, list[0].second, list[1].second});
        } else {
            // Exterior boundaries and interfaces to original tet regions stay fixed.
            for (const auto& [ci, side] : list) {
                auto& c = candidates[ci];
                const bool exterior = std::all_of(
                    c.sides[side].triangles.begin(),
                    c.sides[side].triangles.end(),
                    [&](const Tri& t) { return faces.at(t).tets.size() == 1; });
                if (exterior && planar_convex_quad(input, cell_faces(c.prism)[side + 2])) {
                    flexible_exterior_quads.insert(quad);
                    continue;
                }
                if (!background.accepted.count(quad) &&
                    std::any_of(
                        c.sides[side].triangles.begin(),
                        c.sides[side].triangles.end(),
                        [&](const Tri& t) { return faces.at(t).tets.size() == 2; }))
                    c.original_tet_sides |= 1u << side;
                for (size_t k = 0; k < c.splits.size(); ++k)
                    if (c.split_sides[k][side].triangles != c.sides[side].triangles)
                        c.boundary_splits &= ~(1u << k);
            }
        }
    }
    return result;
}

// Six states per prism. A shared face must have the same diagonal on both sides,
// including quad/quad interfaces so exported reference decompositions also conform.
bool solve_splits(
    const std::vector<Candidate>& candidates,
    const std::vector<Interface>& interfaces,
    std::vector<size_t>& solution,
    size_t& failure,
    size_t& decisions,
    bool& limited)
{
    std::vector<unsigned> domains(candidates.size(), 1);
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& c = candidates[i];
        if (!c.partition || c.conflict) continue;
        domains[i] = c.boundary_splits & c.admissible_splits;
        if (c.selected >= 0) {
            domains[i] &= c.pyramids[c.selected].splits;
        }
        if (!domains[i]) {
            failure = i;
            return false;
        }
    }
    const auto compatible = [&](const Interface& edge, size_t a, size_t b) {
        return candidates[edge.a].split_sides[a][edge.side_a].triangles ==
               candidates[edge.b].split_sides[b][edge.side_b].triangles;
    };
    std::function<bool(std::vector<unsigned>)> search = [&](std::vector<unsigned> d) {
        if (++decisions > 10000) {
            limited = true;
            return false;
        }
        bool changed;
        do {
            changed = false;
            for (const auto& edge : interfaces) {
                unsigned a = 0, b = 0;
                for (size_t i = 0; i < 6; ++i)
                    if (d[edge.a] & (1u << i))
                        for (size_t j = 0; j < 6; ++j)
                            if (d[edge.b] & (1u << j))
                                if (compatible(edge, i, j)) {
                                    a |= 1u << i;
                                    b |= 1u << j;
                                }
                if (!a || !b) {
                    failure = !a ? edge.a : edge.b;
                    return false;
                }
                changed |= a != d[edge.a] || b != d[edge.b];
                d[edge.a] = a;
                d[edge.b] = b;
            }
        } while (changed);
        solution.resize(candidates.size());
        for (size_t i = 0; i < candidates.size(); ++i) {
            const auto original = candidates[i].original_split;
            if (d[i] & (1u << original))
                solution[i] = original;
            else
                for (size_t k = 0; k < 6; ++k)
                    if (d[i] & (1u << k)) {
                        solution[i] = k;
                        break;
                    }
        }
        for (const auto& edge : interfaces) {
            if (compatible(edge, solution[edge.a], solution[edge.b])) continue;
            size_t variable = edge.a;
            if ((d[variable] & (d[variable] - 1)) == 0) variable = edge.b;
            failure = variable;
            const auto preferred = solution[variable];
            for (size_t k = 0; k < 7; ++k) {
                const size_t choice = k == 0 ? preferred : k - 1;
                if ((k != 0 && choice == preferred) || !(d[variable] & (1u << choice))) continue;
                auto next = d;
                next[variable] = 1u << choice;
                if (search(std::move(next))) return true;
                if (limited) return false;
            }
            return false;
        }
        return true;
    };
    return search(std::move(domains));
}

size_t conflicting_pyramid(
    const std::vector<Candidate>& candidates,
    const std::vector<Interface>& interfaces,
    size_t failure)
{
    std::vector<std::vector<size_t>> neighbors(candidates.size());
    for (const auto& e : interfaces) {
        neighbors[e.a].push_back(e.b);
        neighbors[e.b].push_back(e.a);
    }
    std::queue<size_t> pending;
    std::vector<bool> visited(candidates.size(), false);
    pending.push(failure);
    visited[failure] = true;
    while (!pending.empty()) {
        const auto i = pending.front();
        pending.pop();
        if (candidates[i].selected >= 0) return i;
        for (size_t j : neighbors[i])
            if (!visited[j]) {
                visited[j] = true;
                pending.push(j);
            }
    }
    throw std::runtime_error("prism dominant: original reference split unexpectedly infeasible");
}
} // namespace

void validate_prism_dominant_mesh(
    const PrismaticMeshInput& input,
    const PrismDominantMesh& hybrid,
    double min_tet_volume)
{
    require(std::isfinite(min_tet_volume) && min_tet_volume >= 0, "invalid volume floor");
    const auto original_faces = source_faces(input);
    const size_t nt = input.tetrahedra.rows();
    require(hybrid.source_tet_to_cell.size() == nt, "missing source ownership");
    std::vector<size_t> count(nt, 0);
    // Each source boundary triangle must be represented by the same hybrid face on both sides.
    std::map<Tri, std::vector<std::vector<size_t>>> representations;
    std::map<std::vector<size_t>, std::vector<std::vector<size_t>>> orientations;
    for (size_t ci = 0; ci < hybrid.cells.size(); ++ci) {
        const auto& cell = hybrid.cells[ci];
        require(!cell.source_tets.empty(), "cell has no source tetrahedra");
        for (size_t v : cell.vertices)
            require(v < static_cast<size_t>(input.vertices.rows()), "invalid cell vertex");
        require(
            std::set<size_t>(cell.vertices.begin(), cell.vertices.end()).size() ==
                cell.vertices.size(),
            "repeated cell corner");
        require(matches_reference(input, cell), "cell is not a source tetrahedron partition");
        const size_t first = cell.source_tets.front();
        const bool band = input.offset_tet_tags.at(first) == 1;
        if (cell.background_transition)
            require(
                !band && input.input_cells.at(first) != 1,
                "background transition is not in background");
        if (cell.type == HybridCellType::Pyramid)
            require(
                band ? input.vertex_tags.at(cell.vertices[4]) == 1
                     : cell.background_transition && input.input_cells.at(first) != 1 &&
                           input.vertex_tags.at(cell.vertices[4]) == -1,
                "invalid shell/background pyramid apex");
        require(
            band || cell.type == HybridCellType::Tetrahedron ||
                (cell.type == HybridCellType::Pyramid && cell.background_transition),
            "fixed volume cell was merged");
        if (!band && cell.type == HybridCellType::Tetrahedron) {
            const auto original = source_tet(input, first);
            require(
                cell.vertices == std::vector<size_t>(original.begin(), original.end()),
                "fixed cell connectivity changed");
        }
        if (cell.type == HybridCellType::Prism)
            require(
                prism_jacobians(input, cell).positive &&
                    minimum_prism_jacobian(input.vertices, cell).determinant > 0,
                "nonpositive or nonfinite prism Jacobian");
        else
            require(
                positive_decompositions(
                    input,
                    cell,
                    cell.type == HybridCellType::Tetrahedron ? 0 : min_tet_volume),
                "nonpositive or too small decomposition tet");
        for (size_t tid : cell.source_tets) {
            require(tid < nt && ++count[tid] == 1, "source tet covered more than once");
            require(hybrid.source_tet_to_cell[tid] == ci, "source owner mismatch");
            require(
                input.input_cells[tid] == input.input_cells[first] &&
                    input.offset_tet_tags[tid] == input.offset_tet_tags[first],
                "mixed cell region tags");
        }
        for (const auto& f : cell_faces(cell)) {
            orientations[sorted(f)].push_back(canonical_cycle(f));
            if (f.size() == 3) representations[sorted(Tri{f[0], f[1], f[2]})].push_back(sorted(f));
        }
        for (const auto& q : patches(input, cell))
            for (const auto& t : q.triangles)
                representations[t].emplace_back(q.vertices.begin(), q.vertices.end());
    }
    require(
        std::all_of(count.begin(), count.end(), [](size_t n) { return n == 1; }),
        "source tet left uncovered");
    for (const auto& [t, f] : original_faces) {
        const bool internal = f.tets.size() == 2 && hybrid.source_tet_to_cell[f.tets[0]] ==
                                                        hybrid.source_tet_to_cell[f.tets[1]];
        const auto it = representations.find(t);
        if (internal) {
            require(it == representations.end(), "internal triangle emitted as a cell face");
        } else {
            require(
                it != representations.end() && it->second.size() == f.tets.size(),
                "missing interface triangle");
            if (f.tets.size() == 2)
                require(it->second[0] == it->second[1], "nonconforming triangle/quad interface");
        }
    }
    for (const auto& [key, cycles] : orientations) {
        require(cycles.size() <= 2, "nonmanifold hybrid face");
        if (cycles.size() == 2) {
            auto opposite = cycles[1];
            std::reverse(opposite.begin(), opposite.end());
            require(cycles[0] == canonical_cycle(opposite), "hybrid face orientations disagree");
        }
    }
}

namespace {
std::vector<Candidate> discover_candidates(
    const PrismaticMeshInput& input,
    const SourceFaces& faces,
    std::set<size_t>& special,
    std::vector<Tri>& bijective,
    size_t& repeated)
{
    std::map<Tet, size_t> band_tets;
    for (size_t tid = 0; tid < static_cast<size_t>(input.tetrahedra.rows()); ++tid)
        if (input.offset_tet_tags.at(tid) == 1)
            require(
                band_tets.emplace(sorted(source_tet(input, tid)), tid).second,
                "duplicate band tetrahedron");
    for (const auto& [f, incidence] : faces) {
        if (incidence.band_count != 1 || !std::all_of(f.begin(), f.end(), [&](size_t v) {
                return input.vertex_tags.at(v) == 2;
            }))
            continue;
        std::set<int64_t> parents;
        for (size_t v : f) {
            const auto parent = input.corr_input_vertex.at(v);
            require(
                parent >= 0 && parent < input.vertices.rows() && input.vertex_tags.at(parent) == 1,
                "offset face has invalid correspondence");
            parents.insert(parent);
        }
        if (parents.size() == 3)
            bijective.push_back(f);
        else {
            if (input.rod_state && std::all_of(f.begin(), f.end(), [&](size_t v) {
                    return is_rod_offset_vertex(input, v);
                }))
                continue;
            ++repeated;
            // Only the endpoints of a same-correspondence edge need collapse.
            // The third corner of an (a,a,b) face is not itself a collapse seed.
            for (size_t j = 0; j < 3; ++j) {
                const size_t a = f[j], b = f[(j + 1) % 3];
                if (input.corr_input_vertex[a] == input.corr_input_vertex[b]) {
                    special.insert(a);
                    special.insert(b);
                }
            }
        }
    }
    std::vector<Candidate> candidates;
    std::vector<std::vector<size_t>> claims(input.tetrahedra.rows());
    std::vector<std::array<size_t, 6>> columns;
    for (const auto& f : bijective)
        columns.push_back(
            {static_cast<size_t>(input.corr_input_vertex[f[0]]),
             static_cast<size_t>(input.corr_input_vertex[f[1]]),
             static_cast<size_t>(input.corr_input_vertex[f[2]]),
             f[0],
             f[1],
             f[2]});
    for (const auto& column : columns) {
        Candidate c;
        c.prism.type = HybridCellType::Prism;
        c.prism.prism_candidate = candidates.size();
        c.prism.vertices.assign(column.begin(), column.end());
        auto& v = c.prism.vertices;
        const auto bottom = faces.find(sorted(Tri{v[0], v[1], v[2]}));
        if (bottom == faces.end() || bottom->second.band_count == 0)
            c.reason = "missing_input_face";
        const auto six = sorted(v);
        for (int a = 0; a < 3; ++a)
            for (int b = a + 1; b < 4; ++b)
                for (int d = b + 1; d < 5; ++d)
                    for (int e = d + 1; e < 6; ++e) {
                        const auto it = band_tets.find({six[a], six[b], six[d], six[e]});
                        if (it != band_tets.end()) c.prism.source_tets.push_back(it->second);
                    }
        if (c.reason.empty() && c.prism.source_tets.size() == 3 &&
            matches_reference(input, c.prism)) {
            // Orient from the existing input-cap tet, not from a potentially invalid alternative
            // split.
            size_t apex = input.vertices.rows();
            for (size_t tid : c.prism.source_tets) {
                const auto t = source_tet(input, tid);
                if (std::all_of(v.begin(), v.begin() + 3, [&](size_t w) {
                        return std::find(t.begin(), t.end(), w) != t.end();
                    })) {
                    for (size_t w : t)
                        if (std::find(v.begin(), v.begin() + 3, w) == v.begin() + 3) apex = w;
                }
            }
            require(
                apex < static_cast<size_t>(input.vertices.rows()),
                "partition has no input cap");
            if (!tet_volume_above_threshold(input.vertices, {v[0], v[1], v[2], apex}, 0)) {
                std::swap(v[1], v[2]);
                std::swap(v[4], v[5]);
            }
            c.partition = true;
            const auto jacobian = prism_jacobians(input, c.prism);
            c.minimum_global_jacobian = minimum_prism_jacobian(input.vertices, c.prism).determinant;
            c.positive = jacobian.positive && c.minimum_global_jacobian > 0;
            c.minimum_sampled_jacobian = static_cast<double>(jacobian.minimum);
            if (!c.positive) c.reason = "invalid_prism_jacobian";
            const auto qs = patches(input, c.prism);
            require(qs.size() == 3, "prism needs three quadrilateral sides");
            std::copy(qs.begin(), qs.end(), c.sides.begin());
            for (size_t tid : c.prism.source_tets) claims[tid].push_back(candidates.size());
        } else if (c.reason.empty())
            c.reason = "unmatched_tet_partition";
        candidates.push_back(std::move(c));
    }
    for (const auto& list : claims)
        if (list.size() > 1)
            for (size_t ci : list) {
                candidates[ci].conflict = true;
                candidates[ci].reason = "overlapping_prism_partitions";
            }
    return candidates;
}
} // namespace

std::vector<HybridCell> prism_candidates_for_smoothing(const PrismaticMeshInput& input)
{
    const auto faces = source_faces(input);
    std::set<size_t> special;
    std::vector<Tri> bijective;
    size_t repeated = 0;
    const auto candidates = discover_candidates(input, faces, special, bijective, repeated);
    std::vector<HybridCell> result;
    for (const auto& c : candidates)
        if (c.partition && !c.conflict) result.push_back(c.prism);
    return result;
}

namespace {
// Mesh connectivity is rebuilt after validation; retain all other optimizer/export state.
PrismaticMeshInput reconstruction_state(const PrismaticMeshInput& input)
{
    PrismaticMeshInput r;
    r.vertices = input.vertices;
    r.tetrahedra = input.tetrahedra;
    r.vertex_tags = input.vertex_tags;
    r.source_vertex_ids = input.source_vertex_ids;
    r.corr_input_vid = input.corr_input_vid;
    r.source_edges = input.source_edges;
    r.source_faces = input.source_faces;
    r.layered_junction_vertices = input.layered_junction_vertices;
    r.layered_correspondence_report = input.layered_correspondence_report;
    r.relative_tet_volume_floor = input.relative_tet_volume_floor;
    r.corr_input_vertex = input.corr_input_vertex;
    r.input_vertices = input.input_vertices;
    r.offset_vertices = input.offset_vertices;
    r.input_to_offset_vertices = input.input_to_offset_vertices;
    r.input_cells = input.input_cells;
    r.offset_tet_tags = input.offset_tet_tags;
    r.fixed_background_vertices = input.fixed_background_vertices;
    r.offset_components = input.offset_components;
    r.input_to_components = input.input_to_components;
    r.vertex_component_ids = input.vertex_component_ids;
    r.singular_vertex_tags = input.singular_vertex_tags;
    r.optimal_normals = input.optimal_normals;
    r.target_positions = input.target_positions;
    r.input_average_edge_length = input.input_average_edge_length;
    r.target_thickness = input.target_thickness;
    r.optimization_iterations = input.optimization_iterations;
    r.jacobian_smoothing_report = input.jacobian_smoothing_report;
    r.background_remeshing_report = input.background_remeshing_report;
    r.rod_state = input.rod_state;
    r.junction_protection = input.junction_protection;
    return r;
}
} // namespace

namespace {
struct WholeRodLayer
{
    std::array<size_t, 2> axis;
    std::set<size_t> cavity;
    std::vector<Candidate> sectors;
    std::vector<size_t> solution;
    std::vector<detail::BackgroundQuad> exterior;
    std::string reason;
    double minimum_j = std::numeric_limits<double>::infinity();
};

std::map<Tri, Tri> layer_boundary(const Decomposition& tets)
{
    std::map<Tri, std::vector<Tri>> owners;
    for (const auto& t : tets)
        for (const auto& f : tet_faces(t)) owners[sorted(f)].push_back(f);
    std::map<Tri, Tri> boundary;
    for (const auto& [key, fs] : owners) {
        require(fs.size() <= 2, "nonmanifold rod-layer partition");
        if (fs.size() == 1)
            boundary.emplace(key, fs[0]);
        else
            require(!same_orientation(fs[0], fs[1]), "inconsistent rod-layer partition");
    }
    return boundary;
}

PrismDominantMesh build_whole_rod_layers(PrismaticMeshInput& input, double floor)
{
    auto& state = *input.rod_state;
    state.report["interface_swaps"] = align_rod_interfaces_by_swaps(input, floor);
    const auto original_faces = source_faces(input);
    std::map<std::array<size_t, 2>, std::set<size_t>> cavities;
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        if (input.offset_tet_tags[tid] != 1) continue;
        std::set<int64_t> parents;
        for (size_t v : source_tet(input, tid))
            parents.insert(
                input.vertex_tags[v] == 1   ? int64_t(v)
                : input.vertex_tags[v] == 2 ? input.corr_input_vertex[v]
                                            : -1);
        if (parents.size() == 2 && *parents.begin() >= 0)
            cavities[{size_t(*parents.begin()), size_t(*parents.rbegin())}].insert(tid);
    }
    std::vector<WholeRodLayer> layers;
    for (const auto& axis : state.input_edges) {
        WholeRodLayer layer;
        layer.axis = axis;
        const auto reject = [&](const std::string& reason) { layer.reason = reason; };
        if (!state.vertex_frame.count(axis[0]) || !state.vertex_frame.count(axis[1])) {
            reject("endpoint_region_not_a_ring");
            layers.push_back(std::move(layer));
            continue;
        }
        const auto& lo = state.frames[state.vertex_frame.at(axis[0])].ring;
        const auto& hi = state.frames[state.vertex_frame.at(axis[1])].ring;
        if (lo.size() < 3 || lo.size() != hi.size()) {
            reject("unequal_ring_degree");
            layers.push_back(std::move(layer));
            continue;
        }
        layer.cavity = cavities[sorted(axis)];
        if (layer.cavity.empty()) {
            reject("empty_layer_region");
            layers.push_back(std::move(layer));
            continue;
        }
        Decomposition old;
        bool protected_cell = false;
        for (size_t tid : layer.cavity) {
            old.push_back(source_tet(input, tid));
            protected_cell |= junction_tet_protected(input, old.back());
        }
        if (protected_cell) {
            reject("protected_connection");
            layers.push_back(std::move(layer));
            continue;
        }
        const auto boundary = layer_boundary(old);
        std::map<Tri, Tri> caps;
        std::set<Tri> old_exterior;
        for (const auto& [key, f] : boundary) {
            if (std::all_of(f.begin(), f.end(), [&](size_t v) {
                    return input.vertex_tags[v] == 2;
                }))
                old_exterior.insert(key);
            else
                caps.emplace(key, f);
        }
        std::map<Tri, Tri> new_caps;
        std::set<Tri> covered_exterior;
        std::map<Quad, std::vector<std::pair<size_t, int>>> quad_users;
        for (size_t k = 0; k < lo.size(); ++k) {
            Candidate c;
            c.prism.type = HybridCellType::Prism;
            c.prism.vertices =
                {axis[0], lo[k], lo[(k + 1) % lo.size()], axis[1], hi[k], hi[(k + 1) % hi.size()]};
            if (!(minimum_prism_jacobian(input.vertices, c.prism).determinant > 0)) {
                std::swap(c.prism.vertices[1], c.prism.vertices[2]);
                std::swap(c.prism.vertices[4], c.prism.vertices[5]);
            }
            const double jac = minimum_prism_jacobian(input.vertices, c.prism).determinant;
            if (!(std::isfinite(jac) && jac > 0)) reject("nonpositive_native_prism_jacobian");
            layer.minimum_j = std::min(layer.minimum_j, jac);
            c.partition = true;
            c.positive = true;
            c.selected = -1;
            c.splits = decompositions(c.prism);
            c.admissible_splits = 0;
            for (size_t d = 0; d < c.splits.size(); ++d) {
                c.split_sides.push_back(patches(c.prism, c.splits[d]));
                if (std::all_of(c.splits[d].begin(), c.splits[d].end(), [&](const Tet& t) {
                        return tet_volume_above_threshold(input.vertices, t, floor);
                    }))
                    c.admissible_splits |= 1u << d;
            }
            const auto fs = cell_faces(c.prism);
            for (int side = 0; side < 3; ++side) {
                const auto& f = fs[side + 2];
                const Quad q = {f[0], f[1], f[2], f[3]};
                quad_users[sorted(q)].push_back({k, side});
                if (!std::all_of(f.begin(), f.end(), [&](size_t v) {
                        return input.vertex_tags[v] == 2;
                    }))
                    continue; // Internal radial quad: old tet faces impose NO constraint.
                std::vector<Tri> triangles;
                bool has_background = false;
                for (const auto& [key, old_f] : boundary) {
                    if (!std::all_of(key.begin(), key.end(), [&](size_t v) {
                            return std::find(f.begin(), f.end(), v) != f.end();
                        }))
                        continue;
                    triangles.push_back(key);
                    covered_exterior.insert(key);
                }
                // A free exterior may be retriangulated to the ring product surface.
                // With background, the exterior must be paired with valid pyramids.
                for (const auto& t : old_exterior)
                    for (size_t tid : original_faces.at(t).tets)
                        if (!layer.cavity.count(tid)) has_background = true;
                if (has_background) {
                    if (triangles.size() != 2)
                        reject("external_front_retriangulation_required");
                    else {
                        const auto pair = sorted(std::array<Tri, 2>{triangles[0], triangles[1]});
                        for (size_t d = 0; d < c.splits.size(); ++d)
                            if (c.split_sides[d][side].triangles != pair)
                                c.boundary_splits &= ~(1u << d);
                        layer.exterior.push_back({q, pair});
                    }
                }
            }
            for (int side = 0; side < 2; ++side) {
                Tri f = {fs[side][0], fs[side][1], fs[side][2]};
                new_caps.emplace(sorted(f), f);
            }
            layer.sectors.push_back(std::move(c));
        }
        if (caps.size() != new_caps.size())
            reject("external_end_cap_mismatch");
        else
            for (const auto& [key, f] : caps) {
                const auto found = new_caps.find(key);
                if (found == new_caps.end() || !same_orientation(f, found->second))
                    reject("external_end_cap_mismatch");
            }
        // A free boundary can change; any interface to a retained volume must match.
        for (const auto& t : old_exterior)
            if (original_faces.at(t).tets.size() != 1 && !covered_exterior.count(t))
                reject("external_front_retriangulation_required");
        std::vector<Interface> interfaces;
        for (const auto& [q, users] : quad_users) {
            if (users.size() == 2)
                interfaces.push_back(
                    {users[0].first, users[1].first, users[0].second, users[1].second});
            else if (users.size() != 1)
                reject("nonmanifold_layer_quads");
        }
        if (layer.reason.empty()) {
            size_t failure = 0, decisions = 0;
            bool limited = false;
            if (!solve_splits(
                    layer.sectors,
                    interfaces,
                    layer.solution,
                    failure,
                    decisions,
                    limited))
                reject(
                    limited ? "reference_split_search_limit"
                            : "no_positive_compatible_reference_split");
        }
        layers.push_back(std::move(layer));
    }
    std::vector<std::vector<detail::BackgroundQuad>> groups;
    for (const auto& layer : layers)
        groups.push_back(
            layer.reason.empty() ? layer.exterior : std::vector<detail::BackgroundQuad>{});
    std::vector<std::string> pyramid_reasons;
    auto background = detail::plan_atomic_rod_layer_pyramids(input, groups, floor, pyramid_reasons);
    for (size_t i = 0; i < layers.size(); ++i)
        if (layers[i].reason.empty() && !pyramid_reasons[i].empty())
            layers[i].reason = "external_pyramid_" + pyramid_reasons[i];

    std::set<size_t> removed;
    for (const auto& layer : layers)
        if (layer.reason.empty()) removed.insert(layer.cavity.begin(), layer.cavity.end());
    auto reference = reconstruction_state(input);
    std::vector<Tet> new_tets;
    std::vector<int> regions, bands;
    std::map<size_t, size_t> old_to_new;
    PrismDominantMesh result;
    auto append_tet = [&](const Tet& t, int region, int band) {
        const size_t tid = new_tets.size();
        new_tets.push_back(t);
        regions.push_back(region);
        bands.push_back(band);
        return tid;
    };
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        if (removed.count(tid)) continue;
        const auto t = source_tet(input, tid);
        const size_t row = append_tet(t, input.input_cells[tid], input.offset_tet_tags[tid]);
        old_to_new[tid] = row;
        result.cells.push_back({HybridCellType::Tetrahedron, {t.begin(), t.end()}, {row}});
    }
    size_t prism_candidate = 0, accepted = 0;
    nlohmann::json entries = nlohmann::json::array();
    std::map<std::string, size_t> reasons;
    for (auto& layer : layers) {
        nlohmann::json entry = {
            {"input_vertex_ids",
             {input.source_vertex_ids[layer.axis[0]], input.source_vertex_ids[layer.axis[1]]}},
            {"accepted", layer.reason.empty()},
            {"reason", layer.reason},
            {"sectors", layer.sectors.size()},
            {"minimum_native_prism_det_j",
             std::isfinite(layer.minimum_j) ? nlohmann::json(layer.minimum_j)
                                            : nlohmann::json(nullptr)}};
        if (layer.reason.empty()) {
            ++accepted;
            for (size_t k = 0; k < layer.sectors.size(); ++k) {
                auto cell = layer.sectors[k].prism;
                cell.prism_candidate = prism_candidate++;
                const size_t original = *layer.cavity.begin();
                for (const auto& t : layer.sectors[k].splits[layer.solution[k]])
                    cell.source_tets.push_back(append_tet(t, input.input_cells[original], 1));
                result.cells.push_back(std::move(cell));
            }
        } else
            ++reasons[layer.reason];
        entries.push_back(std::move(entry));
    }
    reference.tetrahedra.resize(new_tets.size(), 4);
    for (size_t i = 0; i < new_tets.size(); ++i)
        for (int j = 0; j < 4; ++j) reference.tetrahedra(i, j) = new_tets[i][j];
    reference.input_cells = std::move(regions);
    reference.offset_tet_tags = std::move(bands);
    for (auto& [q, plan] : background.accepted)
        for (auto& tid : plan.cavity) tid = old_to_new.at(tid);
    result.source_tet_to_cell.assign(new_tets.size(), std::numeric_limits<size_t>::max());
    for (size_t ci = 0; ci < result.cells.size(); ++ci)
        for (size_t tid : result.cells[ci].source_tets) result.source_tet_to_cell[tid] = ci;
    detail::apply_background_pyramids(reference, result, background);
    validate_prism_dominant_mesh(reference, result, floor);
    // A kept background volume must preserve the complete outer domain boundary.
    if (!background.accepted.empty()) {
        const auto final_faces = source_faces(reference);
        std::map<Tri, Tri> before, after;
        for (const auto& [q, f] : original_faces)
            if (f.tets.size() == 1) before[q] = f.oriented[0];
        for (const auto& [q, f] : final_faces)
            if (f.tets.size() == 1) after[q] = f.oriented[0];
        require(
            before.size() == after.size(),
            "whole-layer builder changes background domain boundary");
        for (const auto& [q, f] : before)
            require(
                after.count(q) && same_orientation(f, after.at(q)),
                "whole-layer builder changes domain orientation");
    }
    size_t band_tets = 0, bg_tets = 0, bg_pyramids = 0, input_tets = 0;
    for (const auto& cell : result.cells) {
        if (cell.type == HybridCellType::Pyramid) ++bg_pyramids;
        if (cell.type != HybridCellType::Tetrahedron) continue;
        const size_t tid = cell.source_tets.front();
        if (reference.offset_tet_tags[tid] == 1)
            ++band_tets;
        else if (reference.input_cells[tid] == 1)
            ++input_tets;
        else
            ++bg_tets;
    }
    result.report = {
        {"method", "atomic_ring_to_ring_rod_layers"},
        {"prisms", prism_candidate},
        {"pyramids", bg_pyramids},
        {"shell_pyramids", 0},
        {"background_pyramids", bg_pyramids},
        {"band_tetrahedra", band_tets},
        {"input_tetrahedra", input_tets},
        {"background_tetrahedra", bg_tets},
        {"accepted_layers", accepted},
        {"rejected_layers", layers.size() - accepted},
        {"layer_rejections", reasons},
        {"layers", entries},
        {"output_cells", result.cells.size()},
        {"source_tetrahedra", reference.tetrahedra.rows()},
        {"background_pyramid_reconstruction", background.report},
        {"marker_updates", 0},
        {"marked_offset_source_vids", nlohmann::json::array()},
        {"checks",
         {{"source_partition", true},
          {"conforming_faces", true},
          {"consistent_face_orientation", true},
          {"global_prism_jacobians_positive", true},
          {"reference_tetrahedra_positive", true},
          {"layer_internal_old_tet_faces_ignored", true},
          {"layer_atomic_acceptance", true}}}};
    result.report["interface_swaps"] = state.report["interface_swaps"];
    result.report["junction_protection"] = junction_protection_report(input);
    std::vector<Tet> tets(reference.tetrahedra.rows());
    for (size_t i = 0; i < tets.size(); ++i) tets[i] = source_tet(reference, i);
    auto mesh = std::make_unique<TetMesh>();
    mesh->init_with_isolated_vertices(reference.vertices.rows(), tets);
    require(mesh->check_mesh_connectivity_validity(), "invalid whole-layer reference mesh");
    reference.mesh = std::move(mesh);
    label_offset_faces(reference);
    input = std::move(reference);
    logger().info(
        "Atomic rod layers: {}/{} accepted, {} prisms, {} band tets, {} background pyramids",
        accepted,
        layers.size(),
        prism_candidate,
        band_tets,
        bg_pyramids);
    return result;
}
} // namespace

static PrismDominantMesh build_shell_hybrid(PrismaticMeshInput& input, double min_tet_volume)
{
    require(std::isfinite(min_tet_volume) && min_tet_volume >= 0, "invalid volume floor");
    initialize_junction_protection(input);
    const auto faces = source_faces(input);
    std::set<size_t> special;
    std::vector<Tri> bijective;
    size_t repeated = 0;
    auto candidates = discover_candidates(input, faces, special, bijective, repeated);
    // Only terminal/attachment connection stars are protected. Ordinary closed
    // nonmanifold crossings still undergo collapse and normal marker reconstruction.
    const auto& junction_vertices = input.junction_protection->centers;
    for (const auto& c : candidates) {
        const auto& v = c.prism.vertices;
        if (!c.partition || c.conflict || std::any_of(v.begin(), v.begin() + 3, [&](size_t p) {
                return junction_vertices.count(p);
            }))
            for (size_t j = 3; j < 6; ++j)
                if (input.vertex_tags[v[j]] == 2) special.insert(v[j]);
    }
    std::vector<detail::BackgroundQuad> background_quads;
    for (const auto& c : candidates) {
        if (!c.partition || c.conflict || !c.positive) continue;
        const auto fs = cell_faces(c.prism);
        for (int side = 0; side < 3; ++side) {
            const auto& f = fs[side + 2];
            background_quads.push_back({{f[0], f[1], f[2], f[3]}, c.sides[side].triangles});
        }
    }
    auto background = detail::plan_background_pyramids(input, background_quads, min_tet_volume);
    std::set<Quad> flexible_exterior_quads;
    const auto interfaces = split_interfaces(
        candidates,
        faces,
        input,
        min_tet_volume,
        background,
        flexible_exterior_quads);

    // A quad bordering an unmatched tet region must become triangles. These are
    // initial topology/collapse-edge marks, not a buffer grown after a failed solve.
    std::map<Quad, size_t> quad_users;
    for (const auto& c : candidates)
        if (c.partition && !c.conflict)
            for (const auto& q : c.sides) ++quad_users[q.vertices];
    size_t fixed_interface_marks = 0, fixed_interface_quads = 0;
    for (const auto& c : candidates) {
        if (!c.partition || c.conflict) continue;
        for (size_t side = 0; side < 3; ++side) {
            const auto& q = c.sides[side];
            if (quad_users[q.vertices] != 1 || background.accepted.count(q.vertices) ||
                std::all_of(q.triangles.begin(), q.triangles.end(), [&](const Tri& t) {
                    return faces.at(t).tets.size() == 1;
                }))
                continue;
            ++fixed_interface_quads;
            for (size_t j : {side, (side + 1) % 3}) {
                const size_t v = c.prism.vertices[j + 3];
                if (input.vertex_tags[v] == 2) fixed_interface_marks += special.insert(v).second;
            }
        }
    }
    const size_t initial_special = special.size();
    for (auto& c : candidates) {
        if (!c.partition || c.conflict) continue;
        const auto& v = c.prism.vertices;
        for (int side = 0; side < 3; ++side) {
            if (input.vertex_tags[v[(side + 2) % 3]] != 1) continue;
            HybridCell pyramid;
            pyramid.type = HybridCellType::Pyramid;
            pyramid.prism_candidate = c.prism.prism_candidate;
            pyramid.vertices =
                {v[side], v[side + 3], v[(side + 1) % 3 + 3], v[(side + 1) % 3], v[(side + 2) % 3]};
            const bool positive = positive_decompositions(input, pyramid, min_tet_volume);
            unsigned supported = 0;
            for (auto split : decompositions(pyramid)) {
                split.push_back({v[(side + 2) % 3], v[3], v[4], v[5]});
                for (size_t k = 0; k < c.splits.size(); ++k)
                    if (tet_keys(split) == tet_keys(c.splits[k])) supported |= 1u << k;
            }
            supported &= c.admissible_splits;
            const size_t worst = worst_pyramid_corner(input, pyramid);
            const bool jacobian_positive = positive_decompositions(input, pyramid, 0);
            c.pyramids.push_back(
                {std::move(pyramid), side, supported, positive, worst, jacobian_positive});
        }
    }
    size_t passes = 0, constraints = fixed_interface_quads, direction_expansions = 0,
           search_decisions = 0, search_limit_fallbacks = 0;
    nlohmann::json direction_fallbacks = nlohmann::json::array();
    nlohmann::json marker_updates = nlohmann::json::array();
    std::vector<size_t> chosen_splits;
    auto add_mark = [&](size_t ci, const std::string& reason, size_t preferred = 3) {
        auto& c = candidates[ci];
        const auto minimum = minimum_prism_jacobian(input.vertices, c.prism);
        size_t selected = 3;
        for (size_t j = 0; j < 3; ++j) {
            const size_t v = c.prism.vertices[j + 3];
            if (input.vertex_tags[v] != 2 || special.count(v)) continue;
            if (selected == 3 || minimum.edge_values[j] < minimum.edge_values[selected])
                selected = j;
        }
        if (preferred < 3 && input.vertex_tags[c.prism.vertices[preferred + 3]] == 2 &&
            !special.count(c.prism.vertices[preferred + 3]))
            selected = preferred;
        require(selected < 3, "failed element has no unmarked offset corner");
        const size_t vertex = c.prism.vertices[selected + 3];
        special.insert(vertex);
        marker_updates.push_back(
            {{"iteration", passes},
             {"candidate", ci},
             {"reason", reason},
             {"offset_source_vid", input.source_vertex_ids[vertex]}});
        if (c.reason.empty()) c.reason = reason;
    };
    while (true) {
        ++passes;
        bool changed = false;
        for (size_t ci = 0; ci < candidates.size(); ++ci) {
            auto& c = candidates[ci];
            c.selected = -2;
            if (!c.partition || c.conflict) continue;
            const auto& v = c.prism.vertices;
            c.special_vertices = 0;
            c.forbidden = 0;
            size_t marked_corner = 3;
            for (size_t j = 0; j < 3; ++j)
                if (special.count(v[j + 3])) {
                    ++c.special_vertices;
                    marked_corner = j;
                }
            for (size_t side = 0; side < 3; ++side)
                if (special.count(v[side + 3]) || special.count(v[(side + 1) % 3 + 3]))
                    c.forbidden |= 1u << side;
            if (c.special_vertices == 0) {
                if (c.positive)
                    c.selected = -1;
                else {
                    add_mark(ci, "invalid_prism_jacobian");
                    changed = true;
                }
            } else if (c.special_vertices == 1) {
                // The marked corner uniquely fixes the input-apex pyramid direction.
                const int side = static_cast<int>((marked_corner + 1) % 3);
                auto option =
                    std::find_if(c.pyramids.begin(), c.pyramids.end(), [&](const auto& p) {
                        return p.side == side;
                    });
                if (option == c.pyramids.end() || !option->positive || !option->splits) {
                    const std::string reason =
                        option != c.pyramids.end() && !option->jacobian_positive
                            ? "invalid_pyramid_jacobian"
                        : option != c.pyramids.end() && !option->positive
                            ? "pyramid_below_volume_floor"
                            : "no_positive_pyramid_tet_split";
                    const size_t preferred =
                        option == c.pyramids.end()
                            ? 3
                            : (option->side + (option->worst_base_corner >= 2)) % 3;
                    add_mark(ci, reason, preferred);
                    changed = true;
                } else
                    c.selected = static_cast<int>(option - c.pyramids.begin());
            }
        }
        if (changed) continue;
        for (const auto& c : candidates) {
            if (!c.partition || c.conflict) continue;
            require(
                c.special_vertices == 0   ? c.selected == -1
                : c.special_vertices == 1 ? c.selected >= 0
                                          : c.selected == -2,
                "output shape disagrees with the offset vertex marks");
        }
        size_t failure = 0, decisions = 0;
        bool limited = false;
        const bool solved =
            solve_splits(candidates, interfaces, chosen_splits, failure, decisions, limited);
        search_decisions += decisions;
        if (solved) break;
        const size_t transition = conflicting_pyramid(candidates, interfaces, failure);
        direction_fallbacks.push_back(
            {{"failure_candidate", failure},
             {"updated_candidate", transition},
             {"boundary_split_mask", candidates[transition].boundary_splits},
             {"pyramid_split_mask",
              candidates[transition].pyramids[candidates[transition].selected].splits},
             {"original_tet_side_mask", candidates[transition].original_tet_sides},
             {"search_limit", limited}});
        // If no positive joint triangulation exists, degrade only the affected
        // one-mark transition. Do not freeze a three-tet block or mark all corners.
        add_mark(
            transition,
            limited ? "transition_search_limit" : "no_compatible_positive_transition");
        ++direction_expansions;
        search_limit_fallbacks += limited;
    }

    // Build the reference mesh privately. Commit only after checking its boundary and
    // the complete hybrid mesh. Existing vertex positions and input volume stay fixed.
    PrismaticMeshInput reference = reconstruction_state(input);
    nlohmann::json retriangulated = nlohmann::json::array();
    for (size_t ci = 0; ci < candidates.size(); ++ci) {
        auto& c = candidates[ci];
        if (!c.partition || c.conflict) continue;
        const size_t choice = chosen_splits[ci];
        if (choice != c.original_split) {
            const auto original_tets = tet_keys(c.splits[c.original_split]);
            nlohmann::json old_tets = nlohmann::json::array(), new_tets = nlohmann::json::array();
            for (size_t k = 0; k < 3; ++k) {
                const auto& tet = c.splits[choice][k];
                require(
                    tet_volume_above_threshold(
                        input.vertices,
                        tet,
                        original_tets.count(sorted(tet)) ? 0 : min_tet_volume),
                    "retriangulation introduces a nonpositive or below-floor tet");
                const size_t tid = c.prism.source_tets[k];
                std::array<int64_t, 4> before, after;
                for (int j = 0; j < 4; ++j) {
                    before[j] = input.source_vertex_ids[input.tetrahedra(tid, j)];
                    reference.tetrahedra(tid, j) = c.splits[choice][k][j];
                    after[j] = input.source_vertex_ids[reference.tetrahedra(tid, j)];
                }
                old_tets.push_back(before);
                new_tets.push_back(after);
            }
            retriangulated.push_back(
                {{"candidate", ci},
                 {"reference_tet_rows", c.prism.source_tets},
                 {"original_tets_vids", old_tets},
                 {"new_tets_vids", new_tets}});
        }
        if (c.selected >= 0) {
            auto& pyramid = c.pyramids[c.selected].cell;
            const std::set<size_t> corners(pyramid.vertices.begin(), pyramid.vertices.end());
            for (size_t tid : c.prism.source_tets) {
                const auto t = source_tet(reference, tid);
                if (std::all_of(t.begin(), t.end(), [&](size_t v) { return corners.count(v); }))
                    pyramid.source_tets.push_back(tid);
            }
            require(
                pyramid.source_tets.size() == 2 && matches_reference(reference, pyramid),
                "selected split does not realize the input-apex pyramid");
        }
    }
    const auto reference_faces = source_faces(reference);
    std::map<Tri, Tri> old_boundary, new_boundary;
    for (const auto& [key, f] : faces)
        if (f.tets.size() == 1) old_boundary[key] = f.oriented[0];
    for (const auto& [key, f] : reference_faces)
        if (f.tets.size() == 1) new_boundary[key] = f.oriented[0];
    check_boundary_reconstruction(old_boundary, new_boundary, flexible_exterior_quads);
    PrismDominantMesh result;
    const size_t nt = input.tetrahedra.rows();
    result.source_tet_to_cell.assign(nt, std::numeric_limits<size_t>::max());
    auto append = [&](HybridCell cell) {
        for (size_t tid : cell.source_tets) {
            require(
                result.source_tet_to_cell[tid] == std::numeric_limits<size_t>::max(),
                "duplicate source owner");
            result.source_tet_to_cell[tid] = result.cells.size();
        }
        result.cells.push_back(std::move(cell));
    };
    nlohmann::json exceptional = nlohmann::json::array();
    size_t full_tet_candidates = 0;
    double minimum_prism_jacobian = std::numeric_limits<double>::infinity();
    double minimum_global_jacobian = std::numeric_limits<double>::infinity();
    std::map<std::string, size_t> split_counts;
    std::map<std::string, size_t> reasons;
    for (size_t ci = 0; ci < candidates.size(); ++ci) {
        const auto& c = candidates[ci];
        if (c.partition && !c.conflict && c.positive) {
            unsigned choices = 0;
            for (size_t k = 0; k < 6; ++k) choices += (c.admissible_splits >> k) & 1u;
            ++split_counts[std::to_string(choices)];
        }
        if (c.selected == -1) {
            append(c.prism);
            minimum_prism_jacobian = std::min(minimum_prism_jacobian, c.minimum_sampled_jacobian);
            minimum_global_jacobian = std::min(minimum_global_jacobian, c.minimum_global_jacobian);
        } else if (c.selected >= 0) {
            const auto& pyramid = c.pyramids[c.selected].cell;
            append(pyramid);
            for (size_t tid : c.prism.source_tets)
                if (std::find(pyramid.source_tets.begin(), pyramid.source_tets.end(), tid) ==
                    pyramid.source_tets.end())
                    append(retained_tet(reference, tid, ci));
        } else {
            ++full_tet_candidates;
            if (c.partition && !c.conflict)
                for (size_t tid : c.prism.source_tets) append(retained_tet(reference, tid, ci));
        }
        if (!c.reason.empty()) ++reasons[c.reason];
        if (c.selected != -1) {
            std::vector<int64_t> offset_ids;
            for (int j = 3; j < 6; ++j)
                offset_ids.push_back(input.source_vertex_ids[c.prism.vertices[j]]);
            exceptional.push_back(
                {{"candidate", ci},
                 {"offset_source_vids", offset_ids},
                 {"source_tets", c.prism.source_tets},
                 {"reason", c.reason.empty() ? "transition_or_interface" : c.reason},
                 {"minimum_sampled_prism_jacobian",
                  c.partition ? nlohmann::json(c.minimum_sampled_jacobian)
                              : nlohmann::json(nullptr)},
                 {"minimum_global_prism_jacobian",
                  c.partition ? nlohmann::json(c.minimum_global_jacobian)
                              : nlohmann::json(nullptr)},
                 {"special_offset_vertices", c.special_vertices},
                 {"forbidden_quad_mask", c.forbidden},
                 {"result", c.selected >= 0 ? "pyramid_and_tetrahedron" : "tetrahedra"}});
        }
    }
    size_t unassigned_band = 0;
    for (size_t tid = 0; tid < nt; ++tid)
        if (result.source_tet_to_cell[tid] == std::numeric_limits<size_t>::max()) {
            unassigned_band += input.offset_tet_tags[tid] == 1;
            append(retained_tet(reference, tid));
        }
    detail::apply_background_pyramids(reference, result, background);
    // Cavity replacement must preserve the full domain's oriented triangular boundary.
    if (background.report["applied"].get<size_t>()) {
        const auto final_faces = source_faces(reference);
        std::map<Tri, Tri> final_boundary;
        for (const auto& [key, f] : final_faces)
            if (f.tets.size() == 1) final_boundary[key] = f.oriented[0];
        require(final_boundary.size() == new_boundary.size(), "background changes volume boundary");
        for (const auto& [key, f] : new_boundary) {
            const auto it = final_boundary.find(key);
            require(
                it != final_boundary.end() && same_orientation(f, it->second),
                "background changes oriented boundary triangle");
        }
    }
    validate_prism_dominant_mesh(reference, result, min_tet_volume);
    size_t prisms = 0, pyramids = 0, band_tetrahedra = 0, input_tetrahedra = 0,
           background_tetrahedra = 0, background_pyramids = 0;
    for (const auto& cell : result.cells) {
        if (cell.type == HybridCellType::Prism)
            ++prisms;
        else if (cell.type == HybridCellType::Pyramid) {
            ++pyramids;
            background_pyramids += cell.background_transition;
        } else if (reference.offset_tet_tags[cell.source_tets[0]] == 1)
            ++band_tetrahedra;
        else if (reference.input_cells[cell.source_tets[0]] == 1)
            ++input_tetrahedra;
        else
            ++background_tetrahedra;
    }
    std::vector<int64_t> marked_source_ids;
    for (size_t v : special) marked_source_ids.push_back(input.source_vertex_ids[v]);
    result.report = {
        {"prisms", prisms},
        {"pyramids", pyramids},
        {"shell_pyramids", pyramids - background_pyramids},
        {"background_pyramids", background_pyramids},
        {"background_pyramid_reconstruction", background.report},
        {"band_tetrahedra", band_tetrahedra},
        {"input_tetrahedra", input_tetrahedra},
        {"background_tetrahedra", background_tetrahedra},
        {"source_tetrahedra", reference.tetrahedra.rows()},
        {"original_source_tetrahedra", nt},
        {"output_cells", result.cells.size()},
        {"bijective_offset_faces", bijective.size()},
        {"repeated_correspondence_faces", repeated},
        {"initial_special_offset_vertices", initial_special},
        {"marker_iterations", passes},
        {"marker_updates", marker_updates},
        {"marked_offset_source_vids", marked_source_ids},
        {"transition_shape_rule",
         "0 marks: prism; 1 mark: aligned input-apex pyramid + tet; 2 or 3 marks: reconstructed "
         "tets"},
        {"interface_reconstruction_updates", direction_expansions},
        {"fixed_interface_initial_marks", fixed_interface_marks},
        {"flexible_planar_exterior_quads", flexible_exterior_quads.size()},
        {"protected_connection_vertices", junction_vertices.size()},
        {"junction_protection", junction_protection_report(input)},
        {"expanded_special_offset_vertices", special.size()},
        {"fully_tetrahedral_candidates", full_tet_candidates},
        {"unassigned_band_tets_retained", unassigned_band},
        {"candidate_rejections", reasons},
        {"compatibility_passes", passes},
        {"quad_constraints_added", constraints},
        {"pyramid_apex", {{"shell", "input"}, {"background", "new cavity kernel point"}}},
        {"direction_region_expansions", direction_expansions},
        {"direction_fallbacks", direction_fallbacks},
        {"direction_search_decisions", search_decisions},
        {"direction_search_limit_fallbacks", search_limit_fallbacks},
        {"retriangulated_prisms", retriangulated.size()},
        {"retriangulated_regions", retriangulated},
        {"min_tet_volume", min_tet_volume},
        {"jacobian_smoothing", input.jacobian_smoothing_report},
        {"prism_global_jacobian_method", "side-edge quadratic minima"},
        {"minimum_output_prism_global_det_j",
         prisms ? nlohmann::json(minimum_global_jacobian) : nlohmann::json(nullptr)},
        {"prism_jacobian_check",
         {{"criterion", "finite det(J) > 0"},
          {"reference_domain", "r >= 0, s >= 0, r+s <= 1, 0 <= t <= 1"},
          {"triangle_points", jacobian_triangle_points},
          {"height_points", jacobian_height_points},
          {"samples_per_prism", jacobian_triangle_points.size() * jacobian_height_points.size()},
          {"minimum_output_prism_det_j",
           prisms ? nlohmann::json(minimum_prism_jacobian) : nlohmann::json(nullptr)}}},
        {"jacobian_valid_candidate_admissible_split_counts", split_counts},
        {"checks",
         {{"source_partition", true},
          {"conforming_faces", true},
          {"consistent_face_orientation", true},
          {"sampled_prism_jacobians_positive", true},
          {"global_prism_jacobians_positive", true},
          {"global_pyramid_jacobians_positive", true},
          {"reference_tetrahedra_positive", true},
          {"new_reference_tetrahedra_above_min_volume", true},
          {"pyramid_decompositions_above_min_volume", true},
          {"input_volume_and_non_cavity_background_preserved", true},
          {"existing_vertex_positions_preserved", true},
          {"volume_boundary_preserved", true},
          {"all_shell_pyramid_apices_on_input", true}}},
        {"transition_and_tet_regions", exceptional}};
    logger().info(
        "Prism dominant output: {} prisms, {} shell pyramids, {} background pyramids, {} band "
        "tetrahedra; {} input and {} background tetrahedra; {} interface passes",
        prisms,
        pyramids - background_pyramids,
        background_pyramids,
        band_tetrahedra,
        input_tetrahedra,
        background_tetrahedra,
        passes);
    logger().info(
        "Prism geometry: {} Jacobian samples per candidate, {} invalid candidates; minimum "
        "output prism det(J) = {}",
        jacobian_triangle_points.size() * jacobian_height_points.size(),
        reasons.count("invalid_prism_jacobian") ? reasons.at("invalid_prism_jacobian") : 0,
        prisms ? minimum_prism_jacobian : 0);
    logger().info(
        "Marker reconstruction: {} column regions retriangulated, {} interface mark "
        "updates, {} search-limit fallbacks",
        retriangulated.size(),
        direction_expansions,
        search_limit_fallbacks);
    if (!retriangulated.empty() || background_pyramids) {
        std::vector<Tet> tets(reference.tetrahedra.rows());
        for (size_t tid = 0; tid < tets.size(); ++tid) tets[tid] = source_tet(reference, tid);
        auto mesh = std::make_unique<TetMesh>();
        mesh->init_with_isolated_vertices(reference.vertices.rows(), tets);
        require(mesh->check_mesh_connectivity_validity(), "invalid reconstructed tet connectivity");
        reference.mesh = std::move(mesh);
        label_offset_faces(reference);
        input = std::move(reference);
    }
    logger().info(
        "Background transition: {} pyramids, {} new kernel points; {} feasible cavities",
        background_pyramids,
        background_pyramids,
        background.accepted.size());
    if (!input.background_remeshing_report.is_null())
        result.report["background_remeshing"] = input.background_remeshing_report;
    return result;
}
PrismDominantMesh build_prism_dominant_mesh(PrismaticMeshInput& input, double floor)
{
    if (!input.rod_state) return build_shell_hybrid(input, floor);
    auto rods = build_whole_rod_layers(input, floor);
    bool has_shell = false;
    for (size_t i = 0; i < size_t(input.tetrahedra.rows()) && !has_shell; ++i) {
        if (input.offset_tet_tags[i] != 1) continue;
        size_t input_corners = 0;
        for (size_t v : source_tet(input, i)) input_corners += input.vertex_tags[v] == 1;
        has_shell = input_corners >= 3;
    }
    if (!has_shell) return rods;
    auto shells = build_shell_hybrid(input, floor);
    std::set<size_t> reserved;
    std::vector<HybridCell> retained;
    int64_t next_candidate = 0;
    for (const auto& cell : shells.cells)
        next_candidate = std::max(next_candidate, cell.prism_candidate + 1);
    for (const auto& cell : rods.cells) {
        if (cell.type == HybridCellType::Tetrahedron) continue;
        require(
            matches_reference(input, cell),
            "shell pass changed a reserved rod prism/pyramid partition");
        for (size_t tid : cell.source_tets)
            require(reserved.insert(tid).second, "overlapping reserved rod cells");
        retained.push_back(cell);
        if (retained.back().prism_candidate >= 0) retained.back().prism_candidate += next_candidate;
    }
    PrismDominantMesh result;
    for (auto& cell : shells.cells) {
        size_t overlap = 0;
        for (size_t tid : cell.source_tets) overlap += reserved.count(tid);
        if (overlap) {
            require(
                cell.type == HybridCellType::Tetrahedron && cell.source_tets.size() == 1,
                "shell cell overlaps a reserved rod native cell");
            continue;
        }
        result.cells.push_back(std::move(cell));
    }
    result.cells.insert(result.cells.end(), retained.begin(), retained.end());
    result.source_tet_to_cell.assign(input.tetrahedra.rows(), std::numeric_limits<size_t>::max());
    size_t prisms = 0, pyramids = 0, band_tets = 0, volume_tets = 0, bg_tets = 0, bg_pyramids = 0;
    for (size_t ci = 0; ci < result.cells.size(); ++ci) {
        const auto& cell = result.cells[ci];
        for (size_t tid : cell.source_tets) {
            require(
                result.source_tet_to_cell[tid] == std::numeric_limits<size_t>::max(),
                "mixed cell ownership overlap");
            result.source_tet_to_cell[tid] = ci;
        }
        if (cell.type == HybridCellType::Prism)
            ++prisms;
        else if (cell.type == HybridCellType::Pyramid) {
            ++pyramids;
            bg_pyramids += cell.background_transition;
        } else if (input.offset_tet_tags[cell.source_tets[0]] == 1)
            ++band_tets;
        else if (input.input_cells[cell.source_tets[0]] == 1)
            ++volume_tets;
        else
            ++bg_tets;
    }
    validate_prism_dominant_mesh(input, result, floor);
    result.report = shells.report;
    result.report["method"] = "atomic_region_rod_layers_composed_with_shell_prisms";
    result.report["atomic_rod_layers"] = rods.report;
    result.report["shell_reconstruction"] = shells.report;
    result.report["interface_swaps"] =
        rods.report.value("interface_swaps", nlohmann::json::object());
    result.report["prisms"] = prisms;
    result.report["pyramids"] = pyramids;
    result.report["shell_pyramids"] = pyramids - bg_pyramids;
    result.report["background_pyramids"] = bg_pyramids;
    result.report["band_tetrahedra"] = band_tets;
    result.report["input_tetrahedra"] = volume_tets;
    result.report["background_tetrahedra"] = bg_tets;
    result.report["output_cells"] = result.cells.size();
    result.report["source_tetrahedra"] = input.tetrahedra.rows();
    result.report["checks"]["atomic_rod_cells_preserved_during_shell_reconstruction"] = true;
    result.report["checks"]["mixed_interfaces_validated_together"] = true;
    result.report["marked_offset_source_vids"] =
        shells.report.value("marked_offset_source_vids", nlohmann::json::array());
    logger().info(
        "Mixed region strategy: {} native rod prisms, {} total prisms, {} band tets, {} background "
        "pyramids",
        rods.report["prisms"].get<size_t>(),
        prisms,
        band_tets,
        bg_pyramids);
    return result;
}

} // namespace wmtk::components::prismatic_mesh
