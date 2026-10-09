#include "prism_jacobian.hpp"
#include "prismatic_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <wmtk/utils/Logger.hpp>

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
};

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
    bool frozen = false;
    bool partition = false;
    bool conflict = false;
    bool positive = false;
    double minimum_sampled_jacobian = 0;
    double minimum_global_jacobian = 0;
    unsigned forbidden = 0;
    int selected = -2; // -2: all tets, -1: prism, otherwise pyramid option
    bool selected_once = false;
    size_t special_vertices = 0;
    std::string reason;
};

void select(Candidate& c)
{
    // Representation only degrades: prism -> input-apex pyramid+tet -> all tets.
    // Flexible three-tet prism regions may change diagonals until the global solve;
    // frozen regions retain their original tetrahedra.
    if (c.frozen) {
        c.selected = -2;
        c.selected_once = true;
        return;
    }
    if (c.selected_once && c.selected == -2) return;
    if (c.selected_once && c.selected >= 0) {
        if (c.forbidden & (1u << c.pyramids[c.selected].side)) c.selected = -2;
        return;
    }
    c.selected_once = true;
    c.selected = -2;
    if (!c.partition || c.conflict || !c.positive) return;
    if (c.forbidden == 0) {
        c.selected = -1;
        return;
    }
    for (size_t i = 0; i < c.pyramids.size(); ++i) {
        if (!(c.forbidden & (1u << c.pyramids[i].side))) {
            c.selected = static_cast<int>(i);
            return;
        }
    }
}

std::vector<int> selected_sides(const Candidate& c)
{
    if (c.selected == -1) return {0, 1, 2};
    if (c.selected >= 0) return {c.pyramids[c.selected].side};
    return {};
}

struct Interface
{
    size_t a, b;
    int side_a, side_b;
};

std::vector<Interface> split_interfaces(
    std::vector<Candidate>& candidates,
    const SourceFaces& faces,
    const PrismaticMeshInput& input,
    double min_tet_volume)
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
        if (c.positive)
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
                if (std::any_of(
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

void propagate_shapes(
    std::vector<Candidate>& candidates,
    const SourceFaces& faces,
    size_t& passes,
    size_t& constraints)
{
    while (true) {
        ++passes;
        std::map<Quad, std::vector<std::pair<size_t, int>>> requests;
        for (size_t ci = 0; ci < candidates.size(); ++ci)
            for (int side : selected_sides(candidates[ci]))
                requests[candidates[ci].sides[side].vertices].push_back({ci, side});
        size_t added = 0;
        for (const auto& [quad, users] : requests) {
            const auto& first = candidates[users[0].first].sides[users[0].second];
            bool compatible = false;
            if (users.size() == 1) {
                compatible =
                    std::all_of(first.triangles.begin(), first.triangles.end(), [&](const Tri& t) {
                        return faces.at(t).tets.size() == 1;
                    });
            } else if (users.size() == 2) {
                const auto& other = candidates[users[1].first].sides[users[1].second];
                compatible =
                    first.triangles == other.triangles &&
                    std::all_of(first.triangles.begin(), first.triangles.end(), [&](const Tri& t) {
                        return faces.at(t).tets.size() == 2;
                    });
            }
            if (compatible) continue;
            for (const auto& [ci, side] : users) {
                auto& c = candidates[ci];
                if (!(c.forbidden & (1u << side))) {
                    c.forbidden |= 1u << side;
                    ++added;
                }
            }
        }
        constraints += added;
        if (!added) break;
        for (auto& c : candidates) select(c);
    }
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
        if (!c.positive || c.frozen)
            domains[i] &= 1u << c.original_split;
        else if (c.selected >= 0) {
            domains[i] &= c.pyramids[c.selected].splits;
            // A transition's triangular side interfaces need a prism-derived tet
            // buffer. Fixed input caps are triangles, not these lateral quads.
            if (c.original_tet_sides) domains[i] = 0;
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
    std::map<std::vector<size_t>, std::vector<size_t>> face_owners;
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
        if (cell.type == HybridCellType::Pyramid)
            require(input.vertex_tags.at(cell.vertices[4]) == 1, "pyramid apex is not on input");
        const size_t first = cell.source_tets.front();
        const bool band = input.offset_tet_tags.at(first) == 1;
        require(band || cell.type == HybridCellType::Tetrahedron, "fixed volume cell was merged");
        if (!band) {
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
            face_owners[sorted(f)].push_back(ci);
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
    for (const auto& [face, owners] : face_owners) {
        if (face.size() != 3 || owners.size() != 2 ||
            std::all_of(face.begin(), face.end(), [&](size_t v) {
                return input.vertex_tags[v] == 1;
            }))
            continue;
        for (size_t k = 0; k < 2; ++k) {
            const auto& pyramid = hybrid.cells[owners[k]];
            const auto& neighbor = hybrid.cells[owners[1 - k]];
            if (pyramid.type == HybridCellType::Pyramid &&
                neighbor.type == HybridCellType::Tetrahedron)
                require(
                    neighbor.prism_candidate >= 0 &&
                        input.offset_tet_tags[neighbor.source_tets[0]] == 1,
                    "pyramid lateral interface lacks a prism-derived tet buffer");
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
            ++repeated;
            special.insert(f.begin(), f.end());
        }
    }
    std::vector<Candidate> candidates;
    std::vector<std::vector<size_t>> claims(input.tetrahedra.rows());
    for (const auto& f : bijective) {
        Candidate c;
        c.prism.type = HybridCellType::Prism;
        c.prism.prism_candidate = candidates.size();
        for (size_t v : f) c.prism.vertices.push_back(input.corr_input_vertex[v]);
        c.prism.vertices.insert(c.prism.vertices.end(), f.begin(), f.end());
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
                        if (input.vertex_tags[w] == 2) apex = w;
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

PrismDominantMesh build_prism_dominant_mesh(PrismaticMeshInput& input, double min_tet_volume)
{
    require(std::isfinite(min_tet_volume) && min_tet_volume >= 0, "invalid volume floor");
    const auto faces = source_faces(input);
    std::set<size_t> special;
    std::vector<Tri> bijective;
    size_t repeated = 0;
    auto candidates = discover_candidates(input, faces, special, bijective, repeated);
    const size_t initial_special = special.size();
    // Geometry failures seed a larger tet region; correspondence and positions remain fixed.
    for (const auto& c : candidates)
        if (!c.reason.empty()) special.insert(c.prism.vertices.begin() + 3, c.prism.vertices.end());
    const auto interfaces = split_interfaces(candidates, faces, input, min_tet_volume);
    for (auto& c : candidates) {
        if (!c.partition || c.conflict || !c.positive) continue;
        const auto& v = c.prism.vertices;
        for (int side = 0; side < 3; ++side) {
            HybridCell pyramid;
            pyramid.type = HybridCellType::Pyramid;
            pyramid.prism_candidate = c.prism.prism_candidate;
            // Base winds inward; its emitted face winds outward. The apex is INPUT.
            pyramid.vertices =
                {v[side], v[side + 3], v[(side + 1) % 3 + 3], v[(side + 1) % 3], v[(side + 2) % 3]};
            if (!positive_decompositions(input, pyramid, min_tet_volume)) continue;
            unsigned supported = 0;
            for (auto split : decompositions(pyramid)) {
                split.push_back({v[(side + 2) % 3], v[3], v[4], v[5]});
                for (size_t k = 0; k < c.splits.size(); ++k)
                    if (tet_keys(split) == tet_keys(c.splits[k])) supported |= 1u << k;
            }
            supported &= c.admissible_splits;
            if (supported) c.pyramids.push_back({std::move(pyramid), side, supported});
        }
    }
    size_t passes = 0, constraints = 0, direction_expansions = 0, search_decisions = 0,
           search_limit_fallbacks = 0;
    std::vector<size_t> chosen_splits;
    while (true) {
        for (auto& c : candidates) {
            const auto& v = c.prism.vertices;
            c.special_vertices = 0;
            for (int j = 3; j < 6; ++j) c.special_vertices += special.count(v[j]);
            for (int side = 0; side < 3; ++side)
                if (special.count(v[side + 3]) || special.count(v[(side + 1) % 3 + 3]))
                    c.forbidden |= 1u << side;
            select(c);
        }
        propagate_shapes(candidates, faces, passes, constraints);
        size_t failure = 0, decisions = 0;
        bool limited = false;
        const bool solved =
            solve_splits(candidates, interfaces, chosen_splits, failure, decisions, limited);
        search_decisions += decisions;
        if (solved) break;
        // Freeze an affected pyramid region, grow the frontier, then solve again.
        // Every failure freezes a new candidate, so fallback terminates at the source mesh.
        auto& c = candidates[conflicting_pyramid(candidates, interfaces, failure)];
        c.frozen = true;
        c.reason = limited                ? "direction_search_limit"
                   : c.original_tet_sides ? "pyramid_needs_prism_tet_buffer"
                                          : "incompatible_input_apex_interfaces";
        special.insert(c.prism.vertices.begin() + 3, c.prism.vertices.end());
        ++direction_expansions;
        search_limit_fallbacks += limited;
    }

    // Build the reference mesh privately. Commit only after checking its boundary and
    // the complete hybrid mesh; vertex positions and fixed tetrahedra never change.
    PrismaticMeshInput reference;
    reference.vertices = input.vertices;
    reference.tetrahedra = input.tetrahedra;
    reference.vertex_tags = input.vertex_tags;
    reference.input_cells = input.input_cells;
    reference.offset_tet_tags = input.offset_tet_tags;
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
    require(
        old_boundary.size() == new_boundary.size(),
        "retriangulation changes the volume boundary");
    for (const auto& [key, f] : old_boundary) {
        const auto it = new_boundary.find(key);
        require(
            it != new_boundary.end() && same_orientation(f, it->second),
            "retriangulation changes an oriented boundary triangle");
    }
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
    validate_prism_dominant_mesh(reference, result, min_tet_volume);
    size_t prisms = 0, pyramids = 0, band_tetrahedra = 0, input_tetrahedra = 0,
           background_tetrahedra = 0;
    for (const auto& cell : result.cells) {
        if (cell.type == HybridCellType::Prism)
            ++prisms;
        else if (cell.type == HybridCellType::Pyramid)
            ++pyramids;
        else if (input.offset_tet_tags[cell.source_tets[0]] == 1)
            ++band_tetrahedra;
        else if (input.input_cells[cell.source_tets[0]] == 1)
            ++input_tetrahedra;
        else
            ++background_tetrahedra;
    }
    result.report = {
        {"prisms", prisms},
        {"pyramids", pyramids},
        {"band_tetrahedra", band_tetrahedra},
        {"input_tetrahedra", input_tetrahedra},
        {"background_tetrahedra", background_tetrahedra},
        {"source_tetrahedra", nt},
        {"output_cells", result.cells.size()},
        {"bijective_offset_faces", bijective.size()},
        {"repeated_correspondence_faces", repeated},
        {"initial_special_offset_vertices", initial_special},
        {"expanded_special_offset_vertices", special.size()},
        {"fully_tetrahedral_candidates", full_tet_candidates},
        {"unassigned_band_tets_retained", unassigned_band},
        {"candidate_rejections", reasons},
        {"compatibility_passes", passes},
        {"quad_constraints_added", constraints},
        {"pyramid_apex", "input"},
        {"direction_region_expansions", direction_expansions},
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
          {"reference_tetrahedra_positive", true},
          {"new_reference_tetrahedra_above_min_volume", true},
          {"pyramid_decompositions_above_min_volume", true},
          {"fixed_cells_preserved", true},
          {"volume_boundary_preserved", true},
          {"all_pyramid_apices_on_input", true}}},
        {"transition_and_tet_regions", exceptional}};
    logger().info(
        "Prism dominant band: {} prisms, {} pyramids, {} tetrahedra; {} input and {} background "
        "tetrahedra preserved; {} interface passes",
        prisms,
        pyramids,
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
        "Input-apex reconstruction: {} prism regions retriangulated, {} retained-region "
        "expansions, {} search-limit fallbacks",
        retriangulated.size(),
        direction_expansions,
        search_limit_fallbacks);
    if (!retriangulated.empty()) {
        std::vector<Tet> tets(nt);
        for (size_t tid = 0; tid < nt; ++tid) tets[tid] = source_tet(reference, tid);
        auto mesh = std::make_unique<TetMesh>();
        mesh->init_with_isolated_vertices(input.vertices.rows(), tets);
        require(mesh->check_mesh_connectivity_validity(), "invalid reconstructed tet connectivity");
        input.tetrahedra = std::move(reference.tetrahedra);
        input.mesh = std::move(mesh);
        label_offset_faces(input);
    }
    if (!input.background_remeshing_report.is_null())
        result.report["background_remeshing"] = input.background_remeshing_report;
    return result;
}
} // namespace wmtk::components::prismatic_mesh
