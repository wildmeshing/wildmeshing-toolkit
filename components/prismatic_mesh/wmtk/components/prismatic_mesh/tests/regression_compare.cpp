#include <wmtk/utils/AMIPS.h>
#include <wmtk/components/prismatic_mesh/Types.hpp>
#include <wmtk/utils/io.hpp>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>

using namespace wmtk;
using namespace wmtk::components::prismatic_mesh;

namespace {

struct Cell
{
    int type = 0;
    std::vector<size_t> vertices;
};

struct Mesh
{
    std::map<size_t, Vector3d> vertices;
    std::vector<Cell> cells;
    std::set<std::string> physical_groups;
};

using Facet = std::vector<size_t>;
using Triangle = std::array<Vector3d, 3>;

std::vector<Facet> boundary_facets(const Mesh& mesh, bool& conforming);

size_t node_count(const int type)
{
    switch (type) {
    case 4: return 4;
    case 6: return 6;
    case 7: return 5;
    default: return 0;
    }
}

Mesh load(const std::filesystem::path& path)
{
    MshData data;
    data.load(path.string());
    Mesh result;
    for (const auto& block : data.m_spec.nodes.entity_blocks) {
        for (size_t i = 0; i < block.num_nodes_in_block; ++i) {
            result.vertices[block.tags[i]] =
                Vector3d(block.data[3 * i], block.data[3 * i + 1], block.data[3 * i + 2]);
        }
    }
    for (const auto& block : data.m_spec.elements.entity_blocks) {
        const size_t n = node_count(block.element_type);
        if (block.entity_dim != 3 || n == 0) continue;
        for (size_t i = 0; i < block.num_elements_in_block; ++i) {
            Cell cell;
            cell.type = block.element_type;
            for (size_t j = 0; j < n; ++j) {
                cell.vertices.push_back(block.data[i * (n + 1) + j + 1]);
            }
            result.cells.push_back(std::move(cell));
        }
    }
    for (const auto& group : data.m_spec.physical_groups) {
        result.physical_groups.insert(std::to_string(group.dim) + ":" + group.name);
    }
    return result;
}

std::pair<Vector3d, Vector3d> bounds(const Mesh& mesh)
{
    Vector3d lo = Vector3d::Constant(std::numeric_limits<double>::max());
    Vector3d hi = Vector3d::Constant(std::numeric_limits<double>::lowest());
    for (const auto& [_, p] : mesh.vertices) {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    return {lo, hi};
}

std::string point_key(const Vector3d& p, const double eps)
{
    std::ostringstream stream;
    stream << std::llround(p.x() / eps) << ',' << std::llround(p.y() / eps) << ','
           << std::llround(p.z() / eps);
    return stream.str();
}

std::multiset<std::string> canonical_cells(const Mesh& mesh, const double eps)
{
    std::multiset<std::string> result;
    for (const auto& cell : mesh.cells) {
        std::vector<std::string> points;
        for (const size_t tag : cell.vertices) {
            points.push_back(point_key(mesh.vertices.at(tag), eps));
        }
        std::vector<std::vector<size_t>> permutations;
        if (cell.type == 4) {
            std::vector<size_t> permutation = {0, 1, 2, 3};
            do {
                permutations.push_back(permutation);
            } while (std::next_permutation(permutation.begin(), permutation.end()));
        } else if (cell.type == 6) {
            for (size_t layer_swap = 0; layer_swap < 2; ++layer_swap) {
                for (size_t reflection = 0; reflection < 2; ++reflection) {
                    for (size_t shift = 0; shift < 3; ++shift) {
                        std::vector<size_t> permutation(6);
                        for (size_t i = 0; i < 3; ++i) {
                            const size_t ring = (shift + (reflection ? 3 - i : i)) % 3;
                            permutation[i] = ring + 3 * layer_swap;
                            permutation[i + 3] = ring + 3 * (1 - layer_swap);
                        }
                        permutations.push_back(std::move(permutation));
                    }
                }
            }
        } else {
            for (size_t reflection = 0; reflection < 2; ++reflection) {
                for (size_t shift = 0; shift < 4; ++shift) {
                    std::vector<size_t> permutation(5);
                    for (size_t i = 0; i < 4; ++i) {
                        permutation[i] = (shift + (reflection ? 4 - i : i)) % 4;
                    }
                    permutation[4] = 4;
                    permutations.push_back(std::move(permutation));
                }
            }
        }

        std::string best;
        for (const auto& permutation : permutations) {
            std::ostringstream key;
            key << cell.type << ':';
            for (const size_t i : permutation) key << points[i] << ';';
            if (best.empty() || key.str() < best) best = key.str();
        }
        result.insert(std::move(best));
    }
    return result;
}

std::multiset<std::string> canonical_boundary(const Mesh& mesh, const double eps)
{
    bool conforming = false;
    std::multiset<std::string> result;
    for (const auto& facet : boundary_facets(mesh, conforming)) {
        std::vector<std::string> points;
        for (const size_t tag : facet) points.push_back(point_key(mesh.vertices.at(tag), eps));
        std::sort(points.begin(), points.end());
        std::ostringstream key;
        key << facet.size() << ':';
        for (const auto& point : points) key << point << ';';
        result.insert(key.str());
    }
    return result;
}

std::string canonical_hash(const Mesh& mesh)
{
    const auto [lo, hi] = bounds(mesh);
    const double eps = std::max(1e-10, 1e-8 * (hi - lo).norm());
    uint64_t hash = 1469598103934665603ULL;
    const auto consume = [&hash](const std::string& value) {
        for (const unsigned char c : value) {
            hash ^= c;
            hash *= 1099511628211ULL;
        }
        hash ^= 0xff;
        hash *= 1099511628211ULL;
    };
    for (const auto& cell : canonical_cells(mesh, eps)) consume(cell);
    for (const auto& group : mesh.physical_groups) consume(group);
    std::ostringstream stream;
    stream << std::hex << std::setfill('0') << std::setw(16) << hash;
    return stream.str();
}

double tet_volume(const Vector3d& a, const Vector3d& b, const Vector3d& c, const Vector3d& d)
{
    return std::abs((b - a).dot((c - a).cross(d - a))) / 6;
}

double cell_volume(const Mesh& mesh, const Cell& cell)
{
    std::vector<Vector3d> p;
    for (const size_t v : cell.vertices) p.push_back(mesh.vertices.at(v));
    if (cell.type == 4) return tet_volume(p[0], p[1], p[2], p[3]);
    if (cell.type == 6) {
        return tet_volume(p[0], p[1], p[2], p[3]) + tet_volume(p[1], p[2], p[3], p[4]) +
               tet_volume(p[2], p[3], p[4], p[5]);
    }
    return tet_volume(p[0], p[1], p[2], p[4]) + tet_volume(p[0], p[2], p[3], p[4]);
}

std::vector<Facet> cell_facets(const Cell& cell)
{
    const auto& v = cell.vertices;
    if (cell.type == 4) {
        return {{v[0], v[1], v[2]}, {v[0], v[1], v[3]}, {v[0], v[2], v[3]}, {v[1], v[2], v[3]}};
    }
    if (cell.type == 6) {
        return {
            {v[0], v[1], v[2]},
            {v[3], v[4], v[5]},
            {v[0], v[1], v[4], v[3]},
            {v[1], v[2], v[5], v[4]},
            {v[2], v[0], v[3], v[5]}};
    }
    return {
        {v[0], v[1], v[2], v[3]},
        {v[0], v[1], v[4]},
        {v[1], v[2], v[4]},
        {v[2], v[3], v[4]},
        {v[3], v[0], v[4]}};
}

std::vector<Facet> boundary_facets(const Mesh& mesh, bool& conforming)
{
    std::map<Facet, std::pair<size_t, Facet>> counts;
    for (const auto& cell : mesh.cells) {
        for (const auto& facet : cell_facets(cell)) {
            Facet key = facet;
            std::sort(key.begin(), key.end());
            auto& [count, oriented] = counts[key];
            ++count;
            oriented = facet;
        }
    }

    conforming = true;
    std::vector<Facet> result;
    for (const auto& [_, value] : counts) {
        conforming = conforming && value.first <= 2;
        if (value.first == 1) result.push_back(value.second);
    }
    return result;
}

std::vector<Triangle> boundary_triangles(const Mesh& mesh, bool& conforming)
{
    std::vector<Triangle> result;
    for (const auto& facet : boundary_facets(mesh, conforming)) {
        result.push_back(
            {{mesh.vertices.at(facet[0]), mesh.vertices.at(facet[1]), mesh.vertices.at(facet[2])}});
        if (facet.size() == 4) {
            result.push_back(
                {{mesh.vertices.at(facet[0]),
                  mesh.vertices.at(facet[2]),
                  mesh.vertices.at(facet[3])}});
        }
    }
    return result;
}

double point_triangle_squared_distance(const Vector3d& p, const Triangle& triangle)
{
    const Vector3d& a = triangle[0];
    const Vector3d& b = triangle[1];
    const Vector3d& c = triangle[2];
    const Vector3d ab = b - a;
    const Vector3d ac = c - a;
    const Vector3d ap = p - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0) return ap.squaredNorm();

    const Vector3d bp = p - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3) return bp.squaredNorm();

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const double v = d1 / (d1 - d3);
        return (p - (a + v * ab)).squaredNorm();
    }

    const Vector3d cp = p - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6) return cp.squaredNorm();

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const double w = d2 / (d2 - d6);
        return (p - (a + w * ac)).squaredNorm();
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + w * (c - b))).squaredNorm();
    }

    const Vector3d normal = ab.cross(ac);
    if (normal.squaredNorm() == 0) {
        return std::min({ap.squaredNorm(), bp.squaredNorm(), cp.squaredNorm()});
    }
    return std::pow(normal.dot(ap), 2) / normal.squaredNorm();
}

double directed_boundary_distance(const Mesh& source, const Mesh& target)
{
    bool source_conforming = false;
    bool target_conforming = false;
    const auto source_facets = boundary_facets(source, source_conforming);
    const auto target_triangles = boundary_triangles(target, target_conforming);
    if (source_facets.empty() || target_triangles.empty()) {
        return source_facets.empty() == target_triangles.empty()
                   ? 0
                   : std::numeric_limits<double>::infinity();
    }

    std::set<size_t> source_vertices;
    for (const auto& facet : source_facets) {
        source_vertices.insert(facet.begin(), facet.end());
    }
    double max_squared_distance = 0;
    for (const size_t tag : source_vertices) {
        const Vector3d& p = source.vertices.at(tag);
        double min_squared_distance = std::numeric_limits<double>::infinity();
        for (const auto& triangle : target_triangles) {
            min_squared_distance =
                std::min(min_squared_distance, point_triangle_squared_distance(p, triangle));
        }
        max_squared_distance = std::max(max_squared_distance, min_squared_distance);
    }
    return std::sqrt(max_squared_distance);
}

double boundary_distance(const Mesh& a, const Mesh& b)
{
    return std::max(directed_boundary_distance(a, b), directed_boundary_distance(b, a));
}

double tet_amips(std::array<Vector3d, 4> p)
{
    if ((p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0])) < 0) {
        std::swap(p[2], p[3]);
    }
    std::array<double, 12> coordinates;
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 3; ++j) coordinates[3 * i + j] = p[i][j];
    }
    return wmtk::AMIPS_energy(coordinates);
}

double max_amips(const Mesh& mesh)
{
    double result = 0;
    const auto accumulate = [&result](const std::array<Vector3d, 4>& tet) {
        const double energy = tet_amips(tet);
        if (std::isfinite(energy)) result = std::max(result, energy);
    };
    for (const auto& cell : mesh.cells) {
        std::vector<Vector3d> p;
        for (const size_t v : cell.vertices) p.push_back(mesh.vertices.at(v));
        if (cell.type == 4) {
            accumulate({p[0], p[1], p[2], p[3]});
        } else if (cell.type == 6) {
            accumulate({p[0], p[1], p[2], p[3]});
            accumulate({p[1], p[2], p[3], p[4]});
            accumulate({p[2], p[3], p[4], p[5]});
        } else {
            accumulate({p[0], p[1], p[2], p[4]});
            accumulate({p[0], p[2], p[3], p[4]});
        }
    }
    return result;
}

bool valid(const Mesh& mesh)
{
    for (const auto& cell : mesh.cells) {
        if (cell.type == 4) {
            std::array<Vector3d, 4> p;
            for (size_t i = 0; i < 4; ++i) p[i] = mesh.vertices.at(cell.vertices[i]);
            if (!is_valid_tet(p) && !is_valid_tet({p[0], p[1], p[3], p[2]})) return false;
        } else if (cell.type == 6) {
            std::array<Vector3d, 6> p;
            for (size_t i = 0; i < 6; ++i) p[i] = mesh.vertices.at(cell.vertices[i]);
            if (!is_valid_prism(p)) return false;
        } else if (cell.type == 7) {
            std::array<Vector3d, 5> p;
            for (size_t i = 0; i < 5; ++i) p[i] = mesh.vertices.at(cell.vertices[i]);
            if (!is_valid_pyramid(p)) return false;
        }
    }
    return true;
}

nlohmann::json metrics(const Mesh& mesh)
{
    const auto [lo, hi] = bounds(mesh);
    double volume = 0;
    size_t tets = 0;
    size_t prisms = 0;
    size_t pyramids = 0;
    bool conforming = false;
    const auto boundary = boundary_facets(mesh, conforming);
    for (const auto& cell : mesh.cells) {
        volume += cell_volume(mesh, cell);
        tets += cell.type == 4;
        prisms += cell.type == 6;
        pyramids += cell.type == 7;
    }
    return {
        {"vertices", mesh.vertices.size()},
        {"tets", tets},
        {"prisms", prisms},
        {"pyramids", pyramids},
        {"volume", volume},
        {"bbox_min", {lo.x(), lo.y(), lo.z()}},
        {"bbox_max", {hi.x(), hi.y(), hi.z()}},
        {"canonical_hash", canonical_hash(mesh)},
        {"valid", valid(mesh)},
        {"conforming", conforming},
        {"boundary_facets", boundary.size()},
        {"max_amips", max_amips(mesh)}};
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 4) {
        std::cerr << "usage: " << argv[0] << " OLD.msh NEW.msh exact|semantic\n";
        return 2;
    }
    const std::string tier = argv[3];
    if (tier != "exact" && tier != "semantic") {
        std::cerr << "tier must be exact or semantic\n";
        return 2;
    }

    const Mesh old_mesh = load(argv[1]);
    const Mesh new_mesh = load(argv[2]);
    const auto old_metrics = metrics(old_mesh);
    const auto new_metrics = metrics(new_mesh);
    const auto [old_lo, old_hi] = bounds(old_mesh);
    const auto [new_lo, new_hi] = bounds(new_mesh);
    const double diagonal = std::max((old_hi - old_lo).norm(), (new_hi - new_lo).norm());
    const double eps = std::max(1e-10, 1e-8 * diagonal);

    std::vector<std::string> failures;
    if (tier == "exact") {
        if (!old_metrics["valid"].get<bool>() || !new_metrics["valid"].get<bool>()) {
            failures.push_back("one or both meshes contain invalid cells");
        }
        if (!old_metrics["conforming"].get<bool>() || !new_metrics["conforming"].get<bool>()) {
            failures.push_back("one or both meshes are non-conforming");
        }
        if (canonical_cells(old_mesh, eps) != canonical_cells(new_mesh, eps)) {
            failures.push_back("canonical cell multisets differ");
        }
        if (canonical_boundary(old_mesh, eps) != canonical_boundary(new_mesh, eps)) {
            failures.push_back("canonical boundary facet multisets differ");
        }
        if (old_mesh.physical_groups != new_mesh.physical_groups) {
            failures.push_back("physical groups differ");
        }
    } else {
        if (!old_metrics["valid"].get<bool>() || !new_metrics["valid"].get<bool>()) {
            failures.push_back("one or both meshes contain invalid cells");
        }
        if (!old_metrics["conforming"].get<bool>() || !new_metrics["conforming"].get<bool>()) {
            failures.push_back("one or both meshes are non-conforming");
        }
        const double old_volume = old_metrics["volume"];
        const double new_volume = new_metrics["volume"];
        const double relative_volume =
            std::abs(old_volume - new_volume) / std::max(1e-30, std::abs(old_volume));
        if (relative_volume > 1e-6) failures.push_back("relative volume differs by more than 1e-6");
        if (boundary_distance(old_mesh, new_mesh) > 1e-6 * diagonal) {
            failures.push_back("sampled boundary Hausdorff distance exceeds 1e-6 diagonal");
        }
        const double old_ratio =
            old_mesh.cells.empty()
                ? 0
                : static_cast<double>(old_metrics["prisms"].get<size_t>()) / old_mesh.cells.size();
        const double new_ratio =
            new_mesh.cells.empty()
                ? 0
                : static_cast<double>(new_metrics["prisms"].get<size_t>()) / new_mesh.cells.size();
        if (new_ratio + 0.02 < old_ratio) {
            failures.push_back("new prism ratio is more than two percentage points lower");
        }
        const double old_amips = old_metrics["max_amips"];
        const double new_amips = new_metrics["max_amips"];
        if (std::isfinite(old_amips) && std::isfinite(new_amips) &&
            new_amips > std::max(1.05 * old_amips, old_amips + 1e-8)) {
            failures.push_back("new maximum finite AMIPS exceeds the allowed bound");
        }
    }

    const nlohmann::json report = {
        {"equivalent", failures.empty()},
        {"tier", tier},
        {"old", old_metrics},
        {"new", new_metrics},
        {"boundary_hausdorff", boundary_distance(old_mesh, new_mesh)},
        {"failures", failures}};
    std::cout << report.dump(2) << '\n';
    return failures.empty() ? 0 : 1;
}
