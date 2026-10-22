#include "mesh_io.hpp"
#include "read_stl.hpp"

#include <igl/read_triangle_mesh.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <paraviewo/VTUWriter.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::io {
namespace {
void require(bool ok, const std::string& s)
{
    if (!ok) throw std::runtime_error("mesh I/O: " + s);
}
paraviewo::CellType paraview_type(CellType type)
{
    switch (type) {
    case CellType::Point: return paraviewo::CellType::Vertex;
    case CellType::Edge: return paraviewo::CellType::Line;
    case CellType::Triangle: return paraviewo::CellType::Triangle;
    case CellType::Quad: return paraviewo::CellType::Quadrilateral;
    case CellType::Tetrahedron: return paraviewo::CellType::Tetrahedron;
    case CellType::Prism: return paraviewo::CellType::Wedge;
    case CellType::Pyramid: return paraviewo::CellType::Pyramid;
    }
    throw std::runtime_error("mesh I/O: unsupported cell type");
}
Eigen::MatrixXd vtu_field(const std::string& name, const MeshAttribute& attribute)
{
    // Adapt to the existing writer's Float64 fields, without adding a new VTU encoding.
    require(attribute.labels.empty(), "VTU writer does not support attribute labels: " + name);
    require(attribute.cols() != 2, "VTU writer pads two-column attributes: " + name);
    require(
        name.find_first_of("\"<>&\r\n\t") == std::string::npos,
        "VTU writer cannot escape attribute name: " + name);
    Eigen::MatrixXd values = attribute.doubles();
    require(
        !((values.array().abs() > 0) && (values.array().abs() < 1e-16)).any(),
        "VTU writer would round small attribute values to zero: " + name);
    return values;
}
void write_vtu(const std::filesystem::path& path, const MeshData& mesh)
{
    paraviewo::VTUWriter writer;
    require(mesh.V.rows() <= std::numeric_limits<int>::max(), "VTU vertex count exceeds int range");
    for (const auto& [name, attribute] : mesh.vertex_attributes)
        writer.add_field(name, vtu_field(name, attribute));

    std::vector<paraviewo::CellElement> cells;
    const MeshAttributes* schema = nullptr;
    for (const auto& [type, block] : mesh.cells) {
        if (block.connectivity.rows() == 0) {
            require(block.attributes.empty(), "VTU cannot preserve attributes on an empty block");
            continue;
        }
        if (!schema) schema = &block.attributes;
        require(
            block.attributes.size() == schema->size(),
            "VTU requires the same attributes on every cell block");
        for (const auto& [name, attribute] : *schema) {
            const auto it = block.attributes.find(name);
            require(
                it != block.attributes.end() && it->second.cols() == attribute.cols(),
                "VTU requires matching cell attribute names and widths");
        }
        for (Eigen::Index i = 0; i < block.connectivity.rows(); ++i) {
            paraviewo::CellElement cell;
            cell.ctype = paraview_type(type);
            for (Eigen::Index j = 0; j < block.connectivity.cols(); ++j)
                cell.vertices.push_back(static_cast<int>(block.connectivity(i, j)));
            cells.push_back(std::move(cell));
        }
    }
    require(
        cells.size() <= size_t(std::numeric_limits<int>::max()),
        "VTU cell count exceeds int range");
    if (schema)
        for (const auto& [name, attribute] : *schema) {
            Eigen::MatrixXd values(cells.size(), attribute.cols());
            Eigen::Index row = 0;
            for (const auto& [type, block] : mesh.cells) {
                if (block.connectivity.rows() == 0) continue;
                const auto part = vtu_field(name, block.attributes.at(name));
                values.middleRows(row, part.rows()) = part;
                row += part.rows();
            }
            writer.add_cell_field(name, values);
        }
    require(writer.write_mesh(path.string(), mesh.V, cells), "cannot write " + path.string());
}
std::string extension(const std::filesystem::path& p)
{
    auto s = p.extension().string();
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return char(std::tolower(c));
    });
    return s;
}
CellType from_gmsh(int t)
{
    switch (t) {
    case 15: return CellType::Point;
    case 1: return CellType::Edge;
    case 2: return CellType::Triangle;
    case 3: return CellType::Quad;
    case 4: return CellType::Tetrahedron;
    case 6: return CellType::Prism;
    case 7: return CellType::Pyramid;
    default:
        throw std::runtime_error("mesh I/O: unsupported Gmsh element type " + std::to_string(t));
    }
}
int gmsh_type(CellType t)
{
    switch (t) {
    case CellType::Point: return 15;
    case CellType::Edge: return 1;
    case CellType::Triangle: return 2;
    case CellType::Quad: return 3;
    case CellType::Tetrahedron: return 4;
    case CellType::Prism: return 6;
    case CellType::Pyramid: return 7;
    }
    throw std::runtime_error("invalid cell type");
}
std::vector<int> physical_tags(const mshio::MshSpec& s, int dim, int tag)
{
    std::vector<int> result;
    auto find = [&](const auto& entities) {
        for (const auto& e : entities)
            if (e.tag == tag) result = e.physical_group_tags;
    };
    switch (dim) {
    case 0: find(s.entities.points); break;
    case 1: find(s.entities.curves); break;
    case 2: find(s.entities.surfaces); break;
    case 3: find(s.entities.volumes); break;
    }
    return result;
}
MeshAttribute typed_attribute(DataMatrix<double> values, const nlohmann::json& meta)
{
    MeshAttribute a;
    a.values = std::move(values);
    if (meta.contains("labels")) a.labels = meta["labels"].get<std::vector<std::string>>();
    if (meta.value("type", 0) == 1) a.values = a.integers();
    if (meta.value("type", 0) == 2) {
        auto m = a.integers();
        require((m.array() >= 0).all() && (m.array() <= 255).all(), "invalid UInt8 field");
        a.values = DataMatrix<uint8_t>(m.cast<uint8_t>());
    }
    return a;
}
} // namespace

MeshData from_msh(const mshio::MshSpec& s)
{
    require(s.element_node_data.empty(), "ElementNodeData is not supported");
    MeshData out;
    std::unordered_map<size_t, int64_t> nodes;
    out.V.resize(s.nodes.num_nodes, 3);
    int64_t row = 0;
    bool nonstandard_ids = false;
    DataMatrix<int64_t> source_ids(s.nodes.num_nodes, 1);
    for (const auto& b : s.nodes.entity_blocks) {
        require(
            !b.parametric && b.data.size() == b.tags.size() * 3,
            "unsupported or malformed node block");
        for (size_t i = 0; i < b.tags.size(); ++i) {
            require(
                b.tags[i] > 0 && b.tags[i] - 1 <= size_t(std::numeric_limits<int64_t>::max()),
                "node tag exceeds Int64 source ID range");
            require(nodes.emplace(b.tags[i], row).second, "duplicate node tag");
            nonstandard_ids = nonstandard_ids || b.tags[i] != size_t(row) + 1;
            if (row < source_ids.rows()) source_ids(row, 0) = int64_t(b.tags[i] - 1);
            require(row < out.V.rows(), "node count mismatch");
            for (int j = 0; j < 3; ++j) out.V(row, j) = b.data[i * 3 + j];
            ++row;
        }
    }
    require(row == out.V.rows(), "node count mismatch");
    if (nonstandard_ids) out.vertex_attributes["vid"].values = std::move(source_ids);
    using Ref = std::pair<CellType, int64_t>;
    std::unordered_map<size_t, Ref> elements;
    std::map<std::pair<CellType, std::vector<int64_t>>, int64_t> unique;
    std::map<CellType, std::vector<std::set<std::string>>> groups;
    // Historical MshData exports repeat cells in separate physical-group blocks and
    // mark those blocks with empty node blocks. Other files retain cell multiplicity.
    const bool group_copies =
        !s.physical_groups.empty() &&
        std::any_of(s.nodes.entity_blocks.begin(), s.nodes.entity_blocks.end(), [](const auto& b) {
            return b.num_nodes_in_block == 0;
        });
    for (const auto& b : s.elements.entity_blocks) {
        const auto type = from_gmsh(b.element_type);
        const int width = cell_size(type);
        require(
            b.data.size() == b.num_elements_in_block * size_t(width + 1),
            "malformed element block");
        std::set<std::string> names;
        for (int tag : physical_tags(s, b.entity_dim, b.entity_tag))
            for (const auto& g : s.physical_groups)
                if (g.dim == b.entity_dim && g.tag == tag) names.insert(g.name);
        for (size_t i = 0; i < b.num_elements_in_block; ++i) {
            std::vector<int64_t> vertices(width);
            for (int j = 0; j < width; ++j) {
                auto n = nodes.find(b.data[i * (width + 1) + j + 1]);
                require(n != nodes.end(), "unknown node tag");
                vertices[j] = n->second;
            }
            auto key = vertices;
            std::sort(key.begin(), key.end());
            auto& c = out.cells[type].connectivity;
            int64_t r = c.rows();
            bool added = true;
            if (group_copies) {
                auto entry = unique.emplace(std::make_pair(type, key), r);
                r = entry.first->second;
                added = entry.second;
            }
            if (added) {
                c.conservativeResize(c.rows() + 1, width);
                for (int j = 0; j < width; ++j) c(r, j) = vertices[j];
                groups[type].emplace_back();
            }
            groups[type][r].insert(names.begin(), names.end());
            require(
                elements.emplace(b.data[i * (width + 1)], Ref{type, r}).second,
                "duplicate element tag");
        }
    }
    // Element tags retain writer row order even when entities are grouped by membership.
    for (auto& [type, block] : out.cells) {
        std::vector<std::pair<size_t, int64_t>> ordered;
        std::map<int64_t, size_t> first;
        for (const auto& [tag, ref] : elements)
            if (ref.first == type) {
                auto it = first.find(ref.second);
                if (it == first.end() || tag < it->second) first[ref.second] = tag;
            }
        for (const auto& [r, tag] : first) ordered.emplace_back(tag, r);
        std::sort(ordered.begin(), ordered.end());
        auto connectivity = block.connectivity;
        auto memberships = groups[type];
        std::vector<int64_t> remap(ordered.size());
        for (size_t i = 0; i < ordered.size(); ++i) {
            const auto old = ordered[i].second;
            block.connectivity.row(i) = connectivity.row(old);
            groups[type][i] = memberships[old];
            remap[old] = i;
        }
        for (auto& [tag, ref] : elements)
            if (ref.first == type) ref.second = remap[ref.second];
    }
    for (const auto& [type, rows] : groups) {
        std::set<std::string> names;
        for (const auto& r : rows) names.insert(r.begin(), r.end());
        if (names.empty()) continue;
        MeshAttribute a;
        // Columns follow the file's physical-group order, not the names' sort order: readers
        // that number regions by position (the simwild .msh route) must see the same ids.
        for (const auto& g : s.physical_groups)
            if (names.count(g.name) &&
                std::find(a.labels.begin(), a.labels.end(), g.name) == a.labels.end())
                a.labels.push_back(g.name);
        DataMatrix<uint8_t> values = DataMatrix<uint8_t>::Zero(rows.size(), names.size());
        for (size_t i = 0; i < rows.size(); ++i)
            for (size_t j = 0; j < a.labels.size(); ++j) values(i, j) = rows[i].count(a.labels[j]);
        a.values = std::move(values);
        out.cells[type].attributes["tags"] = std::move(a);
    }
    auto read_fields = [&](const auto& data, bool vertex) {
        std::map<std::pair<int, std::string>, std::pair<DataMatrix<double>, nlohmann::json>> fields;
        for (const auto& d : data) {
            require(
                !d.header.string_tags.empty() && d.header.int_tags.size() >= 3,
                "malformed data header");
            nlohmann::json meta = nlohmann::json::object();
            if (d.header.string_tags.size() > 1 && d.header.string_tags[1].rfind("wmtk:", 0) == 0)
                meta = nlohmann::json::parse(d.header.string_tags[1].substr(5));
            const auto name = meta.value("name", d.header.string_tags[0]);
            const int width = meta.value("width", d.header.int_tags[1]),
                      col = meta.value("column", 0);
            require(
                width > 0 && col >= 0 && col + d.header.int_tags[1] <= width,
                "invalid field width");
            for (const auto& e : d.entries) {
                require(e.data.size() == size_t(d.header.int_tags[1]), "invalid data entry width");
                int owner = 0;
                int64_t r = 0, n = out.V.rows();
                if (vertex) {
                    require(nodes.count(e.tag), "unknown data node");
                    r = nodes.at(e.tag);
                } else {
                    require(elements.count(e.tag), "unknown data element");
                    auto ref = elements.at(e.tag);
                    owner = int(ref.first);
                    r = ref.second;
                    n = out.cells[ref.first].connectivity.rows();
                }
                auto& f = fields[{owner, name}];
                if (f.first.size() == 0) {
                    f.first = DataMatrix<double>::Zero(n, width);
                    f.second = meta;
                }
                require(f.first.cols() == width, "inconsistent field width");
                for (size_t j = 0; j < e.data.size(); ++j) f.first(r, col + j) = e.data[j];
                if (meta.contains("special")) {
                    const auto key = std::to_string(e.tag);
                    if (meta["special"].contains(key)) {
                        const auto token = meta["special"][key].template get<std::string>();
                        f.first(r, col) = token == "nan" ? std::numeric_limits<double>::quiet_NaN()
                                          : token == "inf"
                                              ? std::numeric_limits<double>::infinity()
                                              : -std::numeric_limits<double>::infinity();
                    }
                }
            }
        }
        for (auto& [key, f] : fields) {
            auto a = typed_attribute(std::move(f.first), f.second);
            if (vertex)
                out.vertex_attributes[key.second] = std::move(a);
            else
                out.cells[static_cast<CellType>(key.first)].attributes[key.second] = std::move(a);
        }
    };
    read_fields(s.node_data, true);
    read_fields(s.element_data, false);
    out.validate();
    return out;
}

mshio::MshSpec to_msh(const MeshData& mesh)
{
    mesh.validate();
    mshio::MshSpec s;
    s.mesh_format.file_type = 1;
    if (mesh.V.rows()) {
        mshio::NodeBlock b;
        b.entity_dim = 0;
        b.entity_tag = 1;
        b.num_nodes_in_block = mesh.V.rows();
        for (Eigen::Index i = 0; i < mesh.V.rows(); ++i) {
            b.tags.push_back(i + 1);
            for (int j = 0; j < 3; ++j) b.data.push_back(mesh.V(i, j));
        }
        s.nodes.entity_blocks.push_back(std::move(b));
        s.nodes.num_entity_blocks = 1;
        s.nodes.num_nodes = mesh.V.rows();
        s.nodes.min_node_tag = 1;
        s.nodes.max_node_tag = mesh.V.rows();
    }
    size_t next = 1;
    auto fields = [&](const MeshAttributes& attrs,
                      const std::vector<size_t>& tags,
                      std::vector<mshio::Data>& target) {
        for (const auto& [name, a] : attrs) {
            const auto values = a.doubles();
            for (Eigen::Index col = 0; col < values.cols(); ++col) {
                mshio::Data d;
                nlohmann::json meta = {
                    {"name", name},
                    {"width", values.cols()},
                    {"column", col},
                    {"type", a.values.index()},
                    {"labels", a.labels}};
                for (size_t r = 0; r < tags.size(); ++r)
                    if (!std::isfinite(values(r, col)))
                        meta["special"][std::to_string(tags[r])] = std::isnan(values(r, col))
                                                                       ? "nan"
                                                                   : values(r, col) > 0 ? "inf"
                                                                                        : "-inf";
                d.header.string_tags = {
                    values.cols() == 1 ? name : name + "_" + std::to_string(col),
                    "wmtk:" + meta.dump()};
                d.header.real_tags = {0};
                d.header.int_tags = {0, 1, int(tags.size())};
                for (size_t r = 0; r < tags.size(); ++r) {
                    mshio::DataEntry e;
                    e.tag = tags[r];
                    e.data = {std::isfinite(values(r, col)) ? values(r, col) : 0.0};
                    d.entries.push_back(std::move(e));
                }
                target.push_back(std::move(d));
            }
        }
    };
    std::vector<size_t> vtags(mesh.V.rows());
    for (size_t i = 0; i < vtags.size(); ++i) vtags[i] = i + 1;
    fields(mesh.vertex_attributes, vtags, s.node_data);
    for (const auto& [type, c] : mesh.cells) {
        std::vector<size_t> tags;
        // One entity per membership set preserves overlapping physical groups without duplicate
        // cells.
        std::map<std::vector<int>, mshio::ElementBlock> blocks;
        std::vector<int> group_ids;
        auto ta = c.attributes.find("tags");
        DataMatrix<int64_t> membership;
        if (ta != c.attributes.end()) {
            require(
                ta->second.labels.size() == size_t(ta->second.cols()),
                "tags need column labels");
            membership = ta->second.integers();
            for (const auto& name : ta->second.labels) {
                const int id = int(s.physical_groups.size()) + 1;
                group_ids.push_back(id);
                s.physical_groups.push_back({cell_dimension(type), id, name});
            }
        }
        for (Eigen::Index r = 0; r < c.connectivity.rows(); ++r) {
            std::vector<int> groups;
            for (size_t j = 0; j < group_ids.size(); ++j)
                if (membership(r, j)) groups.push_back(group_ids[j]);
            auto& b = blocks[groups];
            b.entity_dim = cell_dimension(type);
            b.element_type = gmsh_type(type);
            ++b.num_elements_in_block;
            b.data.push_back(next);
            tags.push_back(next++);
            for (Eigen::Index j = 0; j < c.connectivity.cols(); ++j)
                b.data.push_back(size_t(c.connectivity(r, j)) + 1);
        }
        for (auto& [groups, b] : blocks) {
            b.entity_tag = int(s.elements.entity_blocks.size()) + 1;
            auto entity = [&](auto& es) {
                using E = typename std::decay_t<decltype(es)>::value_type;
                E e;
                e.tag = b.entity_tag;
                e.physical_group_tags = groups;
                if constexpr (std::is_same_v<E, mshio::PointEntity>) {
                    const auto v = b.data.at(1) - 1;
                    e.x = mesh.V(v, 0);
                    e.y = mesh.V(v, 1);
                    e.z = mesh.V(v, 2);
                } else if (mesh.V.rows()) {
                    const auto lo = mesh.V.colwise().minCoeff().eval(),
                               hi = mesh.V.colwise().maxCoeff().eval();
                    e.min_x = lo[0];
                    e.min_y = lo[1];
                    e.min_z = lo[2];
                    e.max_x = hi[0];
                    e.max_y = hi[1];
                    e.max_z = hi[2];
                }
                es.push_back(e);
            };
            switch (b.entity_dim) {
            case 0: entity(s.entities.points); break;
            case 1: entity(s.entities.curves); break;
            case 2: entity(s.entities.surfaces); break;
            case 3: entity(s.entities.volumes); break;
            }
            s.elements.entity_blocks.push_back(std::move(b));
        }
        fields(c.attributes, tags, s.element_data);
    }
    s.elements.num_entity_blocks = s.elements.entity_blocks.size();
    s.elements.num_elements = next - 1;
    s.elements.min_element_tag = next > 1 ? 1 : 0;
    s.elements.max_element_tag = next - 1;
    return s;
}

MeshData read_mesh(const std::filesystem::path& path)
{
    const auto ext = extension(path);
    if (ext == ".msh") return from_msh(mshio::load_msh(path.string()));
    require(ext != ".vtu", "VTU input remains component-specific");
    MeshData out;
    if (ext == ".obj") {
        std::ifstream file(path);
        require(bool(file), "cannot read " + path.string());
        std::vector<Eigen::Vector3d> vertices, normals;
        std::vector<Eigen::Vector2d> uv;
        std::map<CellType, std::vector<std::vector<int64_t>>> cells;
        std::vector<std::vector<int64_t>> ft, fn;
        std::string line;
        auto index = [](const std::string& s, size_t n) {
            const auto v = std::stoll(s);
            require(v != 0, "OBJ index is zero");
            const int64_t i = v > 0 ? v - 1 : int64_t(n) + v;
            require(i >= 0, "OBJ relative index out of range");
            return i;
        };
        while (std::getline(file, line)) {
            std::istringstream in(line);
            std::string kind;
            in >> kind;
            if (kind == "v" || kind == "vn") {
                Eigen::Vector3d v = Eigen::Vector3d::Zero();
                require(bool(in >> v[0] >> v[1]), "invalid OBJ vertex");
                in >> v[2];
                (kind == "v" ? vertices : normals).push_back(v);
            } else if (kind == "vt") {
                Eigen::Vector2d v;
                require(bool(in >> v[0] >> v[1]), "invalid OBJ UV");
                uv.push_back(v);
            } else if (kind == "f" || kind == "l" || kind == "p") {
                std::vector<int64_t> ids, ti, ni;
                std::string token;
                while (in >> token) {
                    if (token[0] == '#') break;
                    auto a = token.find('/'),
                         b = token.find('/', a == std::string::npos ? token.size() : a + 1);
                    ids.push_back(index(token.substr(0, a), vertices.size()));
                    ti.push_back(
                        a != std::string::npos && b != a + 1 && a + 1 < token.size()
                            ? index(token.substr(a + 1, b - a - 1), uv.size())
                            : -1);
                    ni.push_back(
                        b != std::string::npos && b + 1 < token.size()
                            ? index(token.substr(b + 1), normals.size())
                            : -1);
                }
                if (kind == "f") {
                    require(ids.size() >= 3, "invalid OBJ face");
                    for (size_t j = 1; j + 1 < ids.size(); ++j) {
                        cells[CellType::Triangle].push_back({ids[0], ids[j], ids[j + 1]});
                        ft.push_back({ti[0], ti[j], ti[j + 1]});
                        fn.push_back({ni[0], ni[j], ni[j + 1]});
                    }
                } else if (kind == "l")
                    for (size_t j = 1; j < ids.size(); ++j)
                        cells[CellType::Edge].push_back({ids[j - 1], ids[j]});
                else
                    for (auto id : ids) cells[CellType::Point].push_back({id});
            }
        }
        out.V.resize(vertices.size(), 3);
        for (size_t i = 0; i < vertices.size(); ++i) out.V.row(i) = vertices[i];
        for (const auto& [type, rows] : cells) {
            auto& c = out.cells[type].connectivity;
            c.resize(rows.size(), cell_size(type));
            for (size_t i = 0; i < rows.size(); ++i)
                for (size_t j = 0; j < rows[i].size(); ++j) c(i, j) = rows[i][j];
        }
        auto corners = [&](const auto& source,
                           const auto& ids,
                           const std::string& name,
                           int width) {
            if (source.empty() || ids.empty()) return;
            // A per-corner field needs a value on every corner. Exporters commonly write
            // vt/vn records that the faces do not (all) reference; skip the field then,
            // as other OBJ readers do, rather than reject the geometry.
            for (const auto& corner : ids)
                for (const auto id : corner)
                    if (id < 0) {
                        logger().warn(
                            "OBJ {}: not every face corner references a {} value; the {} "
                            "field is not read",
                            path.string(),
                            name,
                            name);
                        return;
                    }
            DataMatrix<double> values(ids.size(), 3 * width);
            for (size_t i = 0; i < ids.size(); ++i)
                for (int j = 0; j < 3; ++j) {
                    require(size_t(ids[i][j]) < source.size(), "invalid OBJ " + name + " index");
                    for (int k = 0; k < width; ++k) values(i, j * width + k) = source[ids[i][j]][k];
                }
            out.cells[CellType::Triangle].attributes[name].values = std::move(values);
        };
        corners(uv, ft, "uv", 2);
        corners(normals, fn, "normal", 3);
    } else {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        if (ext == ".stl")
            read_stl(path.string(), V, F);
        else
            require(igl::read_triangle_mesh(path.string(), V, F), "cannot read " + path.string());
        out.V = V;
        out.cells[CellType::Triangle].connectivity = F.cast<int64_t>();
    }
    out.validate();
    return out;
}
void write_mesh(const std::filesystem::path& path, const MeshData& mesh)
{
    const auto ext = extension(path);
    mesh.validate();
    if (ext == ".msh") {
        auto s = to_msh(mesh);
        mshio::validate_spec(s);
        mshio::save_msh(path.string(), s);
    } else if (ext == ".vtu")
        write_vtu(path, mesh);
    else
        throw std::runtime_error("mesh I/O: output must be .msh or .vtu");
}
} // namespace wmtk::io
