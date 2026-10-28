#include "read_mesh.hpp"

#include <mshio/mshio.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace wmtk::components::prismatic_mesh::detail {
namespace {
constexpr int64_t max_source_id = 9007199254740991LL;
using RowMap = std::unordered_map<size_t, size_t>;

// mshio's section scanner checks eof(), but not fail(). A failed numeric read can
// otherwise spin forever. Enable failbit exceptions on a bounded stream ending at
// the final non-whitespace character, so normal EOF is set while reading the last
// $End... token (rather than by an extra, failed extraction). Keep large files streamed.
class CheckedMshBuffer : public std::streambuf
{
public:
    explicit CheckedMshBuffer(std::ifstream& file)
        : m_file(file)
    {
        file.seekg(0, std::ios::end);
        m_remaining = file.tellg();
        while (m_remaining > 0) {
            file.seekg(m_remaining - 1);
            char c = 0;
            file.get(c);
            if (!std::isspace(static_cast<unsigned char>(c))) break;
            --m_remaining;
        }
        file.clear();
        file.seekg(0);
    }

protected:
    int_type underflow() override
    {
        if (gptr() != egptr()) return traits_type::to_int_type(*gptr());
        if (m_remaining <= 0) return traits_type::eof();
        const auto count = std::min<std::streamoff>(m_remaining, m_buffer.size());
        m_file.read(m_buffer.data(), count);
        if (m_file.gcount() != count) throw std::ios_base::failure("truncated MSH stream");
        m_remaining -= count;
        setg(m_buffer.data(), m_buffer.data(), m_buffer.data() + count);
        return traits_type::to_int_type(*gptr());
    }

private:
    std::ifstream& m_file;
    std::streamoff m_remaining = 0;
    std::array<char, 65536> m_buffer;
};

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error("prismatic_mesh MSH: " + message);
}

int64_t integer(double value, int64_t lo, int64_t hi, const std::string& name)
{
    require(
        std::isfinite(value) && value >= lo && value <= hi && std::floor(value) == value,
        "invalid " + name);
    return static_cast<int64_t>(value);
}

// DataEntry.tag is a Gmsh node/element tag, not an array row or source vid.
std::optional<std::vector<int64_t>> scalar_field(
    const std::vector<mshio::Data>& fields,
    const std::string& name,
    const RowMap& rows,
    int64_t lo,
    int64_t hi)
{
    const mshio::Data* field = nullptr;
    for (const auto& candidate : fields) {
        if (candidate.header.string_tags.empty() || candidate.header.string_tags.front() != name)
            continue;
        require(field == nullptr, "duplicate field " + name);
        field = &candidate;
    }
    if (!field) return std::nullopt;
    const auto& tags = field->header.int_tags;
    require(tags.size() >= 3 && tags[1] == 1, "expected scalar field " + name);
    require(
        tags[2] >= 0 && static_cast<size_t>(tags[2]) == rows.size() &&
            field->entries.size() == rows.size(),
        "field must cover every node/element: " + name);
    std::vector<int64_t> values(rows.size());
    std::vector<bool> seen(rows.size(), false);
    for (const auto& entry : field->entries) {
        const auto it = rows.find(entry.tag);
        require(it != rows.end(), "unknown node/element tag in " + name);
        const size_t row = it->second;
        require(!seen[row], "duplicate node/element tag in " + name);
        require(entry.data.size() == 1, "expected scalar data in " + name);
        seen[row] = true;
        values[row] = integer(entry.data[0], lo, hi, name);
    }
    return values;
}

// Resolve semantic names through PhysicalNames -> Entities -> element blocks.
// Numeric physical IDs differ between the construction exporter and PRISM.
std::map<int, int> volume_regions(const mshio::MshSpec& spec)
{
    std::map<int, int> physical;
    for (const auto& group : spec.physical_groups) {
        if (group.dim != 3) continue;
        int region = 0;
        if (group.name == "tag_0" || group.name == "input") region = 1;
        if (group.name == "offset" || group.name == "offset_band") region = 2;
        if (group.name == "ambient" || group.name == "background") region = 3;
        require(physical.emplace(group.tag, region).second, "duplicate volume physical tag");
    }
    std::map<int, int> regions;
    for (const auto& entity : spec.entities.volumes) {
        int region = 0;
        for (int tag : entity.physical_group_tags) {
            const auto it = physical.find(tag);
            if (it == physical.end() || it->second == 0) continue;
            require(region == 0 || region == it->second, "conflicting volume physical groups");
            region = it->second;
        }
        require(regions.emplace(entity.tag, region).second, "duplicate volume entity tag");
    }
    return regions;
}
} // namespace

PrismaticMeshInput load_prismatic_msh(const std::filesystem::path& path)
{
    std::ifstream stream(path, std::ios::binary);
    require(stream.good(), "cannot read " + path.string());
    std::string marker, version;
    int binary = -1, data_size = 0;
    stream >> marker >> version >> binary >> data_size;
    require(
        stream.good() && marker == "$MeshFormat" && version == "4.1" &&
            (binary == 0 || binary == 1) && data_size == sizeof(size_t),
        "expected Gmsh 4.1 ASCII or native-endian binary with 8-byte data size");
    stream.clear();
    stream.seekg(0);
    CheckedMshBuffer buffer(stream);
    std::istream checked(&buffer);
    checked.exceptions(std::ios::failbit | std::ios::badbit);
    const auto spec = [&] {
        try {
            return mshio::load_msh(checked);
        } catch (const std::ios_base::failure&) {
            throw std::runtime_error(
                "prismatic_mesh MSH: malformed or truncated file " + path.string());
        }
    }();
    const size_t n = spec.nodes.num_nodes, m = spec.elements.num_elements;
    require(n > 0 && n <= std::numeric_limits<int>::max(), "invalid node count");
    require(m > 0 && m <= std::numeric_limits<int>::max(), "invalid element count");
    PrismaticMeshInput result;
    result.vertices.resize(n, 3);
    result.tetrahedra.resize(m, 4);
    result.source_vertex_ids.resize(n);
    RowMap node_rows, element_rows;
    node_rows.reserve(n);
    element_rows.reserve(m);
    size_t row = 0;
    for (const auto& block : spec.nodes.entity_blocks) {
        require(block.parametric == 0, "parametric nodes are not supported");
        require(
            block.tags.size() == block.num_nodes_in_block &&
                block.data.size() == 3 * block.num_nodes_in_block &&
                block.num_nodes_in_block <= n - row,
            "invalid node block size");
        for (size_t i = 0; i < block.num_nodes_in_block; ++i, ++row) {
            const size_t tag = block.tags[i];
            require(tag > 0 && tag <= static_cast<size_t>(max_source_id) + 1, "invalid node tag");
            require(node_rows.emplace(tag, row).second, "duplicate node tag");
            // Construction correspondence uses zero-based source IDs. A vid field,
            // when present (e.g. PRISM tet exports), overrides this convention.
            result.source_vertex_ids[row] = static_cast<int64_t>(tag - 1);
            for (int j = 0; j < 3; ++j) {
                const double value = block.data[3 * i + j];
                require(std::isfinite(value), "non-finite coordinate");
                result.vertices(row, j) = value;
            }
        }
    }
    require(row == n, "node count does not match blocks");
    std::vector<std::array<size_t, 4>> tets(m);
    std::vector<int> entities(m);
    std::set<std::array<size_t, 4>> seen_tets;
    row = 0;
    for (const auto& block : spec.elements.entity_blocks) {
        require(
            block.entity_dim == 3 && block.element_type == 4,
            "only four-node tetrahedra (Gmsh type 4) are supported; decompose hybrid cells first");
        require(
            block.data.size() == 5 * block.num_elements_in_block &&
                block.num_elements_in_block <= m - row,
            "invalid element block size");
        for (size_t i = 0; i < block.num_elements_in_block; ++i, ++row) {
            const size_t tag = block.data[5 * i];
            require(
                tag > 0 && element_rows.emplace(tag, row).second,
                "invalid or duplicate element tag");
            entities[row] = block.entity_tag;
            for (int j = 0; j < 4; ++j) {
                const auto it = node_rows.find(block.data[5 * i + j + 1]);
                require(it != node_rows.end(), "tetrahedron references unknown node tag");
                tets[row][j] = it->second;
                result.tetrahedra(row, j) = static_cast<int>(it->second);
            }
            auto sorted = tets[row];
            std::sort(sorted.begin(), sorted.end());
            require(
                std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end(),
                "tetrahedron contains repeated vertices");
            require(seen_tets.insert(sorted).second, "duplicate tetrahedron");
        }
    }
    require(row == m, "element count does not match blocks");
    const auto input = scalar_field(spec.element_data, "tag_0", element_rows, 0, 1);
    const auto offset = scalar_field(spec.element_data, "offset_tag", element_rows, -1, 1);
    require(
        input.has_value() == offset.has_value(),
        "tag_0 and offset_tag fields must both be present or both absent");
    result.input_cells.resize(m, 0);
    result.offset_tet_tags.resize(m, -1);
    const auto regions = input ? std::map<int, int>{} : volume_regions(spec);
    for (size_t i = 0; i < m; ++i) {
        if (input) {
            result.input_cells[i] = static_cast<int>((*input)[i]);
            result.offset_tet_tags[i] = (*offset)[i] == 1 ? 1 : -1;
        } else {
            const auto it = regions.find(entities[i]);
            require(
                it != regions.end() && it->second != 0,
                "missing input/band/background physical group for volume entity");
            result.input_cells[i] = it->second == 1 ? 1 : 0;
            result.offset_tet_tags[i] = it->second == 2 ? 1 : -1;
        }
    }
    if (auto ids = scalar_field(spec.node_data, "vid", node_rows, 0, max_source_id))
        result.source_vertex_ids = std::move(*ids);
    auto corr = scalar_field(spec.node_data, "corr_input_vid", node_rows, -1, max_source_id);
    require(corr.has_value(), "missing corr_input_vid NodeData");
    result.corr_input_vid = std::move(*corr);
    result.vertex_tags.resize(n, -1);
    if (auto labels = scalar_field(spec.node_data, "labels", node_rows, -1, 2)) {
        for (size_t i = 0; i < n; ++i)
            result.vertex_tags[i] = (*labels)[i] == 0 ? -1 : static_cast<int>((*labels)[i]);
    } else {
        for (size_t i = 0; i < m; ++i)
            if (result.input_cells[i] == 1)
                for (auto v : tets[i]) result.vertex_tags[v] = 1;
        for (size_t i = 0; i < n; ++i) {
            if (result.corr_input_vid[i] < 0) continue;
            require(result.vertex_tags[i] != 1, "correspondence on an input-volume vertex");
            result.vertex_tags[i] = 2;
        }
    }
    std::unordered_map<int64_t, size_t> source_rows;
    source_rows.reserve(n);
    result.corr_input_vertex.resize(n, -1);
    result.input_to_offset_vertices.resize(n);
    for (size_t i = 0; i < n; ++i) {
        require(source_rows.emplace(result.source_vertex_ids[i], i).second, "duplicate vid");
        if (result.vertex_tags[i] == 1) result.input_vertices.push_back(i);
        if (result.vertex_tags[i] == 2)
            result.offset_vertices.push_back(i);
        else
            require(result.corr_input_vid[i] == -1, "correspondence on a non-offset vertex");
    }
    for (auto v : result.offset_vertices) {
        const auto it = source_rows.find(result.corr_input_vid[v]);
        require(
            it != source_rows.end(),
            "offset vertex has missing or unknown input correspondence");
        require(
            result.vertex_tags[it->second] == 1,
            "correspondence target is not an input vertex");
        result.corr_input_vertex[v] = static_cast<int64_t>(it->second);
        result.input_to_offset_vertices[it->second].push_back(v);
    }
    result.mesh = std::make_unique<TetMesh>();
    result.mesh->init_with_isolated_vertices(n, tets);
    return result;
}
} // namespace wmtk::components::prismatic_mesh::detail
