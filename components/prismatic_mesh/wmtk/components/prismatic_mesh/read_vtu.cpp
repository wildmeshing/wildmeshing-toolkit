#include "prismatic_mesh.hpp"

#include <tinyxml2.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace wmtk::components::prismatic_mesh {
namespace {
using tinyxml2::XMLElement;
void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error("prismatic_mesh VTU: " + message);
}
std::string attr(const XMLElement* e, const char* key, const char* fallback = "")
{
    const char* value = e->Attribute(key);
    return value ? value : fallback;
}
const XMLElement* child(const XMLElement* e, const char* name)
{
    const auto* result = e->FirstChildElement(name);
    require(result != nullptr, std::string("missing ") + name);
    return result;
}
const XMLElement* array(const XMLElement* e, const char* name)
{
    for (auto* a = e->FirstChildElement("DataArray"); a; a = a->NextSiblingElement("DataArray")) {
        if (attr(a, "Name") == name) return a;
    }
    throw std::runtime_error(std::string("prismatic_mesh VTU: missing array ") + name);
}
std::vector<unsigned char> decode(const std::string& text)
{
    const std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string s;
    for (unsigned char c : text)
        if (!std::isspace(c)) s.push_back(c);
    require(s.size() % 4 == 0, "invalid base64 length");
    std::vector<unsigned char> bytes;
    for (size_t i = 0; i < s.size(); i += 4) {
        uint32_t value = 0;
        int padding = 0;
        for (size_t j = 0; j < 4; ++j) {
            value <<= 6;
            if (s[i + j] == '=') {
                require(j >= 2, "invalid base64 padding");
                ++padding;
            } else {
                const auto digit = alphabet.find(s[i + j]);
                require(digit != std::string::npos && padding == 0, "invalid base64 character");
                value |= static_cast<uint32_t>(digit);
            }
        }
        bytes.push_back(static_cast<unsigned char>(value >> 16));
        if (padding < 2) bytes.push_back(static_cast<unsigned char>(value >> 8));
        if (padding < 1) bytes.push_back(static_cast<unsigned char>(value));
    }
    return bytes;
}
template <typename T>
T scalar(const unsigned char* bytes, bool little)
{
    unsigned char buffer[sizeof(T)];
    std::copy(bytes, bytes + sizeof(T), buffer);
    const uint16_t one = 1;
    const bool host_little = *reinterpret_cast<const unsigned char*>(&one) == 1;
    if (little != host_little) std::reverse(buffer, buffer + sizeof(T));
    T value;
    std::memcpy(&value, buffer, sizeof(T));
    return value;
}
struct Reader
{
    bool little;
    bool header64;
    std::vector<long double> read(const XMLElement* a, size_t count, int components = 1) const
    {
        require(
            attr(a, "NumberOfComponents", "1") == std::to_string(components),
            "unexpected component count for " + attr(a, "Name", "Points"));
        const std::string type = attr(a, "type");
        const std::unordered_map<std::string, size_t> sizes = {
            {"Float64", 8},
            {"Float32", 4},
            {"Int64", 8},
            {"UInt64", 8},
            {"Int32", 4},
            {"UInt32", 4},
            {"Int16", 2},
            {"UInt16", 2},
            {"Int8", 1},
            {"UInt8", 1}};
        require(sizes.count(type) != 0, "unsupported scalar type " + type);
        std::vector<long double> values;
        const auto format = attr(a, "format", "ascii");
        if (format == "ascii") {
            std::istringstream stream(a->GetText() ? a->GetText() : "");
            long double v;
            while (stream >> v) values.push_back(v);
            require(stream.eof(), "invalid ASCII number");
        } else {
            require(format == "binary", "only ASCII and inline binary arrays are supported");
            auto bytes = decode(a->GetText() ? a->GetText() : "");
            const size_t header = header64 ? 8 : 4;
            require(bytes.size() >= header, "missing binary length header");
            const uint64_t length = header64 ? scalar<uint64_t>(bytes.data(), little)
                                             : scalar<uint32_t>(bytes.data(), little);
            const size_t width = sizes.at(type);
            require(
                length == bytes.size() - header && length % width == 0,
                "invalid binary payload length");
            for (size_t i = header; i < bytes.size(); i += width) {
                const auto* p = bytes.data() + i;
#define READ_TYPE(name, cpp_type) \
    if (type == name) values.push_back(scalar<cpp_type>(p, little));
                READ_TYPE("Float64", double)
                READ_TYPE("Float32", float)
                READ_TYPE("Int64", int64_t)
                READ_TYPE("UInt64", uint64_t)
                READ_TYPE("Int32", int32_t)
                READ_TYPE("UInt32", uint32_t)
                READ_TYPE("Int16", int16_t)
                READ_TYPE("UInt16", uint16_t)
                READ_TYPE("Int8", int8_t)
                READ_TYPE("UInt8", uint8_t)
#undef READ_TYPE
            }
        }
        require(values.size() == count, "wrong array length for " + attr(a, "Name", "Points"));
        for (auto v : values) require(std::isfinite(v), "non-finite array value");
        return values;
    }
};
int64_t integer(long double v, int64_t min, int64_t max, const char* name)
{
    require(v >= min && v <= max && std::floor(v) == v, std::string("invalid ") + name);
    return static_cast<int64_t>(v);
}
} // namespace

PrismaticMeshInput load_prismatic_mesh(const std::filesystem::path& path)
{
    require(path.extension() == ".vtu", "input must be a .vtu file");
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.string().c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error(
            "prismatic_mesh VTU: cannot read " + path.string() + ": " + doc.ErrorStr());
    }
    const auto* root = doc.FirstChildElement("VTKFile");
    require(root != nullptr, "missing VTKFile");
    require(attr(root, "type") == "UnstructuredGrid", "expected UnstructuredGrid");
    require(attr(root, "compressor").empty(), "compressed VTU is not supported");
    const auto byte_order = attr(root, "byte_order", "LittleEndian");
    require(byte_order == "LittleEndian" || byte_order == "BigEndian", "invalid byte_order");
    const auto header = attr(root, "header_type", "UInt32");
    require(header == "UInt32" || header == "UInt64", "invalid header_type");
    Reader reader{byte_order == "LittleEndian", header == "UInt64"};
    const auto* piece = child(child(root, "UnstructuredGrid"), "Piece");
    require(piece->NextSiblingElement("Piece") == nullptr, "multiple pieces are not supported");
    int n = 0, m = 0;
    require(
        piece->QueryIntAttribute("NumberOfPoints", &n) == tinyxml2::XML_SUCCESS && n > 0,
        "invalid NumberOfPoints");
    require(
        piece->QueryIntAttribute("NumberOfCells", &m) == tinyxml2::XML_SUCCESS && m > 0,
        "invalid NumberOfCells");
    const auto points = reader.read(child(child(piece, "Points"), "DataArray"), size_t(n) * 3, 3);
    const auto* cells = child(piece, "Cells");
    const auto connectivity = reader.read(array(cells, "connectivity"), size_t(m) * 4);
    const auto offsets = reader.read(array(cells, "offsets"), m);
    const auto types = reader.read(array(cells, "types"), m);
    PrismaticMeshInput result;
    result.vertices.resize(n, 3);
    result.tetrahedra.resize(m, 4);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < 3; ++j) {
            const double v = static_cast<double>(points[size_t(i) * 3 + j]);
            require(std::isfinite(v), "coordinate exceeds Float64 range");
            result.vertices(i, j) = v;
        }
    std::set<std::array<size_t, 4>> seen;
    std::vector<std::array<size_t, 4>> tets(m);
    for (int i = 0; i < m; ++i) {
        require(
            types[i] == 10 && offsets[i] == (int64_t(i) + 1) * 4,
            "only four-node tetrahedra (VTK type 10) are supported");
        for (int j = 0; j < 4; ++j) {
            const auto v = integer(connectivity[size_t(i) * 4 + j], 0, n - 1, "vertex index");
            result.tetrahedra(i, j) = static_cast<int>(v);
            tets[i][j] = static_cast<size_t>(v);
        }
        auto sorted = tets[i];
        std::sort(sorted.begin(), sorted.end());
        require(
            std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end(),
            "tetrahedron contains repeated vertices");
        require(seen.insert(sorted).second, "duplicate tetrahedron");
    }
    const auto* pd = child(piece, "PointData");
    const auto labels = reader.read(array(pd, "labels"), n);
    const auto ids = reader.read(array(pd, "vid"), n);
    const auto corr = reader.read(array(pd, "corr_input_vid"), n);
    std::unordered_map<int64_t, size_t> id_to_row;
    result.input_to_offset_vertices.resize(n);
    result.corr_input_vertex.resize(n, -1);
    for (int i = 0; i < n; ++i) {
        const int label = static_cast<int>(integer(labels[i], -1, 2, "labels"));
        // Source IDs are stored as Float64 in the construction export; restrict to exact integers.
        const auto id = integer(ids[i], 0, 9007199254740991LL, "vid");
        const auto target = integer(corr[i], -1, 9007199254740991LL, "corr_input_vid");
        require(id_to_row.emplace(id, i).second, "duplicate vid");
        result.vertex_tags.push_back(label == 0 ? -1 : label);
        result.source_vertex_ids.push_back(id);
        result.corr_input_vid.push_back(target);
        if (label == 1) result.input_vertices.push_back(i);
        if (label == 2) result.offset_vertices.push_back(i);
        require(label == 2 || target == -1, "correspondence on a non-offset vertex");
    }
    for (size_t i : result.offset_vertices) {
        const auto it = id_to_row.find(result.corr_input_vid[i]);
        require(it != id_to_row.end(), "offset vertex has missing or unknown input correspondence");
        require(result.vertex_tags[it->second] == 1, "correspondence target is not an input vertex");
        result.corr_input_vertex[i] = static_cast<int64_t>(it->second);
        result.input_to_offset_vertices[it->second].push_back(i);
    }
    const auto* cd = child(piece, "CellData");
    const auto input = reader.read(array(cd, "tag_0"), m);
    const auto offset = reader.read(array(cd, "offset_tag"), m);
    for (int i = 0; i < m; ++i) {
        result.input_cells.push_back(static_cast<int>(integer(input[i], 0, 1, "tag_0")));
        const auto band = integer(offset[i], -1, 1, "offset_tag");
        result.offset_tet_tags.push_back(band == 1 ? 1 : -1);
    }
    result.mesh = std::make_unique<TetMesh>();
    result.mesh->init(n, tets);
    return result;
}
} // namespace wmtk::components::prismatic_mesh
