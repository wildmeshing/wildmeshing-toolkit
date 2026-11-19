#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <variant>
#include <vector>

namespace wmtk {

// Values agree with VTK's linear cell types. Connectivity always indexes rows of V.
enum class CellType {
    Point = 1,
    Edge = 3,
    Triangle = 5,
    Quad = 9,
    Tetrahedron = 10,
    Prism = 13,
    Pyramid = 14
};
template <typename T>
using DataMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using AttributeValues = std::variant<DataMatrix<double>, DataMatrix<int64_t>, DataMatrix<uint8_t>>;

struct MeshAttribute
{
    AttributeValues values = DataMatrix<double>();
    // Optional labels for columns, e.g. the names of overlapping material tags.
    std::vector<std::string> labels;

    Eigen::Index rows() const;
    Eigen::Index cols() const;
    DataMatrix<double> doubles() const;
    DataMatrix<int64_t> integers() const;
};
using MeshAttributes = std::map<std::string, MeshAttribute>;

struct CellData
{
    DataMatrix<int64_t> connectivity;
    MeshAttributes attributes;
};

// The exact positions of the vertices that could not be rounded: their rows of V hold only the
// rounding, in which cells can be degenerate or inverted. Python's mesh["V_rational"]. Empty when
// every vertex is exactly its row of V.
struct ExactVertices
{
    // Rows of V, unique.
    std::vector<int64_t> rows;
    // rows.size() x V.cols(), row-major: GMP rationals in base 10, "p/q" or "p".
    std::vector<std::string> values;

    bool empty() const { return rows.empty(); }
};

// An owning interchange value, independent of the mutable topology/attribute machinery.
struct MeshData
{
    DataMatrix<double> V{0, 3};
    ExactVertices V_exact;
    MeshAttributes vertex_attributes;
    std::map<CellType, CellData> cells;

    void validate() const;
    const CellData& cell(CellType type) const;
};

using MeshMap = std::map<std::string, MeshData>;
using ComponentResult = std::pair<MeshMap, nlohmann::json>;

int cell_size(CellType type);
int cell_dimension(CellType type);

} // namespace wmtk
