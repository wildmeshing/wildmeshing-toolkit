#include "MeshData.hpp"

#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <type_traits>

namespace wmtk {
namespace {
void require(bool ok, const std::string& message)
{
    if (!ok) throw std::runtime_error("MeshData: " + message);
}
void validate_attributes(const MeshAttributes& attributes, Eigen::Index rows)
{
    for (const auto& [name, a] : attributes) {
        require(!name.empty(), "empty attribute name");
        require(a.rows() == rows && a.cols() > 0, "invalid shape for attribute " + name);
        require(
            a.labels.empty() || a.labels.size() == size_t(a.cols()),
            "invalid labels for " + name);
        require(
            std::set<std::string>(a.labels.begin(), a.labels.end()).size() == a.labels.size(),
            "duplicate labels for " + name);
    }
}
} // namespace
Eigen::Index MeshAttribute::rows() const
{
    return std::visit([](const auto& m) { return m.rows(); }, values);
}
Eigen::Index MeshAttribute::cols() const
{
    return std::visit([](const auto& m) { return m.cols(); }, values);
}
DataMatrix<double> MeshAttribute::doubles() const
{
    return std::visit(
        [](const auto& m) -> DataMatrix<double> {
            using S = typename std::decay_t<decltype(m)>::Scalar;
            DataMatrix<double> out = m.template cast<double>();
            if constexpr (std::is_same_v<S, int64_t>) {
                for (Eigen::Index i = 0; i < m.size(); ++i) {
                    const auto d = out.data()[i];
                    require(
                        d >= -9223372036854775808.0 && d < 9223372036854775808.0 &&
                            static_cast<int64_t>(d) == m.data()[i],
                        "lossy conversion to double");
                }
            }
            return out;
        },
        values);
}
DataMatrix<int64_t> MeshAttribute::integers() const
{
    return std::visit(
        [](const auto& m) -> DataMatrix<int64_t> {
            using S = typename std::decay_t<decltype(m)>::Scalar;
            if constexpr (!std::is_same_v<S, double>)
                return m.template cast<int64_t>();
            else {
                DataMatrix<int64_t> out(m.rows(), m.cols());
                for (Eigen::Index i = 0; i < m.size(); ++i) {
                    const auto x = m.data()[i];
                    require(
                        std::isfinite(x) && std::floor(x) == x && x >= -9223372036854775808.0 &&
                            x < 9223372036854775808.0,
                        "invalid integer attribute");
                    out.data()[i] = static_cast<int64_t>(x);
                }
                return out;
            }
        },
        values);
}
int cell_size(CellType t)
{
    switch (t) {
    case CellType::Point: return 1;
    case CellType::Edge: return 2;
    case CellType::Triangle: return 3;
    case CellType::Quad:
    case CellType::Tetrahedron: return 4;
    case CellType::Prism: return 6;
    case CellType::Pyramid: return 5;
    }
    throw std::runtime_error("MeshData: unsupported cell type");
}
int cell_dimension(CellType t)
{
    switch (t) {
    case CellType::Point: return 0;
    case CellType::Edge: return 1;
    case CellType::Triangle:
    case CellType::Quad: return 2;
    default: cell_size(t); return 3;
    }
}
const CellData& MeshData::cell(CellType t) const
{
    auto it = cells.find(t);
    require(it != cells.end(), "missing cell block " + std::to_string(int(t)));
    return it->second;
}
void MeshData::validate() const
{
    require(V.cols() == 3 && V.allFinite(), "V must have three finite coordinate columns");
    validate_attributes(vertex_attributes, V.rows());
    for (const auto& [type, block] : cells) {
        const auto& c = block.connectivity;
        require(c.cols() == cell_size(type), "incorrect connectivity width");
        // A cell may repeat a vertex. Degenerate cells occur in real input files, and the
        // components remove them during cleaning exactly as the file readers do.
        require(
            c.size() == 0 || (c.minCoeff() >= 0 && c.maxCoeff() < V.rows()),
            "vertex index out of range");
        validate_attributes(block.attributes, c.rows());
    }
}
} // namespace wmtk
