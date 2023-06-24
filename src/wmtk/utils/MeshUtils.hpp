#pragma once

#include <wmtk/Mesh.hpp>

#include <string>

namespace wmtk {
class MeshUtils
{
private:
    MeshUtils() = default;

public:
    template <typename Mat>
    static MeshAttributeHandle<typename Mat::Scalar> set_matrix_attribute(
        const Mat& data,
        const std::string& name,
        const PrimitiveType& type,
        Mesh& mesh)
    {
        MeshAttributeHandle<typename Mat::Scalar> handle =
            mesh.register_attribute<typename Mat::Scalar>(name, type, data.cols());

        auto accessor = mesh.create_accessor(handle);
        const auto tuples = mesh.get_all(type);
        for (size_t i = 0; i < tuples.size(); ++i) {
            const auto& t = tuples[i];
            accessor.vector_attribute(t) = data.row(i).transpose();
        }

        return handle;
    }
};
} // namespace wmtk