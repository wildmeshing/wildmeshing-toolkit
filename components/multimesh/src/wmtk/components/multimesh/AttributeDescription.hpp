#pragma once
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/attribute/AttributeType.hpp>

namespace wmtk::components::multimesh {


// the minimal information to uniquely extract an attribute handle
struct AttributeDescription
{
    std::string path;
    PrimitiveType primitive_type;
    attribute::AttributeType type;
};
} // namespace wmtk::components::multimesh
