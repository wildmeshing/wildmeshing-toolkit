#include "AttributeDescription.hpp"
#include <nlohmann/json.hpp>


namespace wmtk::components::multimesh::utils {

    namespace {
        NLOHMANN_JSON_SERIALIZE_ENUM(PrimitiveType ,\
            {\
                {PrimitiveType::Vertex, 0},\
                {PrimitiveType::Edge, 1},\
                {PrimitiveType::Triangle, 2},\
                {PrimitiveType::Tetrahedron, 3},\
            })
        NLOHMANN_JSON_SERIALIZE_ENUM(attribute::AttributeType,\
            {\
                {attribute::AttributeType::Char, "char"},\
                {attribute::AttributeType::Double, "double"},\
                {attribute::AttributeType::Int64, "int"},\
                {attribute::AttributeType::Rational, "rational"},\
            })
    }
    WMTK_NLOHMANN_JSON_FRIEND_TO_JSON_PROTOTYPE(AttributeDescription) {


        WMTK_NLOHMANN_ASSIGN_TYPE_TO_JSON(path, dimension, type);
    }

    PrimitiveType AttributeDescription::primitive_type() const {
        assert(dimension < 4);

        return get_primitive_type_from_id(dimension);
    }

}
