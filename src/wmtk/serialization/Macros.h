#pragma once
// datatype is missing an include to numeric
#include <numeric>

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wcast-qual"
#pragma clang diagnostic ignored "-Wswitch-enum"
#elif (defined(__GNUC__) || defined(__GNUG__)) && !(defined(__clang__) || defined(__INTEL_COMPILER))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wswitch-enum"
#endif
#include <highfive/H5DataType.hpp>
#if defined(__clang__)
#pragma clang diagnostic pop
#elif (defined(__GNUC__) || defined(__GNUG__)) && !(defined(__clang__) || defined(__INTEL_COMPILER))
#pragma GCC diagnostic pop
#endif


// Macro for pre-declaring HighFive's HDF5 datatypes
// HighFive defines a macro HIGHFIVE_REGISTER_TYPE that implements a template specialization.
// That macro cannot be used to fully specialize in a header so this macro is
// provided so specialization can be declared in a header and implementation
// performed in a source file
#define WMTK_HIGHFIVE_DECLARE_TYPE(type) \
    template <>                          \
    HighFive::DataType HighFive::create_datatype<type>();

// For any new Attribute type we want to record we have to register the type
// using HighFive's HIGHFIVE_REGISTER_TYPE and then declare its attribute type.
// For example, if we want to record a AttributeCollection<Vec>
// Where Vec is
// struct Vec { double x,y; };
// Then we must declare:
//
// HighFive::CompoundType vec_datatype() {
//      return {
//              {"x", create_datatype<double>()},
//              {"y", create_datatype<double>()}
//      };
// }
//
// and then outside of any namespace's scope we declare the following two lines
// HIGHFIVE_REGISTER_TYPE(Vec, vec_datatype)
// WMTK_HDF5_REGISTER_ATTRIBUTE_TYPE(Vec)
#define WMTK_HDF5_REGISTER_ATTRIBUTE_TYPE(type)     \
    HIGHFIVE_REGISTER_TYPE(                         \
        wmtk::AttributeCollectionValueChange<type>, \
        wmtk::AttributeCollectionValueChange<type>::datatype)

#define WMTK_HDF5_DECLARE_ATTRIBUTE_TYPE(type) \
    WMTK_HIGHFIVE_DECLARE_TYPE(wmtk::AttributeCollectionValueChange<type>)

