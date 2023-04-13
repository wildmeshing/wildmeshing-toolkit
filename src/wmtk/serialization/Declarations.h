#pragma once

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

