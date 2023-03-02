#pragma once
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wcast-qual"
#pragma clang diagnostic ignored "-Wswitch-enum"
#elif (defined(__GNUC__) || defined(__GNUG__)) && !(defined(__clang__) || defined(__INTEL_COMPILER))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wswitch-enum"
#endif
#include <highfive/H5DataSet.hpp>
#include <highfive/H5DataType.hpp>
#include <highfive/H5File.hpp>
#if defined(__clang__)
#pragma clang diagnostic pop
#elif (defined(__GNUC__) || defined(__GNUG__)) && !(defined(__clang__) || defined(__INTEL_COMPILER))
#pragma GCC diagnostic pop
#endif
namespace wmtk {
// checks whether the file holding the dataset exists
bool does_dataset_exist(const HighFive::File& file, const std::string& name);
HighFive::DataSet
create_dataset(HighFive::File& file, const std::string& name, const HighFive::DataType& datatype);
template <typename T>
HighFive::DataSet create_dataset(HighFive::File& file, const std::string& name)
{
    return create_dataset(file, name, HighFive::create_datatype<T>());
}

} // namespace wmtk
