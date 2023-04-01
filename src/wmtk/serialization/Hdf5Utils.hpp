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


// Macro for pre-declaring HighFive's HDF5 datatypes
// HighFive defines a macro HIGHFIVE_REGISTER_TYPE that implements a template specialization.
// That macro cannot be used to fully specialize in a header so this macro is
// provided so specialization can be declared in a header and implementation
// performed in a source file
#define WMTK_HIGHFIVE_DECLARE_TYPE(type) \
    template <>                          \
    inline HighFive::DataType HighFive::create_datatype<type>();


namespace wmtk::utils {


// checks whether the file holding the dataset exists
bool does_dataset_exist(const HighFive::File& file, const std::string& name);


HighFive::DataSet create_extendable_dataset(
    HighFive::File& file,
    const std::string& name,
    const HighFive::DataType& datatype);


template <typename T>
HighFive::DataSet create_extendable_dataset(HighFive::File& file, const std::string& name)
{
    return create_extendable_dataset(file, name, HighFive::create_datatype<T>());
}


// Helper for adding values to the end of a dataset holding a 1d vector takes
// in a dataset and the input data in a vector outputs the start and end
// indices of the data in the dataset
template <typename T, typename Allocator>
std::array<size_t, 2> append_values_to_dataset(
    HighFive::DataSet& dataset,
    const std::vector<T, Allocator>& data);
// takes in a dataset and the input data in a vector
// outputs the start and end indices of the data in the dataset

// Helper for adding a single value to the end of a dataset
// outputs the new size of the dataset
template <typename T>
size_t append_value_to_dataset(HighFive::DataSet& dataset, const T& value);


// implementation

template <typename T, typename Allocator>
std::array<size_t, 2> append_values_to_dataset(
    HighFive::DataSet& dataset,
    const std::vector<T, Allocator>& data)
{
    assert(data.size() != 0);

    // compute the new dataset size
    std::vector<size_t> first_position = dataset.getDimensions();
    // double check that the dataset is 1-dimensional
    assert(first_position.size() == 1);
    std::vector<size_t> new_size{first_position[0] + data.size()};
    std::vector<size_t> count{data.size()};

    // resize the datset to the right size
    dataset.resize(new_size);

    dataset.select(first_position, count).write(data);
    return std::array<size_t, 2>{{first_position[0], new_size[0]}};
}

template <typename T>
size_t append_value_to_dataset(HighFive::DataSet& dataset, const T& value)
{
    // compute the new dataset size
    std::vector<size_t> first_position = dataset.getDimensions();
    // double check that the dataset is 1-dimensional
    assert(first_position.size() == 1);
    std::vector<size_t> new_size{first_position[0] + 1};
    std::vector<size_t> count{1};

    // resize the datset to the right size
    dataset.resize(new_size);

    dataset.select(first_position, count).write_raw(&value);
    return new_size[0];
}

} // namespace wmtk::utils
