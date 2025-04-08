###############################################################################
# Create Python module for Wildmeshing Toolkit
include(pybind11)
include(pybind11_json)

pybind11_add_module(pywmtk)

add_subdirectory(src) # Add sources to wmtk
target_include_directories(pywmtk PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

target_link_libraries(pywmtk PRIVATE 
  Eigen3::Eigen
  # pybind11::module
  pybind11::json
  wmtk::simplicial_embedding
)

# Extra warnings
# target_link_libraries(pywmtk PRIVATE wmtk::warnings)