# hdf5 (https://github.com/HDFGroup/hdf5)
# License: ???

if(TARGET hdf5::hdf5)
    return()
endif()

message(STATUS "Third-party: creating target 'hdf5'")

option(HDF5_GENERATE_HEADERS "" OFF)
option(HDF5_BUILD_EXAMPLES "" OFF)
option(HDF5_BUILD_TOOLS "" OFF)
option(HDF5_BUILD_UTILS "" OFF)
option(HDF5_BUILD_HL_TOOLS "" OFF)
option(HDF5_BUILD_HL_LIB "" ON)
option(HDF5_TEST_CPP "" OFF)
option(HDF5_TEST_EXAMPLES "" OFF)
option(HDF5_TEST_FORTRAN "" OFF)
option(HDF5_TEST_JAVA "" OFF)
option(HDF5_TEST_PARALLEL "" OFF)
option(HDF5_TEST_SERIAL "" OFF)
option(HDF5_TEST_SWMR "" OFF)
option(HDF5_TEST_TOOLS "" OFF)
option(HDF5_TEST_VFD "" OFF)
option(HDF5_ENABLE_ALL_WARNINGS "" OFF)
option(HDF5_ENABLE_EMBEDDED_LIBINFO "" OFF)
#To prevent changes in the oput dirs
set (HDF5_EXTERNALLY_CONFIGURED 1)

include(CPM)
set(HDF5_RELEASE_TAG hdf5-1_14_3)

#we fetch the zip file to get prebuilt files (and avoid a perl dependency)
CPMAddPackage("https://github.com/HDFGroup/hdf5/releases/download/${HDF5_RELEASE_TAG}/${HDF5_RELEASE_TAG}.zip")

target_link_libraries(hdf5-static INTERFACE hdf5_hl-static)
add_library(hdf5::hdf5 ALIAS hdf5-static)