# Try to find the GNU Multiple Precision Arithmetic Library (GMP)
# See http://gmplib.org/

if(TARGET gmp)
    return()
endif()


find_package(MPFR 2.3.0)
find_package(GMP)

add_library(gmp INTERFACE)
target_include_directories(gmp INTERFACE ${GMP_INCLUDES} ${MPFR_INCLUDES})
target_link_libraries(gmp INTERFACE ${GMP_LIBRARIES} ${MPFR_LIBRARIES})