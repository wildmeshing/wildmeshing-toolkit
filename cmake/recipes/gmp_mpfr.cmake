if(WIN32)
    find_path(GMP_TRY_FIND_INCLUDES
    NAMES
        gmp.h
    PATHS
        ${INCLUDE_INSTALL_DIR}
    PATH_SUFFIXES
        include
    )

    find_library(GMP_TRY_FIND_LIBRARIES
    NAMES
        gmp
        libgmp-10
    PATHS
        ${LIB_INSTALL_DIR}
    PATH_SUFFIXES
        lib
    )

    if(GMP_TRY_FIND_INCLUDES AND GMP_TRY_FIND_LIBRARIES)
        message(STATUS "System GMP found: ${GMP_TRY_FIND_INCLUDES}")
    else()
        message(WARNING "Third-party: downloading gmp + mpfr. The code may be crazy slow! Please install GMP using your preferred package manager. Do not forget to delete your <build>/CMakeCache.txt for the changes to take effect.")

        include(CPM)
        CPMAddPackage(
            NAME gmp_mpfr
            URL "https://github.com/CGAL/cgal/releases/download/v5.2.1/CGAL-5.2.1-win64-auxiliary-libraries-gmp-mpfr.zip"
            DOWNLOAD_ONLY YES
        )

        # For CGAL and Cork
        set(ENV{GMP_DIR} "${gmp_mpfr_SOURCE_DIR}/gmp")
        set(ENV{MPFR_DIR} "${gmp_mpfr_SOURCE_DIR}/gmp")
    endif()


    
else()
    # On Linux/macOS, gmp+mpfr should be installed system-wide
endif()