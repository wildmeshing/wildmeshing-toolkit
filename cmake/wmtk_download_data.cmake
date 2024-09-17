function(wmtk_download_data DATA_DIR GIT_REPOSITORY GIT_TAG)

    string(MD5 REPO_HASH "${GIT_REPOSITORY}${GIT_TAG}")

    SET(WMTK_DOWNLOAD_DATA_FOLDER ${WMTK_DATA_ROOT}/${REPO_HASH})
    MESSAGE(STATUS "Download data to ${WMTK_DOWNLOAD_DATA_FOLDER} from GIT_REPOSITORY ${GIT_REPOSITORY} GIT_TAG ${GIT_TAG}")

    include(FetchContent)
    FetchContent_Populate(
        wmtk_id
        QUIET
        GIT_REPOSITORY ${GIT_REPOSITORY}
        GIT_TAG ${GIT_TAG}
        SOURCE_DIR ${WMTK_DOWNLOAD_DATA_FOLDER}
    )
    set(DATA_DIR ${WMTK_DOWNLOAD_DATA_FOLDER} PARENT_SCOPE)

    # MESSAGE(STATUS "Download data to ${wmtk_itd_${REPO_HASH}_SRC} from GIT_REPOSITORY ${GIT_REPOSITORY} GIT_TAG ${GIT_TAG}")
    #
    # CPMAddPackage(
    #     NAME wmtk_itd_${REPO_HASH}
    #     GIT_REPOSITORY ${GIT_REPOSITORY}
    #     GIT_TAG ${GIT_TAG}
    #     GIT_SHALLOW TRUE
    #     DOWNLOAD_ONLY TRUE
    # )
    # set(DATA_DIR "${wmtk_itd_${REPO_HASH}_SRC}" PARENT_SCOPE)


endfunction()