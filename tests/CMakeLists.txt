file(GLOB_RECURSE PROJECT_HEADER
    ${PROJECT_SOURCE_DIR}/include/core/*.h
    ${PROJECT_SOURCE_DIR}/include/core/*.cuh
    ${PROJECT_SOURCE_DIR}/include/math/*.h
    ${PROJECT_SOURCE_DIR}/include/math/*.cuh
    ${PROJECT_SOURCE_DIR}/include/utils/*.h
    ${PROJECT_SOURCE_DIR}/include/utils/*.cuh
    ${PROJECT_SOURCE_DIR}/include/solver/*.h
    ${PROJECT_SOURCE_DIR}/include/solver/*.cuh
    ${PROJECT_SOURCE_DIR}/include/engine/*.h
    ${PROJECT_SOURCE_DIR}/include/engine/*.cuh
    ${PROJECT_SOURCE_DIR}/include/geometry_utils/*.h
    ${PROJECT_SOURCE_DIR}/include/geometry_utils/*.cuh
)

file(GLOB_RECURSE COMMON_HEADER
    ${PROJECT_SOURCE_DIR}/tools/*.h
    ${PROJECT_SOURCE_DIR}/tools/*.cuh
)

file(GLOB_RECURSE INTERNAL_HEADER
    ${PROJECT_SOURCE_DIR}/src/core/*.h
    ${PROJECT_SOURCE_DIR}/src/core/*.cuh
    ${PROJECT_SOURCE_DIR}/src/solver/*.h
    ${PROJECT_SOURCE_DIR}/src/solver/*.cuh
    ${PROJECT_SOURCE_DIR}/src/engine/*.h
    ${PROJECT_SOURCE_DIR}/src/engine/*.cuh
    ${PROJECT_SOURCE_DIR}/src/utils/*.h
    ${PROJECT_SOURCE_DIR}/src/utils/*.cuh
)
file(GLOB_RECURSE COMMON_SRC
    ${PROJECT_SOURCE_DIR}/tools/*.cpp
    ${PROJECT_SOURCE_DIR}/tools/*.cu
)

source_group(
    TREE "${PROJECT_SOURCE_DIR}/include"
    PREFIX "Header Files"
    FILES ${PROJECT_HEADER}
)

source_group(
    TREE "${PROJECT_SOURCE_DIR}/src"
    PREFIX "Header Files"
    FILES ${INTERNAL_HEADER}
)

source_group(
    TREE "${PROJECT_SOURCE_DIR}/tools"
    PREFIX "Header Files"
    FILES ${COMMON_HEADER}
)

source_group(
    TREE "${PROJECT_SOURCE_DIR}/tools"
    PREFIX "Source Files"
    FILES ${COMMON_SRC}
)

add_subdirectory(test_svd)
add_subdirectory(test_trimesh)
add_subdirectory(test_sdf_geometry)
add_subdirectory(test_object_creation)
add_subdirectory(test_multi_scene)
add_subdirectory(test_rigid_collision)
add_subdirectory(test_c_api)
add_subdirectory(test_connected_line)
add_subdirectory(test_rigid_coupling)
add_subdirectory(test_benchmark)

if(ENGINE_STATIC)
    # Thread-related classes are not exported,
    # so we only test them in static builds
    add_subdirectory(test_thread)
endif()
