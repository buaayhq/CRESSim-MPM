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

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/tools)

add_subdirectory(example_c_api)
add_subdirectory(example_cutting)
add_subdirectory(example_object_creation)
