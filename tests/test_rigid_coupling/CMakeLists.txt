set(TARGET_NAME "test_rigid_coupling")

file(GLOB_RECURSE TARGET_HEADER
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cuh
)

file(GLOB_RECURSE TARGET_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

add_executable(${TARGET_NAME} ${TARGET_SRC} ${COMMON_SRC})

target_sources(${TARGET_NAME} PRIVATE ${PROJECT_HEADER} ${COMMON_HEADER} ${TARGET_HEADER})

source_group(
    TREE "${PROJECT_SOURCE_DIR}/tests/common"
    PREFIX "Header Files"
    FILES ${TARGET_HEADER}
)

source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}"
    PREFIX "Source Files"
    FILES ${TARGET_SRC}
)

target_include_directories(${TARGET_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/tools)

target_include_directories(${TARGET_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/core)
target_include_directories(${TARGET_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/engine)
target_include_directories(${TARGET_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/geometry_utils)
target_include_directories(${TARGET_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/math)
target_include_directories(${TARGET_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/solver)
target_include_directories(${TARGET_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/utils)

target_include_directories(${TARGET_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/extern/glad/include)
target_include_directories(${TARGET_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/extern/glad/include/glad)
target_include_directories(${TARGET_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/extern/glad/include/KHR)
target_include_directories(${TARGET_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/extern/GLFW/include)

if(WIN32)
    target_link_directories(${TARGET_NAME} PRIVATE ${GLFW_LIBRARY_DIR})
    target_link_libraries(${TARGET_NAME} PRIVATE glfw3_mt)
elseif(UNIX)
    find_package(glfw3 REQUIRED)
    target_link_libraries(${TARGET_NAME} PRIVATE glfw)
endif()

target_link_libraries(${TARGET_NAME} PRIVATE Eigen3::Eigen glad crmpm_engine)

set_target_properties(${TARGET_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
