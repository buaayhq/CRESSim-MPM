add_library(glad STATIC glad.c)

# set_target_properties(glad PROPERTIES LINKER_LANGUAGE C)

target_include_directories(glad PUBLIC
    ${PROJECT_SOURCE_DIR}/extern/glad/include
)
