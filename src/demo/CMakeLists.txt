include_directories(${PROJECT_SOURCE_DIR})

include_directories(${PROJECT_BINARY_DIR})

file(GLOB_RECURSE CXX_FILES
        ${PROJECT_SOURCE_DIR}/src/math/*.cc
)

add_executable(demo_logger demo_logger.cc)
add_executable(demo_rotate ${CXX_FILES} demo_rotate.cc)
target_link_libraries(demo_rotate PUBLIC ${PROJECT_LINK_LIBRARIES})
