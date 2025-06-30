set_target_properties(ImGui-SFML PROPERTIES
    CXX_CLANG_TIDY ""
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ImGui-SFML::ImGui-SFML
)


# Manually list each subdirectory (replaces GLOB_RECURSE)
add_subdirectory(controller)
add_subdirectory(model)
add_subdirectory(view)

# Include current dir's files (if any)
file(GLOB SRC_FILES "*.cpp" "*.h")
target_sources(${PROJECT_NAME} PRIVATE ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
